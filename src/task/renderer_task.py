# Author: Dominik Bauer
# Vision for Robotics Group, Automation and Control Institute (ACIN)
# TU Wien, Vienna

import OpenGL
OpenGL.ERROR_CHECKING = True
import os
os.environ['PYOPENGL_PLATFORM'] = "egl"
import OpenGL.EGL as egl
import OpenGL.GL as gl
from glumpy import gloo

import ctypes
from ctypes import pointer
import numpy as np

from verefine.renderer import Model, ScreenQuad
import verefine.config as config


class EglRenderer:

    _main_egl_ctx = None

    def __init__(self, dataset, width, height):
        self.width, self.height = width, height

        # egl related
        self.eglDpy, self.eglCfg, self.eglSurf, self.eglCtx = None, None, None, None
        self._setup_egl()
        self._create_egl_context()
        self._activate_egl_context()

        # super().__init__(dataset, width, height)
        self.near, self.far = config.CLIP_NEAR, config.CLIP_FAR
        # FBOs: one per stage
        self.normal_depth_buf = np.zeros((height, width, 4), np.float32).view(gloo.TextureFloat2D)
        self.depth_buf = np.zeros((height, width), np.float32).view(gloo.DepthTexture)
        self.fbo_stage_1 = gloo.FrameBuffer(color=self.normal_depth_buf, depth=self.depth_buf)

        self.cost_buf = np.zeros((height, width, 4), np.float32).view(gloo.TextureFloat2D)
        self.fbo_stage_2 = gloo.FrameBuffer(color=self.cost_buf)

        # load models
        self.dataset = dataset
        self.models = [Model(mesh, offset) for mesh, offset in zip(self.dataset.meshes, self.dataset.obj_model_offset)]
        self.models.append(ScreenQuad())  # used for stage 2

        # VBO: one for all models - bind once, draw according to model indices
        self.vertex_buffer = np.hstack([model.vertex_buffer for model in self.models]).view(gloo.VertexBuffer)
        for model in self.models:
            model.vertex_buffer = self.vertex_buffer

        # shader
        with open("/task/3rdparty/verefine/src/verefine/score.vert", 'r') as file:
            shader_vertex = "".join(file.readlines())
        with open("/task/3rdparty/verefine/src/verefine/score.frag", 'r') as file:
            shader_fragment = "".join(file.readlines())
        self.program = gloo.Program(shader_vertex, shader_fragment)
        self.program['u_texture'] = np.zeros((512, 512, 3), np.float32)
        self.program.bind(self.vertex_buffer)  # is bound once -- saves some time

        gl.glViewport(0, 0, self.width, self.height)
        gl.glClearColor(0.0, 0.0, 0.0, 0.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDisable(gl.GL_CULL_FACE)  # fixes rendering of single-sided lamp mesh in LINEMOD

        self.observation = None

        self._deactivate_egl_context()

    def set_observation(self, observation_d, observation_n):
        if self.observation is not None:
            self.observation.delete()
        self.observation = (np.dstack([observation_n, observation_d]).astype(np.float32)[::-1, :, :])\
            .view(gloo.TextureFloat2D)
        self.program['u_observation'] = self.observation

    def render(self, model_ids, model_trafos, extrinsics, intrinsics, cull_back=True):
        # PREPARE RENDERING
        self._activate_egl_context()
        # compose model matrix:
        # - apply model space offset
        # - apply pose (in camera space)
        # - transform to world space
        # - transpose to be row major (OpenGL)
        mats_off = [self.models[model_id].mat_offset for model_id in model_ids]
        mats_model = [m.copy() for m in model_trafos]

        mat_world2cam = extrinsics.copy()
        R = mat_world2cam[:3, :3].T
        t = np.matmul(-R, mat_world2cam[:3, 3])
        mat_cam2world = np.eye(4)
        mat_cam2world[:3, :3], mat_cam2world[:3, 3] = R, t.ravel()

        mats_model = [(np.matmul(mat_cam2world, np.matmul(mat_model, mat_off))).T for mat_model, mat_off in zip(mats_model, mats_off)]

        # prepare view and projection matrices (row major)
        mat_view = self._compute_view(mat_cam2world)  # gl view matrix from camera matrix
        mat_proj = self._compute_proj(intrinsics)  # projection matrix
        mat_view_proj = np.matmul(mat_view, mat_proj)  # view-projection matrix

        # STAGE 1) compute \hat{d}_T and \hat{n}_T: render model under estimated pose
        self.fbo_stage_1.activate()
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        if not cull_back:
            gl.glDisable(gl.GL_CULL_FACE)  # fixes rendering of single-sided lamp mesh in LINEMOD
        self.program['u_mode'] = 0

        for i, (model_id, m) in enumerate(zip(model_ids, mats_model)):
            self.program['u_mv'] = np.matmul(m, mat_view)
            self.program['u_mvp'] = np.matmul(m, mat_view_proj)

            try:
                model = self.models[model_id]
                model.draw(self.program)
            except ValueError as _:
                print("failed to draw")
                return 0
        if not cull_back:
            gl.glEnable(gl.GL_CULL_FACE)  # enable again for rendering other stuff
        buffer = np.zeros((self.height, self.width, 4), dtype=np.float32)
        gl.glReadPixels(0, 0, self.width, self.height, gl.GL_RGBA, gl.GL_FLOAT, buffer)
        buffer.shape = self.height, self.width, 4
        buffer = buffer[::-1]

        self._deactivate_egl_context()

        return buffer[..., 3], buffer[..., :3]  # depth and normals

    def compute_score(self, model_ids, model_trafos, extrinsics, intrinsics):
        # PREPARE RENDERING
        self._activate_egl_context()
        # compose model matrix:
        # - apply model space offset
        # - apply pose (in camera space)
        # - transform to world space
        # - transpose to be row major (OpenGL)
        mats_off = [self.models[model_id].mat_offset for model_id in model_ids]
        mats_model = [m.copy() for m in model_trafos]

        mat_world2cam = extrinsics.copy()
        R = mat_world2cam[:3, :3].T
        t = np.matmul(-R, mat_world2cam[:3, 3])
        mat_cam2world = np.eye(4)
        mat_cam2world[:3, :3], mat_cam2world[:3, 3] = R, t.ravel()

        mats_model = [(np.matmul(mat_cam2world, np.matmul(mat_model,mat_off))).T for mat_model, mat_off in zip(mats_model, mats_off)]

        # prepare view and projection matrices (row major)
        mat_view = self._compute_view(mat_cam2world)  # gl view matrix from camera matrix
        mat_proj = self._compute_proj(intrinsics)  # projection matrix
        mat_view_proj = np.matmul(mat_view, mat_proj)  # view-projection matrix

        # STAGE 1) compute \hat{d}_T and \hat{n}_T: render model under estimated pose
        self.fbo_stage_1.activate()
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        self.program['u_mode'] = 0

        for i, (model_id, m) in enumerate(zip(model_ids, mats_model)):
            self.program['u_mv'] = np.matmul(m, mat_view)
            self.program['u_mvp'] = np.matmul(m, mat_view_proj)

            try:
                model = self.models[model_id]
                model.draw(self.program)
            except ValueError as _:
                print("failed to draw")
                return 0

        # STAGE 2)
        # - compute sub-scores f_d(T) and f_n(T) per-pixel
        self.fbo_stage_2.activate()
        self.program['u_texture'] = self.normal_depth_buf
        self.program['u_mode'] = 1
        self.program['u_mv'] = np.eye(4)
        self.program['u_mvp'] = np.eye(4)
        self.models[-1].draw(self.program)

        # - compute verification score \bar{f}(T): speed-up summation by reading from mipmap layer
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.cost_buf.handle)
        gl.glGenerateMipmap(gl.GL_TEXTURE_2D)
        layer = 4
        buf = np.zeros((int(self.height / (2 ** layer)), int(self.width / (2 ** layer)), 4), dtype=np.float32)
        per_pixel_fit = gl.glGetTexImage(gl.GL_TEXTURE_2D, layer, gl.GL_RGBA, gl.GL_FLOAT, buf)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)

        self._deactivate_egl_context()

        n_valid = per_pixel_fit[:, :, 0].sum()
        if n_valid == 0:  # no valid pixel -> minimal fit
            return 0
        else:
            mean_f_d = per_pixel_fit[:, :, 1].sum() / n_valid
            mean_f_n = per_pixel_fit[:, :, 2].sum() / n_valid
            return (mean_f_d + mean_f_n) * 0.5

    def _compute_view(self, cam):
        R, t = cam[:3, :3], cam[:3, 3]

        # camera coord axes
        z = np.matmul(R, [0, 0, 1])
        z = -z / np.linalg.norm(z)
        y = np.matmul(R, [0, -1, 0])
        y = np.asarray(y) / np.linalg.norm(y)
        x = np.cross(y, z)
        x = x / np.linalg.norm(x)

        # invert to get view matrix
        view = np.eye(4)
        view[:3, :3] = np.vstack((x, y, z)).T
        view[3, :3] = np.matmul(-view[:3, :3].T, t)

        return view

    def _compute_proj(self, intrinsics):
        width, height = self.width, self.height
        near, far = self.near, self.far
        fx = intrinsics[0, 0]
        fy = intrinsics[1, 1]
        cx = intrinsics[0, 2]
        cy = intrinsics[1, 2]
        s = intrinsics[0, 1]
        q = -(far + near) / (far - near)
        qn = -2 * far * near / (far - near)
        mat_proj = np.array([[2 * fx / width, -2 * s / width, (-2 * cx + width) / width, 0],
                             [0, 2 * fy / height, (2 * cy - height) / height, 0],
                             [0, 0, q, qn],
                             [0, 0, -1, 0]]).T
        return mat_proj

    def _setup_egl(self):
        # see https://developer.nvidia.com/blog/egl-eye-opengl-visualization-without-x-server/ (C++)

        # get display (physical or off-screen)
        self.eglDpy = egl.eglGetDisplay(egl.EGL_DEFAULT_DISPLAY)  # EGLDisplay
        if egl.eglGetError() != int(egl.EGL_SUCCESS):
            print("could not get EGLDisplay")

        # initialize EGL on display
        major, minor = egl.EGLint(), egl.EGLint()
        egl.eglInitialize(self.eglDpy, pointer(major), pointer(minor))
        if egl.eglGetError() != int(egl.EGL_SUCCESS):
            print("could not initialize EGL")

        # select appropriate configuration
        configAttribs = [
            egl.EGL_SURFACE_TYPE, egl.EGL_PBUFFER_BIT, egl.EGL_BLUE_SIZE, 8,
            egl.EGL_GREEN_SIZE, 8, egl.EGL_RED_SIZE, 8, egl.EGL_DEPTH_SIZE, 24,
            egl.EGL_RENDERABLE_TYPE, egl.EGL_OPENGL_BIT, egl.EGL_NONE
        ]
        configAttribs = (egl.EGLint * len(configAttribs))(*configAttribs)
        self.eglCfg = egl.EGLConfig()
        numConfigs = egl.EGLint()

        egl.eglChooseConfig(self.eglDpy, configAttribs, pointer(self.eglCfg), 1, pointer(numConfigs))
        if egl.eglGetError() != int(egl.EGL_SUCCESS):
            print("could not choose EGLConfig")

        # create surface (~ window with buffers)
        width, height = (self.width, self.height)
        pbufferAttribs = [
            egl.EGL_WIDTH,
            width,
            egl.EGL_HEIGHT,
            height,
            egl.EGL_NONE,
        ]
        pbufferAttribs = (egl.EGLint * len(pbufferAttribs))(*pbufferAttribs)

        self.eglSurf = egl.eglCreatePbufferSurface(self.eglDpy, self.eglCfg, pbufferAttribs)  # EGLSurface
        if egl.eglGetError() != int(egl.EGL_SUCCESS):
            print("could not create EGLSurface")

        # bind API
        egl.eglBindAPI(egl.EGL_OPENGL_API)
        if egl.eglGetError() != int(egl.EGL_SUCCESS):
            print("could not bind OpenGL API")

    def _create_egl_context(self):
        if self._main_egl_ctx is None:
            # new context
            self.eglCtx = egl.eglCreateContext(self.eglDpy, self.eglCfg, egl.EGL_NO_CONTEXT, None)
            self._main_egl_ctx = self.eglCtx
        else:
            # setup for context/resource sharing
            # self.eglCtx = egl.eglCreateContext(self.eglDpy, self.eglCfg, self._main_egl_ctx, None)
            self.eglCtx = self._main_egl_ctx
        if egl.eglGetError() != int(egl.EGL_SUCCESS):
            print("could not create EGLContext")

    def _activate_egl_context(self):
        egl.eglMakeCurrent(self.eglDpy, self.eglSurf, self.eglSurf, self.eglCtx)
        if egl.eglGetError() != int(egl.EGL_SUCCESS):
            print("could not make context current")

    def _deactivate_egl_context(self):
        egl.eglMakeCurrent(self.eglDpy, egl.EGL_NO_SURFACE, egl.EGL_NO_SURFACE, egl.EGL_NO_CONTEXT)
        if egl.eglGetError() != int(egl.EGL_SUCCESS):
            print("could not make context current")

    def _destroy_egl_context(self):
        egl.eglDestroyContext(self.eglDpy, self.eglCtx)
        egl.eglTerminate(self.eglDpy)

