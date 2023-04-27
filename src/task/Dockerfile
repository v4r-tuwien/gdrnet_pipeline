# use ros as base image
FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 as nvidia

MAINTAINER Dominik Bauer <bauer@acin.tuwien.ac.at>

# essentials
RUN apt-get update --fix-missing\
    && apt-get install -y --no-install-recommends \
    apt-utils git curl nano tmux unzip wget build-essential cmake python-dev python-pip python-tk

# for opengl/rendering
RUN apt-get install -y libsm6 libglib2.0-0 libgl1-mesa-glx libpcl-dev libglfw3-dev libgles2-mesa-dev \
    libgtk2.0-dev

# -------- install ROS -----------------

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  python-rosdep \
  python-rosinstall \
  python-vcstools

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init  \
  && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get install -y \
  ros-kinetic-ros-base=1.3.2-0*

RUN apt-get update && apt-get install -y --no-install-recommends \
     python-catkin-tools \
     ros-kinetic-tf \
     && rm -rf /var/lib/apt/lists/*

# ----------------

# create a catkin workspace
WORKDIR /task
RUN mkdir -p /task/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /task/catkin_ws; catkin init'
RUN echo "source /task/catkin_ws/devel/setup.bash" >> /root/.bashrc

# clone and build message and service definitions
RUN /bin/bash -c 'cd /task/catkin_ws/src; \
                  git clone https://github.com/v4r-tuwien/object_detector_msgs.git'
RUN /bin/bash -c 'cd /task/catkin_ws/src; \
                  git clone https://gitlab.informatik.uni-bremen.de/robokudo/robokudo_msgs.git'
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /task/catkin_ws; catkin build'

# clone and build ros_numpy
WORKDIR /task/3rdparty
RUN /bin/bash -c 'cd /task/3rdparty; \
                  git clone https://github.com/eric-wieser/ros_numpy.git'
WORKDIR /task/3rdparty/ros_numpy
RUN python setup.py install
RUN /bin/bash -c 'cd /task/3rdparty; \
                  git clone https://github.com/dornik/verefine.git'

# copy src files
WORKDIR /task
COPY ./src/task /task/src
COPY ./src/verefine/util /task/src/util
COPY ./config/config.json /task/src
ENV PYTHONPATH=/task:/task/src:$PYTHONPATH
RUN mv /task/src/renderer.py /task/3rdparty/verefine/src/verefine
RUN rm /task/src/util/renderer.py
RUN mv /task/src/renderer_task.py /task/src/util/renderer.py
ENV PYTHONPATH=/task/3rdparty/verefine:/task/3rdparty/verefine/src:$PYTHONPATH


# install python requirements
WORKDIR /task
RUN pip install --upgrade pip==20.3.4
RUN pip install --upgrade pip
RUN pip install wheel trimesh==3.16.4 # smh get errors otherwise
RUN pip install --default-timeout=100 -r /task/src/requirements.txt
RUN pip install --default-timeout=100 -r /task/src/requirements1.txt

RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get update
RUN apt-get install -y --no-install-recommends gcc-4.9
RUN apt-get install -y --no-install-recommends --only-upgrade libstdc++6
RUN pip install scikit-image==0.14.5 open3d==0.9.0.0

ENV __NV_PRIME_RENDER_OFFLOAD=1 
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

RUN pip install tinyobjloader==2.0.0rc7

RUN mkdir -p /verefine/data
COPY ./data /verefine/data

RUN mkdir -p /config
COPY ./config /config

# entry point
COPY ./src/task/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
