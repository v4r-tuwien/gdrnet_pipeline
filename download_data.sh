wget -O gdrnpp_ycbv_weights.zip "https://owncloud.tuwien.ac.at/index.php/s/fkCygRgrV9C7zDH/download"
unzip gdrnpp_ycbv_weights.zip
mkdir -p ./src/gdrnpp/output/gdrn/ycbv
cp -r gdrnpp_ycbv_weights.pth ./src/gdrnpp/output/gdrn/ycbv/gdrnpp_ycbv_weights.pth
rm -r gdrnpp_ycbv_weights.zip gdrnpp_ycbv_weights.pth

wget -O gdrnpp_ycbv_models.zip "https://owncloud.tuwien.ac.at/index.php/s/gUxThY2caSsvix2/download"
unzip gdrnpp_ycbv_models.zip
mkdir -p ./src/gdrnpp/datasets/BOP_DATASETS/ycbv
cp -r models ./src/gdrnpp/datasets/BOP_DATASETS/ycbv
rm -r gdrnpp_ycbv_models.zip models

# # Ycb ichores weights and models
wget -O ycb_ichores.zip "https://owncloud.tuwien.ac.at/index.php/s/qTajYeAhCghzRl3/download"
unzip ycb_ichores.zip

unzip ycb_ichores/yolov8_weights/ycb_ichores.zip
cp -r train10/weights/last.pt ./src/yolov8/ycb_ichores.pt
rm -r train10 ycb_ichores/yolov8_weights

mkdir -p ./src/gdrnpp/output/gdrn/ycb_ichores
cp -r ycb_ichores/gdrn_weight/model_final.pth ./src/gdrnpp/output/gdrn/ycb_ichores/gdrnpp_ycb_ichores_weights.pth
rm -r ycb_ichores/gdrn_weight 

mkdir -p ./src/gdrnpp/datasets/BOP_DATASETS/ycb_ichores
cp -r ycb_ichores/models ./src/gdrnpp/datasets/BOP_DATASETS/ycb_ichores
rm -r ycb_ichores.zip ycb_ichores