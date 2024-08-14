mkdir -p ./data
mkdir -p ./data/datasets
mkdir -p ./data/datasets/ycbv
mkdir -p ./data/datasets/ycb_ichores
mkdir -p ./data/weights
mkdir -p ./data/weights/yolov8
mkdir -p ./data/weights/gdrnpp

# YCBV Dataset
wget -O gdrnpp_ycbv_weights.zip "https://owncloud.tuwien.ac.at/index.php/s/fkCygRgrV9C7zDH/download"
unzip gdrnpp_ycbv_weights.zip
cp -r gdrnpp_ycbv_weights.pth ./data/weights/gdrnpp/gdrnpp_ycbv_weights.pth
rm -r gdrnpp_ycbv_weights.zip gdrnpp_ycbv_weights.pth

wget -O gdrnpp_ycbv_models.zip "https://owncloud.tuwien.ac.at/index.php/s/gUxThY2caSsvix2/download"
unzip gdrnpp_ycbv_models.zip
cp -r models ./data/datasets/ycbv/models
rm -r gdrnpp_ycbv_models.zip models

wget -O yolov8_ycbv.zip "https://owncloud.tuwien.ac.at/index.php/s/eh3qmWzuAsJiqr1/download"
unzip yolov8_ycbv.zip
cp -r train/weights/last.pt ./data/weights/yolov8/ycbv.pt
rm -r train yolov8_ycbv.zip

# YCB-iChores Dataset
wget -O ycb_ichores.zip "https://owncloud.tuwien.ac.at/index.php/s/qTajYeAhCghzRl3/download"
unzip ycb_ichores.zip

unzip ycb_ichores/yolov8_weights/ycb_ichores.zip
cp -r train10/weights/last.pt ./data/weights/yolov8/ycb_ichores.pt
rm -r train10 ycb_ichores/yolov8_weights

cp -r ycb_ichores/gdrn_weight/model_final.pth ./data/weights/gdrnpp/gdrnpp_ycb_ichores_weights.pth
rm -r ycb_ichores/gdrn_weight 

cp -r ycb_ichores/models ./data/datasets/ycb_ichores/models
rm -r ycb_ichores.zip ycb_ichores