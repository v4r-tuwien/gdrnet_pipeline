wget -O yolov5_ycbv_weights.zip "https://owncloud.tuwien.ac.at/index.php/s/lbnwdUAR3uSpKOj/download"
unzip yolov5_ycbv_weights.zip
cp -r yolov5_ycbv_weights.pt ./src/yolov5/yolov5_ycbv_weights.pt
rm -r yolov5_ycbv_weights.zip yolov5_ycbv_weights.pt

wget -O gdrnpp_ycbv_weights.zip "https://owncloud.tuwien.ac.at/index.php/s/fkCygRgrV9C7zDH/download"
unzip gdrnpp_ycbv_weights.zip
mkdir -p ./src/gdrnpp/outputs
cp -r gdrnpp_ycbv_weights.pth ./src/gdrnpp/outputs/gdrnpp_ycbv_weights.pth
rm -r gdrnpp_ycbv_weights.zip gdrnpp_ycbv_weights.pth

wget -O gdrnpp_ycbv_models.zip "https://owncloud.tuwien.ac.at/index.php/s/gUxThY2caSsvix2/download"
unzip gdrnpp_ycbv_models.zip
mkdir -p ./src/gdrnpp/datasets/BOP_DATASETS/ycbv
cp -r models ./src/gdrnpp/datasets/BOP_DATASETS/ycbv
rm -r gdrnpp_ycbv_models.zip models