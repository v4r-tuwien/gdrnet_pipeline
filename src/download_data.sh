wget -O data.zip "https://owncloud.tuwien.ac.at/index.php/s/6GZSFUHRSMfUxxh/download"
unzip data.zip
cd verefine_data
unzip data.zip
mv data ../
cd ../
rm -rf verefine_data
rm -rf data.zip
