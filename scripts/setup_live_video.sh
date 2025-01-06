sudo apt update
sudo apt install -y build-essential cmake git libjpeg-dev
git clone https://github.com/jacksonliam/mjpg-streamer.git
cd mjpg-streamer/mjpg-streamer-experimental
make
sudo make install
cd ../..
rm -rf mjpg-streamer