# Humanola Piper
## A. Prerequisites
Install the prerequisites: 
```
sudo apt install -y \
  ffmpeg \ 
  libsm6 \
  libxext6 \
  libstdc++6 \
  can-utils \
  ethtool \
  iproute2 \
  kmod
```
## B. Setting Up For CAN
First of all, on a freshly booted device, run the following: 
```
./scripts/can_config.sh
```
should the command above fail, try giving the file execute permission by running the following:
```
chmod +x ./scripts/can_config.sh
```
Once, CAN is configured, the OS now knows the location of the ports and its mapping, now let's activate those ports: 
```
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```
Now you are ready to use the repository for teleoperation. 

## B. Installing Dependencies
In general, we recommend using `conda` / `mamba` or any flavours that supports downloading from `conda-forge` to install dependencies. 
```
conda create --name piper
```
then activate: 
```
conda activate piper
```
Finally, install dependencies: 
```
pip install -r requirements.txt
conda install pinocchio -y
```

## C. Running the Script
Finally, run the humanola script after getting your `ROBO_ID` and `API_KEY`.
```
python3 main.py
```