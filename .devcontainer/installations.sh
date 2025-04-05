# install libcamera and libcamera-apps
sudo apt install -y git python3-jinja2
sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff-dev pybind11-dev
sudo apt install -y qtbase5-dev libqt5core5a
sudo apt install -y meson cmake
sudo apt install -y python3-yaml python3-ply
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
cd /tmp
sudo git clone https://github.com/raspberrypi/libcamera.git
cd /tmp/libcamera
sudo meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
sudo ninja -C build install

sudo apt install -y libegl1-mesa-dev
cd /tmp
sudo git clone https://github.com/anholt/libepoxy.git
cd /tmp/libepoxy
sudo mkdir _build
cd /tmp/libepoxy/_build
sudo meson
sudo ninja
sudo ninja install
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
cd /tmp
sudo git clone https://github.com/raspberrypi/libcamera-apps.git
cd /tmp/libcamera-apps
sudo mkdir build
# x11-apps for X11 Debugging
sudo apt install x11-apps -y
# for urdf designing and visualization
sudo apt install ros-jazzy-rviz2 -y
sudo apt install ros-jazzy-xacro -y
# install udev
sudo apt install udev -y

pip3 install torch torchvision torchaudio --break-system-packages
pip3 install transformers --break-system-packages