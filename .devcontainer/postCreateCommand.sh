sudo rosdep update 
sudo rosdep install --from-paths src --ignore-src -r -y 
sudo chown -R $(whoami) /home/ws/ 
sudo chmod 666 /dev/ttyACM0
cd ..
git submodule update --init --recursive
sudo ln -sf /tmp/libcamera/build/src/libcamera/libcamera.so* /opt/ros/jazzy/lib/