ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG video $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.

# install libcamera and libcamera-apps
RUN apt install -y git python3-jinja2
RUN apt install -y libboost-dev
RUN apt install -y libgnutls28-dev openssl libtiff-dev pybind11-dev
RUN apt install -y qtbase5-dev libqt5core5a
RUN apt install -y meson cmake
RUN apt install -y python3-yaml python3-ply
RUN apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
WORKDIR /tmp
RUN git clone https://github.com/raspberrypi/libcamera.git
WORKDIR /tmp/libcamera
RUN meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
RUN ninja -C build install

RUN apt install -y libegl1-mesa-dev
WORKDIR /tmp
RUN git clone https://github.com/anholt/libepoxy.git
WORKDIR /tmp/libepoxy
RUN mkdir _build
WORKDIR /tmp/libepoxy/_build
RUN meson
RUN ninja
RUN ninja install
RUN apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
WORKDIR /tmp
RUN git clone https://github.com/raspberrypi/libcamera-apps.git
WORKDIR /tmp/libcamera-apps
RUN mkdir build
# x11-apps for X11 Debugging
RUN apt install x11-apps -y
# for urdf designing and visualization
RUN apt install ros-jazzy-rviz2 -y
RUN apt install ros-jazzy-xacro -y
# install udev
RUN apt install udev -y

USER $USERNAME
CMD ["/bin/bash"]