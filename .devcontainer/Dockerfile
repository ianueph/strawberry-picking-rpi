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
# x11-apps for X11 Debugging
# for urdf designing and visualization
# install udev
RUN mkdir -p /home/ws/.devcontainer/
COPY installations.sh /home/ws/.devcontainer/installations.sh
RUN chmod +x /home/ws/.devcontainer/installations.sh
RUN /home/ws/.devcontainer/installations.sh

RUN chmod 666 /dev/ttyACM0

USER $USERNAME
CMD ["/bin/bash"]