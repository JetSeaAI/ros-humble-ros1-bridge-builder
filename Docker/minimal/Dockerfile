# Use the official ROS Humble base image
FROM ros:humble-ros-base-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* 

EXPOSE 221

ENV ROS_DISTRO=humble \
    GZ_VERSION=harmonic \
    SHELL=/bin/bash \
    NB_USER=arg \
    NB_UID=1000

###################################### user #####################################
RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${NB_UID} \
    ${NB_USER} 

RUN echo "root:root" | chpasswd \
    && echo "${NB_USER}:111111" | chpasswd

ENV HOME=/home/${NB_USER}


# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-dev-tools \
    libboost-all-dev \
    ros-humble-rviz2 \
    ros-humble-image-transport\
    ros-humble-rmw-cyclonedds-cpp \ 
    openssh-server \
    libopencv-dev\
    pyqt5-dev \
    python3-pandas \
    ros-humble-geodesy\
    ros-humble-pcl-conversions 


# Install Python packages
# RUN pip3 install pyserial future bluerobotics-ping cython pyqt5

# Install necessary packages for Neovim and LazyVim
RUN apt-get install ninja-build gettext cmake unzip curl build-essential python3.10-venv ripgrep fd-find xsel -y
RUN wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash \
    && . ~/.nvm/nvm.sh \
    && nvm install node

#Install Neovim
RUN git clone https://github.com/neovim/neovim \
    && cd neovim \
    && git checkout stable\
    && make CMAKE_BUILD_TYPE=Release \
    && make install

# Install LazyVim plugins
RUN git clone https://github.com/JetSeaAI/LazyVim-starter.git ~/.config/nvim \
    && rm -rf ~/.config/nvim/.git 

# Bootstrap lazy.nvim
RUN nvim --headless "+Lazy! sync" +qa
# Second bootup to make sure everything's ok 
RUN nvim --headless "+Lazy! sync" +qa


#clean up
RUN rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /home/${NB_USER}
# COPY js-perceptions /home/${NB_USER}/js-perceptions

#Add vpn configuration
# ADD js-/cyclonedds.xml /home/${NB_USER}/cyclonedds.xml
    
RUN echo "root ALL=(ALL)  ALL" >> /etc/sudoers \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc

# Enable colcon command-line completion
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /${HOME}/.bashrc
# Source the workspace setup.bash
RUN echo "source /home/arg/ros-humble-ros1-bridge-builder/ros-humble-ros1-bridge/install/local_setup.bash" >> /${HOME}/.bashrc
#Using CycloneDDS to avoid coudn't receive message from the topic
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /${HOME}/.bashrc

# RUN echo "export CYCLONEDDS_URI=file:///$(pwd)/cyclonedds.xml " >> /${HOME}/.bashrc

# Print information on terminal start
RUN echo 'echo "Welcome to the ROS $ROS_DISTRO Docker container!"' >> /${HOME}/.bashrc 
    # && echo 'echo "Available launch files:"' >> /${HOME}/.bashrc \
    # && echo 'find /home/arg/js-perceptions/launch -name "*.launch.py"' >> /${HOME}/.bashrc \
    # && echo 'echo "To run the Simrad radar: ros2 launch halo_radar_visualize halo_radar_bringup.launch.py "' >> /${HOME}/.bashrc \
    # && echo 'echo "To run the Imagenex881a sonar: ros2 launch imagenex_881a imagenex_881a_py.launch.py "' >> /${HOME}/.bashrc \
    # && echo 'echo "To run the Ping360 sonar: ros2 launch ping360_sonar ping360_bringup_py.launch.py  "' >> /${HOME}/.bashrc 

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]