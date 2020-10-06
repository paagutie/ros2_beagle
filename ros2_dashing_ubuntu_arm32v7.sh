
echo "This script install ROS 2 Dashing Diademata on the Beaglebone Blue Board"

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8


echo "Install some development tools and ROS tools ..."

sudo apt update && apt install -y \
    pkg-config \
    lsb-release \
    curl \
    gfortran \
    libpcre3 \
    libpcre3-dev \
    bash-completion \
    dirmngr \
    gnupg2

# ROS2 dependencies
sudo apt install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget


curl http://repo.ros2.org/repos.key | sudo apt-key add -
sh -c 'echo "deb [arch=armhf] http://repo.ros2.org/ubuntu/main \
    `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep

python3 -m pip install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

python3 -m pip install -U \
    pytest \
    pytest-cov \
    pytest-runner \
    setuptools

# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev
    
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev

echo "Get ROS 2 code ..."
mkdir -p ~/ros2_dashing/src
cd ~/ros2_dashing
wget https://raw.githubusercontent.com/ros2/ros2/dashing/ros2.repos
vcs import src < ros2.repos

echo "Done."
echo "Installing dependencies ..."

sudo rosdep init
rosdep update
rosdep install --from-paths src \
    --ignore-src \
    --rosdistro dashing -y \
    --skip-keys "console_bridge \
        fastcdr \
        fastrtps \
        libopensplice67 \
        libopensplice69 \
        rti-connext-dds-5.3.1 \
        urdfdom_headers"

touch \
    src/ros2/rviz/COLCON_IGNORE \
    src/ros2/urdf/COLCON_IGNORE \
    src/ros2/kdl_parser/COLCON_IGNORE \
    src/ros2/urdfdom/COLCON_IGNORE \
    src/ros2/robot_state_publisher/COLCON_IGNORE \
    src/ros2/ros1_bridge/COLCON_IGNORE \
    src/ros2/demos/intra_process_demo/COLCON_IGNORE \
    src/ros2/demos/pendulum_control/COLCON_IGNORE \
    src/ros2/demos/pendulum_msgs/COLCON_IGNORE \
    src/ros2/rmw_connext/COLCON_IGNORE \
    src/ros2/rmw_opensplice/COLCON_IGNORE \
    src/ros2/rosidl_typesupport_connext/COLCON_IGNORE \
    src/ros2/rosidl_typesupport_opensplice/COLCON_IGNORE 


echo "Done."
echo "Warning, the process could take hours!"
echo "Building the code in the workspace ..."
colcon build --symlink-install


echo "Done."
echo "The build has been completed :)"


