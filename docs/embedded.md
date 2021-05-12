---
layout: default
title: "Embedded Software"
permalink: /embedded/
---

# Embedded Software

May 09, 2021 

## Choosing the platform to run ROS embedded

For this task, I need to take into consideration all the development of embedded micro-controllers and CPU's over the years since I started working with robotics. I started by looking around my place for boards that I had already purchased in the past. I found more than I remember acquiring:

- 2 Arduinos Uno
- 3 ESP8266 boards
- 1 Raspberry Pi 1 (original)
- 1 Raspberry Pi 1 Model A+
- 1 Raspberry Pi 3 Model B+
- 1 BeagleBone Black

Then I found out that I have never even opened the Pi 3, even if it was something that I purchased in 2018 along with the case, power adapter, and everything needed to get it started. I also did a quick benchmark on what people were using to run ROS and was impressed: almost everything could run ROS.

From AVR micro-controllers running `rosbridge` or `rosserial` to fully-fledged CPU with Intel i5 processors like the *Intel NUC D54250* that obviously could run the whole suite with `roscpp` or `rospy`. While researching, I understood that there were two types of embedded ROS: "bridged" and unified. The *bridged* architecture is mostly for boards with low processing power or without the proper tools to run a complete ROS framework. But the unified architecture seemed simpler if you got the power for it, it's possible to run your code wherever it's best. 

![ROS for embedded](/assets/img/ros_embedded.png)

With that in mind, it would be best to have more CPU power and avoid complexity. For the first version I will be using a **Raspberry Pi 3 B+** for two main reasons:

- Convenience: The B+ model has embedded wireless capabilities so it's possible to run it *headless,*
- Community: when compared to the BeableBone boards, the Raspberry family seems to be more popular, so it's easier to find support and instructions.

## Compiling ROS on RaspberryPi

The task at hand now was to build ROS on the to-be-embedded board. We need to optimize the operating system because the main code will be running on the ARM processor. It's also important to have seamless communication between the board and the master CPU. For that reason, I choose the latest Raspberry Pi OS Lite over the Ubuntu installation. 

The Lite version is a minimal operating system, so This would give room to inverse kinematics calculations and other processes instead of dealing with an unnecessary graphical interface. 

I tried to use the [official instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools) for build ROS2 on Ubuntu, but had to make some adjustments on the build configurations to avoid getting errors like this one after eleven hours of waiting:

```bash
Failed   <<< fastrtps [11h 8min 33s, exited with code 2]
Aborted  <<< test_osrf_testing_tools_cpp [10h 44min 41s]                                            
Aborted  <<< rttest [10h 45min 21s]                                                                 
Aborted  <<< rcutils [10h 57min 30s]
```

This happens mostly because there is not enough memory to handle every test while running all 4 threads at the same time. So, I pat myself on the back for using a popular board and searched for similar cases. In the end, I used two main sources:

- A [Medium article](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304) from *Sander van Dijk;*
- A [ROS forum post](https://answers.ros.org/question/373296/rosdep-cant-find-cyclonedds-during-foxy-install-on-raspbian/) from *bassline.*

### Step 1: Get everything needed

Download the *apt* repository, tools and ROS 2 code as per the official instructions.

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
sudo apt install --no-install-recommends -y \
  libcunit1-dev
mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos
```

Install all dependencies with `rosdep`.

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"
```

### Step 2: Configure the build

Set some files to ignore some visualization sub-trees as the article from *van Dijk.*

```bash
cd ~/ros2_foxy/
touch src/ros2/rviz/AMENT_IGNORE
touch src/ros-visualization/AMENT_IGNORE
touch src/ros2/system_tests/AMENT_IGNORE
```

Edit `/etc/sysctl.conf` as root to stop swapping by adding `vm.swappiness = 0` like the forum post from *bassline;*

```bash
sudo echo 'vm.swappiness = 0' >> /etc/sysctl.conf
```

Created a file `~/.colcon/defaults.yaml` to configure *colcon* build with `DBUILD_TESTING=OFF`.

```bash
build:
  cmake-args:
    - -DCMAKE_SHARED_LINKER_FLAGS='-latomic -lpython3.7m'
    - -DCMAKE_EXE_LINKER_FLAGS='-latomic -lpython3.7m'
    - -DCMAKE_BUILD_TYPE=RelWithDebInfo
    - -DBUILD_TESTING=OFF
```

### Step 3: Get something to do while compiling

This step is a simple command but a lot of waiting. I limited the compiler to one instance per thread and only used 2 threads, as recommended by *bassline.* For that, I use the command:

```bash
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --packages-skip-build-finished --continue-on-error --parallel-workers 2
```

And then I left it to work. Because I was running it headless and without a graphical interface, the command line via SSH was responsive and I could check on it once in a while. It took me 3 hours to compile everything.

### Step 4: Testing communication

In one terminal, I opened an SSH connection to the Raspberry Pi and instantiated a `talker` node from the demos:

```bash
export ROS_DOMAIN_ID=42
. ~/ros2_foxy/install/local_setup.bash
ros2 run demo_nodes_py talker
```

In another terminal, this time without the SSH connection, source the setup file and then run a `listener`:

```bash
export ROS_DOMAIN_ID=42
. ~/ros2_foxy/install/local_setup.bash
ros2 run demo_nodes_py listener
```

* * *

[⌫ Back](./../)