# PX4 Gazebo Plugin Suite for MAVLink SITL and HITL

[![Build Status](https://github.com/PX4/sitl_gazebo/workflows/Build%20Tests/badge.svg)](https://github.com/PX4/sitl_gazebo/actions?query=workflow%3A%22Build+Tests%22) [![MacOS Build Tests](https://github.com/PX4/sitl_gazebo/workflows/MacOS%20Build%20Tests/badge.svg)](https://github.com/PX4/sitl_gazebo/actions?query=workflow%3A%22MacOS+Build+Tests%22) 

This is a flight simulator for rovers, boats, multirotors, VTOL, fixed wing. It uses the motor model and other pieces from the RotorS simulator, but in contrast to RotorS has no dependency on ROS. Original project: https://github.com/ethz-asl/rotors_simulator.

**If you use this simulator in academic work, please cite RotorS as per the README in the above link.**


# Installation

This simulator is designed to be used with the PX4 Autopilot. Please follow the official developer toolchain installation instructions:
http://docs.px4.io/main/en/sim_gazebo_classic/

# Contributing and Testing

Please refer to the installations instructions above for normal usage and to get the development environment installed. This section covers specifics for developers interested to contribute to the simulator.

## *sitl_gazebo* plugin dependencies

Some plugins on this packages require some specific dependencies:

* Protobuf is required to generate custom protobuf messages to be published and subscribed between topics of different plugins;
* Jinja 2 is used to generate some SDF models from templates;
* Gstreamer is required for a plugin that streams video from a simulated camera.

## Build *sitl_gazebo*

Clone the repository to your computer.

**IMPORTANT: If you do not clone to ~/src/sitl_gazebo, all remaining paths in these instructions will need to be adjusted.**

```bash
mkdir -p ~/src
cd src
git clone --recursive https://github.com/PX4/sitl_gazebo.git
```

Create a build folder in the top level of your repository:

```bash
mkdir build
```

Navigate into the build directory and invoke CMake from it:

```bash
cd ~/src/sitl_gazebo
cd build
cmake ..
```

Now build the gazebo plugins by typing:

```bash
make -j$(nproc) -l$(nproc)
```

Next add the location of this build directory to your gazebo plugin path, e.g. add the following line to your `.bashrc` (Linux) or `.bash_profile` (Mac) file:

```bash
# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/src/sitl_gazebo/build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/src/sitl_gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
```

You also need to add the the root location of this repository, e.g. add the following line to your `.bashrc` (Linux) or `.bash_profile` (Mac) file:

```bash
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=$HOME/src/sitl_gazebo
```

## Install

If you wish the libraries and models to be usable anywhere on your system without
specifying th paths, install as shown below.

**Note: If you are using Ubuntu, it is best to see the packaging section.**

```bash
sudo make install
```


## Testing

Gazebo will now launch when typing 'gazebo' on the shell:

```bash
. /usr/share/gazebo/setup.sh
. /usr/share/mavlink_sitl_gazebo/setup.sh
gazebo worlds/iris.world
```

Please refer to the documentation of the particular flight stack how to run it against this framework, e.g. [PX4](http://dev.px4.io/simulation-gazebo.html)


### Unit Tests

For building and running test an installation of 'googletest' is needed.

On Ubuntu it can be installed with:

```bash
sudo apt-get install libgtest-dev
cd /usr/src/googletest
sudo cmake . && cd googletest
sudo make -j$(nproc) -l$(nproc)
sudo cp *.a /usr/lib
```

On macOS it needs to be installed from source:

```bash
git clone https://github.com/google/googletest
pushd googletest
mkdir build
pushd build
cmake ..
make -j$(nproc) -l$(nproc)
make install
```

When writing test it’s important to be careful which API functions of Gazebo are called. As no Gazebo server is running during the tests some functions can produce undefined behaviour (e.g. segfaults).

## CUDA Hardware Accelerated H264 encoding (optional)

1. Download CUDA 10.0 from https://developer.nvidia.com/cuda-toolkit-archive.
2. Download Video Codec SDK 9.0 from https://developer.nvidia.com/video-codec-sdk-archive.
3. Install both archives:

```bash
wget https://raw.githubusercontent.com/jackersson/env-setup/master/gst-nvidia-docker/install_video_codec_sdk.sh
chmod +x install_video_codec_sdk.sh
sudo ./install_video_codec_sdk.sh
sudo dpkg -i cuda-repo-ubuntu*.deb
sudo apt-key add /var/cuda-repo-<version>/7fa2af80.pub
sudo apt-get update
sudo apt-get install cuda
```

4. Reboot your system and run the command `nvidia-smi` to verify the successul installation of CUDA.
5. Install GStreamer 1.18.3:

```bash
git clone https://github.com/GStreamer/gst-build -b 1.18.3
cd gst-build
meson -Dbuildtype=release -Dstrip=true -Dgst-plugins-bad:introspection=enabled -Dgst-plugins-bad:nvcodec=enabled builddir
ninja -C builddir
sudo meson install -C builddir
```

6. Add `<useCuda>true</useCuda>` to any `gazebo_gst_camera_plugin` in a SDF file. For example `./models/fpv_cam/fpv_cam.sdf`.

#### *catkin tools*

With *catkin*, the unit tests are enabled by default.

```bash
# After setting up the catkin workspace
catkin build -j4 -l4 -DBUILD_ROS_PLUGINS=ON
cd build/mavlink_sitl_gazebo/
catkin run_tests
```

#### Plain CMake

For building the tests with plain CMake, the flag `ENABLE_UNIT_TESTS` needs to be provided.

```bash
mkdir build && cd build
cmake -DENABLE_UNIT_TESTS=On ..
```

Then build and run the tests:

```bash
make -j$(nproc) -l$(nproc)
make test
```


## Packaging

### Debian packages

To create a debian package for Ubuntu and install it to your system.

```bash
cd Build
cmake ..
make
rm *.deb
cpack -G DEB
sudo dpkg -i *.deb
```
