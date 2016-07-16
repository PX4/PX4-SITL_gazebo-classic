# Gazebo for MAVLink SITL

This is a flight simulator for multirotors, VTOL and fixed wing. It uses the motor model and other pieces from the RotorS simulator, but in contrast to RotorS has no dependency on ROS. This repository is in the process of being re-integrated into RotorS, which then will support ROS and MAVLink as transport options: https://github.com/ethz-asl/rotors_simulator

**If you use this simulator in academic work, please cite RotorS as per the README in the above link.**

## Install Gazebo Simulator

Follow instructions on the [official site](http://gazebosim.org/tutorials?cat=install) to install Gazebo. Mac OS users should install Gazebo 7, Linux users PGazebo 6. Failing to install the right version can render the simulation inoperational.


## Protobuf

Install the protobuf library, which is used as interface to Gazebo.

### Ubuntu Linux

```bash
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libgazebo6-dev
```

### Mac OS

```bash
brew install graphviz sdformat3 protobuf eigen opencv
brew install gazebo7
```

## Build Gazebo Plugins (all operating systems)

Clone the gazebo plugins repository to your computer. IMPORTANT: If you do not clone to ~/src/sitl_gazebo, all remaining paths in these instructions will need to be adjusted.

```bash
mkdir -p ~/src
cd src
git clone https://github.com/Dronecode/sitl_gazebo.git
```

Create a build folder in the top level of your repository:

```bash
mkdir Build
```

Navigate into the build directory and invoke CMake from it:

```bash
cd ~/src/sitl_gazebo
cd Build
cmake ..
```

Now build the gazebo plugins by typing:

```bash
make
```

## Install

Finally install the plugin to your system:

**Note: If you are using ubuntu, it is best to see the packaging section.**

```bash
sudo make install
```

## Testing

Gazebo will now launch when typing 'gazebo' on the shell:

```bash
gazebo
```

Please refer to the documentation of the particular flight stack how to run it against this framework, e.g. [PX4](http://dev.px4.io/simulation-gazebo.html)

## Packaging

### Deb

To create a debian package for ubuntu and install it to your system.

```bash
cd Build
cmake ..
make
rm *.deb
cpack -G DEB
sudo dpkg -i *.deb
```
