# Gazebo for MAVLink SITL

This is a flight simulator for multirotors, VTOL and fixed wing.

NOTE: This repository is in the process of being re-integrated into RotorS: https://github.com/ethz-asl/rotors_simulator

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

Next add the location of this build directory to your gazebo plugin path, e.g. add the following line to your .bashrc (Linux) or .bash_profile (Mac) file:


```bash
# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/src/sitl_gazebo/Build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/src/sitl_gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
```

You also need to add the the root location of this repository, e.g. add the following line to your .bashrc (Linux) or .bash_profile (Mac) file:
```bash
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=$HOME/src/sitl_gazebo
```

Navigate into the build directory and invoke CMake from it:

```bash
cd ~/src/sitl_gazebo
cd Build
cmake ..
```

Autogenerate the sdf file with the command
```bash
make sdf
```

Now build the gazebo plugins by typing:

```bash
make
```

Gazebo will now launch when typing 'gazebo' on the shell:

```bash
gazebo
```

Please refer to the documentation of the particular flight stack how to run it against this framework.
