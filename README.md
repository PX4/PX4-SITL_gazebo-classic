# Gazebo for PX4 SITL



## Prerequisites for Ubuntu 14.04

### Install Gazebo Simulator
Follow instructions on the [official site] (http://gazebosim.org/tutorials?cat=install)

### Install Protobuf Compiler
 Clone repository from [here](https://github.com/google/protobuf):
```bash
git clone https://github.com/google/protobuf.git
```

Gazebo requires version 2.5.0 of protobuf, therefore you will have to checkout the correct version.
In the top level of the protobuf repository type:

```bash
git checkout v2.5.0
```
After that follow the protobuf build instructions located in the README of the repository.

## Prerequisites for Mac OS

```bash
brew install gazebo
```

## Build Gazebo Pluggins
Clone the gazebo pluggins repository to your computer:
```bash
git clone <url_to_repository>
```

From the cloned repository copy the folders **materials**, **meshes** and the files **iris.sdf***, **model.config** into the **.gazebo/models** directory of your gazebo installation. (Normally this folder is located in your home directory).

Create a build folder in the top level of your repository:
```bash
mkdir Build
```
Next add the location of this build directory to your gazebo plugin path, e.g. add the following line to your .bashrc or .bash_profile (Mac) file:

```bash
# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/src/sitl_gazebo/Build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/src/sitl_gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
```

Navigate into the build directory and invoke CMake from it:
```bash
cd Build
cmake ../
```

Now build the gazebo pluggins by typing:
```bash
make
```

If everything has gone well you can launch the PX4 SITL Simulation in a terminal. For instructions see section **Compile and Run the Simulated Autopilot** of the PX4 SITL instructions page [here.](https://pixhawk.org/dev/simulation/native_sitl)

Once you have started the simulation you can launch gazebo in a terminal, type:
```bash
gazebo
```

Insert the IRIS model from the **insert** tab. This should trigger the communication with the PX4 SITL app.
