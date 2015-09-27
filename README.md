# Gazebo for PX4 SITL
## Instructions for Users (Ubuntu 14.04)
### Install Gazebo Simulator
Follow instructions on the [official site] (http://gazebosim.org/tutorials?cat=install)

### Install Protobuf Compiler
 Clone repository from [here](https://github.com/google/protobuf):
 ```
 git clone https://github.com/google/protobuf.git
 ```
Gazebo requires version 2.5.0 of protobuf, therefore you will have to checkout the correct version.
In the top level of the protobuf repository type:
```
git checkout v2.5.0
```
After that follow the protobuf build instructions located in the README of the repository.

#### Build Gazebo Pluggins
Clone the gazebo pluggins repository to your computer:
```
git clone <url_to_repository>
```

From the cloned repository copy the folders **materials**, **meshes** and the files **iris.sdf***, **model.config** into the **.gazebo/models** directory of your gazebo installation. (Normally this folder is located in your home directory).

Create a build folder in the top level of your repository:
```
mkdir Build
```
Next add the location of this build directory to your gazebo plugin path, e.g. add the following line to your .bashrc file:
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~<path_to_your_repository>/Build
```

Navigate into the build directory and invoke CMake from it:
```
cd Build
cmake ../
```

Now build the gazebo pluggins by typing:
```
make
```

If everything has gone well you can launch the PX4 SITL Simulation in a terminal. For instructions see section **Compile and Run the Simulated Autopilot** of the PX4 SITL instructions page [here.](https://pixhawk.org/dev/simulation/native_sitl)

Once you have started the simulation you can launch gazebo in a terminal, type:
```
gazebo
```

Insert the IRIS model from the **insert** tab. This should trigger the communication with the PX4 SITL app.
