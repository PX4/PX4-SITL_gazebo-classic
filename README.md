# sitl_gazebo

Install gazebo 4.1

Install protobuf compiler https://github.com/google/protobuf

Clone this repo, get materials, meshes, iris.sdf, model.config out of this repo and put in the .gazebo/models repo of your gazebo installation.

create a build directory then cmake ../ followed by make.

Now if it compiles correctly, launch gazebo with "gazebo", go to insert model and drag and drop iris. The simulator is ready, launch posix sitl.

It should work.
