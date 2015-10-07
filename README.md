# sitl_gazebo

Install gazebo.

Install protobuf compiler https://github.com/google/protobuf  (make sure its the same version that is used by your gazebo version. For mac, i think appropriate protobuf plugin comes packed with gazebo)

Clone this repo, take the folders - materials, meshes and files - iris.sdf, model.config out of this repo and put in the .gazebo/models directory of your gazebo installation.

Create a build directory then cmake ../ followed by make. Then, add current build directory to your gazebo plugin path.

Running simulation with pixhawk posix sitl.
	- Start sitl with make sitl_quad_gazebo
	- Start gazebo with gazebo
		- Insert iris model from insert tab. Thats it, if it installed correctly, they should begin communicating over raw UDP in mavlink. GCS is the same as with jmavsim sitl. 

(Try attitude_estimator_ekf instead of attitude_estimator_q if iris seems to roll too much at takeoff)
