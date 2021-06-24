# testing_tejal
temporary repo

## Steps to Run Locally
1. Follow all the [installation](installation/) steps
1. `cd ~/catkin_ws/src/`
1. `git clone https://github.com/tejalbarnwal/testing_tejal.git (https)`  
 
1. `catkin_make` OR `catkin build`
1. in a new terminal do `roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_rplidar'
1. in a seperate terminal do `rosrun testing_tejal relative_local_setpoint.py`
1. in a seperate terminal do `rosrun testing_tejal raster.py`
