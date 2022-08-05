# AutonomiSim
This repository is the home to the source code for the USN SeaDrone simulation implementation, based off the VRX/WAM-V simulation repository (https://github.com/osrf/vrx)

- Provides a platform for the development and testing of the usn SeaDrone. 
- Fork of the original osrf/vrx gazebo
- Made to aid development of the real SeaDrone Robot
- Hydrodynamics, localization, navigation  
<br />

## Startup

To start the simulation, run the ```usn_drone_gazebo/launch/usn_drone.launch``` launch file: \
```roslaunch usn_drone_gazebo main_setup.launch``` \
This launch file initializes the example_world gazebo map, and spawns the robot. \
The file also launches the localization node, and the cmd_vel, which simplifies the development of autonomous navigation. 
</br></br>

### Navigation

The repository implements a PD+LOS algorithm for navigation, as a ROS action server. \
```rosrun usn_navigation server_pd_los.py``` \
The action server takes a 2DPoseArray, consisting of n poses. \
When the server receives a goal, it iterates through the list of positions (x, y) and publishes on the 'cmd_vel' topic, navigating towards the next position.

```rosrun usn_navigation visualize_path_taken.py``` will subscribe to the localization of the robot and publish a Path message which may be viewed in RviZ.

<img src="https://cdn.discordapp.com/attachments/941352029907996732/998490481732354059/unknown.png" alt="LOS guidance test" title="Navigation test" width="600"/>




# USN team:
- Edvart Grüner Bjerke

</br></br></br></br>






# VRX team:
## Reference

```
@InProceedings{bingham19toward,
  Title                    = {Toward Maritime Robotic Simulation in Gazebo},
  Author                   = {Brian Bingham and Carlos Aguero and Michael McCarrin and Joseph Klamo and Joshua Malia and Kevin Allen and Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle                = {Proceedings of MTS/IEEE OCEANS Conference},
  Year                     = {2019},
  Address                  = {Seattle, WA},
  Month                    = {October}
}
```
## Contacts

 * Carlos Aguero <caguero@openrobotics.org>
 * Michael McCarrin <mrmccarr@nps.edu>
 * Brian Bingham <bbingham@nps.edu>
