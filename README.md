# ROS_Unity

## Repository with robot simulation with Unity ml-agent and manipulation with ROS

### ml-agent is a Unity package which enable the training of agents on Unity physics engine. Linked repo : 

![](braccio_reacher.gif)

#### Braccio robot (5 ddl) trained with PPO on a reaching task using Unity Transforms

![](reacher_rigidBodies.gif)

#### Arm robot (3 ddl) trained with PPO on a reaching task using Unity RigidBodies.

### Ros# is a Unity/Ros package for communication between Unity simulation scene and ROS/Gazebo. It also enable conversion of urdf files to Unity gameObject.

![](braccio_unity.gif)

#### Braccio robot following the movements of virtual Unity Braccio Robot. Base rotation need to be fixed.

![alt text](https://github.com/sabeaussan/ROS_Unity/blob/master/WebCam.png?raw=true)
#### Pose estimation of an Aruco marker

#### TODO :

- Fix braccio rotation
- Add sim2real transfer 
  linked articles : https://arxiv.org/pdf/2002.11635.pdf
		    https://arxiv.org/pdf/1709.07857.pdf
 		    https://arxiv.org/pdf/1910.07113.pdf


