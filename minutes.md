# Minutes
A log of all of our meetings

## 3 November 2017
### 1000-1500
* Set up git repository
* Figured out how to track AR tags using Baxter's head camera
  * Edit the ```pr2_indiv_no_kinect.launch``` file
  * Set ```cam_image_topic``` to ```/cameras/head_camera/image```
  * Set ```cam_info_topic``` to ```/cameras/head_camera/camera_info```
  * Set ```output_frame``` to ```/head_camera```
* Did a ```rostopic echo``` on the ```/ar_pose_marker``` to see the positions
  * When you hold a tag in front of the camera, you see the Cartesian coordinates publishe to this topic

## 6 November 2017
### 1000-1600
* Followed Lab 4 to git clone the ar_track_alvar directory into our workspace and and the appropriate changes that we discovered on Nov.3 2017 
* Also decided to do some research on how to move the arms of Baxter and discovered that 
* It would be best to first plan with MoveIt and see if we can gain any helpful resources. 
* We came into the lab and started to test out the ar_track_alvar files and check to see if everything we did was working correctly. The when we echoed the ar_track pose, we seemed to be getting the position of the AR tag with repect to head_camera
* Although the MoveIt interface provided a useful insight to what we need to do, we realized that the best approacht that we could do is refer back to Lab 5 and use Inverse Kinematics. We borrowed code form the lab and want use the infromation that we gather from the AR tags in order to make the Baxter move into a specified location. That location is believed to be the location of the AR tag in space. However, over inverse kinematics is still a work in progress. 

### 1930-2115
* Figured out how to debug tf
* Running ```rosrun tf tf_echo base reference/head_camera``` works
  * Emphasis on the ```reference/head_camera``` instead of just ```head_camera```
* Took a dinner break at Snack Shack

### 2200-0130
* Rebooted Baxter because we thought there was something off with the configuration
  * Rebooting seemed to help with the RViz; can display RobotModel w/out errors
* NOTE: Baxter can only run two cameras at once due to bandwidth limitations
  * Ran into this issue after rebooting Baxter
  * Run ```rosrun baxter_tools camera_control.py -l``` to list cameras
  * Run ```rosrun baxter_tools camera_control.py -o <camera name>``` to open ```<camera name>```
  * Run ```rosrun baxter_tools camera_control.py -c <camera name>``` to close ```<camera name>```
* Decided to switch to Sawyer robot due to wonkiness of Baxter's head camera
  * NOTE: Sawyer can only run one camera at a time due to bandwidth limitations
  * Still unsure how to use head_camera; ran ```rosrun intera_examples camera_display.py -c head_camera``` to force the head_camera to work
* Tested ar_track_alvar on the Sawyer
  * Dim lights introduced noise in the tracking, but the translation was otherwise accurate
  * Very accurate detection; we'll probably switch to the Sawyer
* Started testing the inverse kinematics
  * Not going well so far; keeps aborting

## 7 November 2017
### 2100-2300
* NOTE Sawyer cameras are on topics ```/io/internal_camera/*```
* Because this is getting annoying, cheat sheet for setting up inverse kinematics
  * Make sure robot is enabled
  * Run ```rosrun intera_interface joint_trajectory_action_server.py```
  * Run ```roslaunch sawyer_moveit_config move_group.launch electric_gripper:=true```
  * Use ```rosrun tf tf_echo head_camera right_wrist``` or something to check positions
* Discovered consistent f-up in inverse kinematics
  * Always get error ```/sdk_position_w_id_joint_trajectory_action_server_right: Exceeded Error Threshold on right_j5: <some number>```
* Yuge bug, found that our planning frame was actually ```/base``` and not desired ```/head_camera```
  * Put ```print(group.get_planning_frame())``` in ik node
