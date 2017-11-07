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

### 1930-
* Figured out how to debug tf
* Running ```rosrun tf tf_echo base reference/head_camera``` works
  * Emphasis on the ```reference/head_camera``` instead of just ```head_camera```
