# Minutes
A log of all of our meetings

## 3 November 2017
### 1000-1500
* Set up git repository
* Figured out how to track AR tags using Baxter's head camera
  * Edit the ```pr2_indiv_no_kniect.launch``` file
  * Set ```cam_image_topic``` to ```/cameras/head_camera/image```
  * Set ```cam_info_topic``` to ```/cameras/head_camera/camera_info```
  * Set ```output_frame``` to ```/head_camera```
* Did a ```rostopic echo``` on the ```/ar_pose_marker``` to see the positions
  * When you hold a tag in front of the camera, you see the Cartesian coordinates publishe to this topic

## 6 November 2017
### 1000-1600
