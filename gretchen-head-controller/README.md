
# check tf between links
- rosrun rqt_tf_tree rqt_tf_tree
- rosrun tf tf_echo /base_link /camera_color_optical_frame

# control manual
- get joint infos
	- rostopic echo /gretchen/joint/poses
- set position
	- when you run head_controller, you can use this function.
rostopic pub /gretchen/joint/cmd std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0,0]" 
