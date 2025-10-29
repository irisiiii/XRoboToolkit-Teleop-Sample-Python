  370  source install/setup.bash
  371  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=right_camera     camera_namespace:=camera2     enable_color:=true     enable_depth:=true     pointcloud.enable:=true
  372  ros2 node list
  373  chmod +x start_cameras.sh
  374  source install/setup.bash 
  375  chmod +x start_cameras.sh
  376  root@zy2-white:~/vr# chmod +x start_cameras.sh
  377  chmod: cannot access 'start_cameras.sh': No such file or directory
  378  source install/setup.bash 
  379  adb connect 192.168.1.133:5555
  380  source install/setup.bash 
  381  python3 questVR_ros2/questVR_ros2/test_jaka_vr_meshcat4_test.py
  382  /bin/python3 /root/vr/questVR_ros2/questVR_ros2/test_jaka_vr_meshcat4.py
  383  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  384  source install/setup.bash
  385  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=true     pointcloud.enable:=true
  386  source install/setup.bash 
  387  ros2 launch frame_sync  start_full_system.launch.py
  388  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  389  sudo ip link set can1 up
  390  ros2 launch frame_sync  start_full_system.launch.py
  391  colcon build --packages-select frame_sync
  392  source install/setup.bash 
  393  ros2 launch frame_sync  start_full_system.launch.py
  394  source install/setup.bash 
  395  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  396  source install/setup.bash
  397  ros2 launch jaka_robot_driver jaka_driver.launch.py
  398  source install/setup.bash 
  399  ros2 topic list
  400  ros2 topic echo /left_arm/jiazhua_state
  401  ros2 topic list
  402  source install/setup.bash 
  403  adb connect 192.168.1.133:5555
  404  source install/setup.bash 
  405  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  406  source install/setup.bash 
  407  adb connect 192.168.1.133:5555
  408  pip install pykalman
  409  pip show pykalman
  410  source install/setup.bash
  411  ros2 launch jaka_robot_driver jaka_driver.launch.py
  412  which python
  413  ros2 launch frame_sync start_full_system.launch.py
  414  source install/setup.bash 
  415  ros2 launch frame_sync start_full_system.launch.py
  416  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  417  source install/setup.bash 
  418  adb connect 192.168.1.133:5555
  419  source install/setup.bash 
  420  ros2 launch frame_sync start_full_system.launch.py
  421  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  422  sudo ip link set can1 up
  423  ros2 launch frame_sync start_full_system.launch.py
  424  sudo ip link set can1 up
  425  ros2 launch frame_sync start_full_system.launch.py
  426  source install/setup.bash
  427  ros2 launch jaka_robot_driver jaka_driver.launch.py
  428  colcon build --packages-select jaka_robot_driver symlink-install
  429  source install/setup.bash
  430  ros2 launch jaka_robot_driver jaka_driver.launch.py
  431  source install/setup.bash
  432  ros2 launch jaka_robot_driver jaka_driver.launch.py
  433  source install/setup.bash
  434  ros2 launch jaka_robot_driver jaka_driver.launch.py
  435  pkill -f ros2
  436  rm -rf /dev/shm/fastrtps_*
  437  rm -rf /dev/shm/fast_datasharing_*
  438  rm -rf /tmp/fastrtps_*
  439  pkill -f ros2-daemon
  440  ros2 daemon stop
  441  ros2 daemon start
  442  source install/setup.bash
  443  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  444  source install/setup.bash
  445  ros2 launch frame_sync start_full_system.launch.py
  446  pkill ros2
  447  source install/setup.bash 
  448  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  449  source install/setup.bash
  450  ros2 launch frame_sync start_full_system.launch.py
  451  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  452  sudo ip link set can1 up
  453  source install/setup.bash
  454  ros2 launch frame_sync start_full_system.launch.py
  455  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  456  source install/setup.bash
  457  ros2 launch jaka_robot_driver jaka_driver.launch.py
  458  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  459  source install/setup.bash
  460  /bin/python3 questVR_ros2/questVR_ros2/test_jaka_vr_meshcat4.py
  461  adb connect 192.168.1.133:5555
  462  /bin/python3 questVR_ros2/questVR_ros2/test_jaka_vr_meshcat4.py
  463  chmod +x keyboard_gripper_control.py
  464  source /opt/ros/humble/setup.bash
  465  source /root/vr/install/setup.bash
  466  source install/setup.bash
  467  ros2 run jiazhua_driver jiazhua_node
  468  python3 keyboard_gripper_control.py
  469  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  470  sudo ip link set can1 up
  471  source install/setup.bash
  472  ros2 launch frame_sync start_full_system.launch.py
  473  source install/setup.bash 
  474  source install/setup.bash
  475  ros2 run jiazhua_driver jiazhua_node
  476  source install/setup.bash
  477  python3 keyboard_gripper_control.py
  478  source install/setup.bash
  479  ros2 launch frame_sync start_full_system.launch.py
  480  source install/setup.bash
  481  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  482  source install/setup.bash
  483  ros2 launch frame_sync start_full_system.launch.py
  484  colcon build --packages-select frame_sync
  485  source install/setup.bash 
  486  source install/setup.bash
  487  python3 keyboard_gripper_control.py
  488  sudo fdisk -l
  489  sudo apt update
  490  sudo apt install fdisk
  491  sudo fdisk -l
  492  df -h
  493  source install/setup.bash
  494  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  495  source install/setup.bash
  496  ros2 launch jaka_robot_driver jaka_driver.launch.py
  497  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  498  source install/setup.bash
  499  ros2 launch frame_sync start_full_system.launch.py
  500  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  501  ros2 launch orbbec_camera gemini_330_series.launch.py     enable_color:=true     enable_depth:=false     enable_pointcloud:=false     color_fps:=30
  502  colcon build --packages-select  frame_sync_msgs
  503  colcon build --packages-select frame_sync
  504  colcon build
  505  source install/setup.bash
  506  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=true     pointcloud.enable:=true
  507  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  508  echo $ROS_DISTRO
  509  ros2 --version
  510  source install/setup.bash
  511  ros2 launch orbbec_camera gemini_330_series.launch.py     enable_color:=true     enable_depth:=false     enable_pointcloud:=false     color_fps:=30
  512  source install/setup.bash
  513  ros2 launch jaka_robot_driver jaka_driver.launch.py
  514  source install/setup.bash
  515  ros2 launch orbbec_camera gemini_330_series.launch.py     enable_color:=true     enable_depth:=false     enable_pointcloud:=false     color_fps:=30
  516  source install/setup.bash
  517  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  518  source install/setup.bash
  519  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=right_camera     camera_namespace:=camera2     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  520  ros2 daemon stop
  521  source install/setup.bash
  522  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  523  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  524  source install/setup.bash
  525  ros2 run robot_state_converter converter
  526  ros2 topic list
  527  ros2 topic echo /right_arm/joint_states
  528  ros2 topic list
  529  ros2 topic echo /left_arm/joint_states
  530  cd /root/jwq/vr && source install/setup.bash && ros2 run robot_state_converter converter
  531  cd /root/jwq/vr && source install/setup.bash && ros2 run robot_state_converter converter
  532  ros2 topic list
  533  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  534  source install/setup.bash
  535  python3 src/jaka_robot_driver/scripts/play_dance.py 3 --loop
  536  colcon build --packages-select jaka_robot_driver symlink-install
  537  source install/setup.bash
  538  ros2 launch jaka_robot_driver jaka_driver.launch.py
  539  source install/setup.bash
  540  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  541  colcon build --packages-select jaka_robot_driver symlink-install
  542  source install/setup.bash
  543  ros2 launch jaka_robot_driver jaka_driver.launch.py
  544  adb connect 192.168.1.133:5555
  545  cd ..
  546  git clone https://github.com/jmcoholich/openteach.git
  547  cd openteach/
  548  ros2 topic list
  549  ros2 topic echo /camera1/left_camera/color/image_rect_raw
  550  ros2 topic hz /camera1/left_camera/color/image_rect_raw
  551  ros2 topic list
  552  source install/setup.bash
  553  ros2 launch jaka_robot_driver jaka_driver.launch.py
  554  source install/setup.bash
  555  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  556  source install/setup.bash
  557  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     color_fps:=20
  558  source install/setup.bash
  559  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     color_fps:=20
  560  source install/setup.bash
  561  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     color_width:=640     color_height:=480     color_fps:=20
  562  ros2 param list /camera1/left_camera | grep fps
  563  source install/setup.bash
  564  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  565  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  566  source install/setup.bash
  567  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  568  source install/setup.bash
  569  ros2 launch orbbec_camera gemini_330_series.launch.py     enable_color:=true     enable_depth:=false     enable_pointcloud:=false     color_fps:=30
  570  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  571  ros2 run demo_nodes_cpp talker &
  572  source install/setup.bash
  573  ros2 launch jaka_robot_driver jaka_driver.launch.py
  574  echo $RMW_IMPLEMENTATION
  575  source install/setup.bash
  576  python3 keyboard_gripper_control.py
  577  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  578  sudo ip link set can1 up
  579  source install/setup.bash
  580  ros2 run jiazhua_driver jiazhua_node
  581  sudo ip link set can1 up
  582  ros2 run jiazhua_driver jiazhua_node
  583  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  584  ls /dev/ttyACM*
  585  sudo slcand -o -c -f -s8 /dev/ttyACM1 can1
  586  sudo ip link set can1 up
  587  ros2 run jiazhua_driver jiazhua_node
  588  source install/setup.bash
  589  ros2 run jiazhua_driver jiazhua_node
  590  source install/setup.bash
  591  ros2 launch jaka_robot_driver jaka_driver.launch.py
  592  source install/setup.bash
  593  ros2 launch jaka_robot_driver jaka_driver.launch.py
  594  echo $RMW_IMPLEMENTATION
  595  source install/setup.bash
  596  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  597  source install/setup.bash
  598  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  599  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     rgb_camera.profile:=848x480x30
  600  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     fps:=30
  601  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     depth_module.color_profile:=640x480x15
  602  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     depth_module.color_profile:=640x480x20
  603  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     depth_module.color_profile:=640x480x15
  604  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     depth_module.color_profile:=640x480x20
  605  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     depth_module.color_profile:=848x480x30
  606  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false &
  607  source install/setup.bash
  608  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false      depth_module.color_profile:=640x480x30
  609  source install/setup.bash
  610  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false  \
  611  source install/setup.bash
  612  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  613  source install/setup.bash
  614  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  615  source install/setup.bash
  616  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false      depth_module.color_profile:=640x480x20
  617  source install/setup.bash
  618  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false  \
  619  source install/setup.bash
  620  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false  \
  621  source install/setup.bash
  622  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  623  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  624  source install/setup.bash
  625  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  626  source install/setup.bash
  627  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=right_camera     camera_namespace:=camera2     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  628  source install/setup.bash
  629  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=right_camera     camera_namespace:=camera2     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  630  source install/setup.bash
  631  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=right_camera     camera_namespace:=camera2     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     depth_module.color_profile:=640x480x15
  632  clear
  633  source install/setup.bash
  634  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  635  source install/setup.bash
  636  ros2 launch orbbec_camera gemini_330_series.launch.py     enable_color:=true     enable_depth:=false     enable_pointcloud:=false     color_fps:=30
  637  source install/setup.bash
  638  ros2 run robot_state_converter converter
  639  source install/setup.bash
  640  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  641  source install/setup.bash 
  642  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  643  source install/setup.bash 
  644  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  645  source install/setup.bash 
  646  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  647  source install/setup.bash 
  648  ros2 run image_view image_view --ros-args --remap image:=/camera1/left_camera/color/image_rect_raw
  649  sudo apt install ros-humble-rqt-image-view
  650  rqt_image_view
  651  sudo apt update
  652  sudo apt install ros-humble-image-view
  653  ros2 run image_view image_view --ros-args --remap image:=/camera1/left_camera/color/image_rect_raw
  654  ros2 service call /start_capture std_srvs/srv/Trigger "{}"
  655  ros2 topic hz /camera1/left_camera/color/image_raw
  656  source install/setup.bash 
  657  ros2 topic hz /camera1/left_camera/color/image_raw
  658  ros2 topic list
  659  ros2 topic hz /camera1/left_camera/color/image_rect_raw
  660  source install/setup.bash 
  661  ros2 param describe /camera1/left_camera depth_module.color_profile
  662  ros2 topic hz /camera1/left_camera/color/image_rect_raw
  663  source install/setup.bash
  664  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  665  source install/setup.bash
  666  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=right_camera     camera_namespace:=camera2     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  667  source install/setup.bash
  668  python3 keyboard_gripper_control.py
  669  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  670  sudo ip link set can1 up
  671  python3 keyboard_gripper_control.py
  672  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  673  sudo ip link set can1 up
  674  python3 keyboard_gripper_control.py
  675  source install/setup.bash 
  676  python3 keyboard_gripper_control.py
  677  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false     depth_module.color_profile:=848x480x15
  678  pkill -f realsense2_camera
  679  lsusb -t
  680  source install/setup.bash
  681  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  682  source install/setup.bash
  683  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
  684  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  685  /opt/apps/roboticsservice/runService.sh
  686  uname -a
  687  python3 --version
  688  conda --version
  689  conda env list
  690  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python && conda create -n xr-robotics python=3.10 -y
  691  source /opt/miniconda3/etc/profile.d/conda.sh && conda activate xr-robotics && python --version
  692  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python && source /opt/miniconda3/etc/profile.d/conda.sh && conda activate xr-robotics && yes y | bash setup_conda.sh --install
  693  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python && source /opt/miniconda3/etc/profile.d/conda.sh && conda activate xr-robotics && python -c "
  694  import sys
  695  print('Python version:', sys.version)
  696  print('---')
  697  try:
  698      import mujoco
  699      print('✅ MuJoCo:', mujoco.__version__)
  700  except Exception as e:
  701      print('❌ MuJoCo:', e)
  702  try:
  703      import placo
  704      print('✅ Placo installed')
  705  except Exception as e:
  706      print('❌ Placo:', e)
  707  try:
  708      import xrobotoolkit_sdk
  709      print('✅ XRoboToolkit SDK installed')
  710  except Exception as e:
  711      print('❌ XRoboToolkit SDK:', e)
  712  try:
  713      import torch
  714      print('✅ PyTorch:', torch.__version__)
  715  except Exception as e:
  716      print('❌ PyTorch:', e)
  717  try:
  718      import xrobotoolkit_teleop
  719      print('✅ xrobotoolkit_teleop installed')
  720  except Exception as e:
  721      print('❌ xrobotoolkit_teleop:', e)
  722  print('---')
  723  print('🎉 All core dependencies installed successfully!')
  724  "
  725  source /opt/miniconda3/etc/profile.d/conda.sh && conda activate xr-robotics && pip list | wc -l
  726  python scripts/simulation/teleop_x7s_placo.py
  727  netstat -tuln | grep 7002
  728  head -50 /root/jwq/XRoboToolkit-Teleop-Sample-Python/assets/jaka/dual_arm.urdf | grep -A 2 "filename"
  729  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python/assets/jaka && sed -i 's|package://dual_arm/meshes/|meshes/|g' dual_arm.urdf && echo "✅ URDF 路径已修复"
  730  grep -n "filename=" /root/jwq/XRoboToolkit-Teleop-Sample-Python/assets/jaka/dual_arm.urdf | head -5
  731  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python && source /opt/miniconda3/etc/profile.d/conda.sh && conda activate xr-robotics && timeout 10 python scripts/simulation/teleop_jaka_placo.py 2>&1 | head -50
  732  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  733  conda ls
  734  conda env listy
  735  conda env list
  736  conda activate xr-robotics
  737  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python
  738  python scripts/simulation/teleop_jaka_placo.py
  739  colcon build --packages-select jaka_robot_interfaces jaka_robot_driver
  740  conda install -c conda-forge empy
  741  colcon build --packages-select jaka_robot_interfaces jaka_robot_driver
  742  pip install empy lark-parser pyparsing pydot numpy
  743  colcon build --packages-select jaka_robot_interfaces jaka_robot_driver
  744  source install/setup.bash 
  745  python scripts/ros2/teleop_jaka_ros2.py
  746  conda activate xr-robotics
  747  source install/setup.bash
  748  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python
  749  source install/setup.bash
  750  ource install/setup.bash
  751  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  752  git clone https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git
  753  sudo dkpg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
  754  dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
  755  /opt/apps/roboticsservice/runService.sh
  756  conda env list
  757  cd ./XRoboToolkit-Teleop-Sample-Python/
  758  ls
  759  python scripts/simulation/teleop_x7s_placo.py
  760  conda env list
  761  conda activate xr-robotics
  762  python scripts/simulation/teleop_x7s_placo.py
  763  python XRoboToolkit-Teleop-Sample-Python/scripts/simulation/teleop_jaka_placo.py
  764  python scripts/simulation/teleop_jaka_placo.py
  765  python XRoboToolkit-Teleop-Sample-Python/scripts/simulation/teleop_jaka_placo.py
  766  python scripts/simulation/teleop_jaka_placo.py
  767  pwd
  768  conda env list
  769  pwd
  770  wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
  771  tree -h
  772  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python && python3 verify_dual_arm_fix.py
  773  conda env list
  774  conda activate xr-robotics
  775  source install/setup.bash 
  776  ros2 topic list
  777  conda activate xr-robotics
  778  source install/setup.bash
  779  /opt/apps/roboticsservice/runService.sh
  780  colcon build --packages-select jaka_robot_driver
  781  source install/setup.bash 
  782  colcon build --packages-select jaka_robot_interfaces jaka_robot_driver
  783  source install/setup.bash 
  784  conda activate xr-robotics
  785  source install/setup.bash 
  786  ros2 launch jaka_robot_driver jaka_driver.launch
  787  source install/setup.bash 
  788  ros2 launch jaka_robot_driver jaka_driver.launch
  789  ros2 launch jaka_robot_driver jaka_driver.launch.py
  790  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zy2/demo/jaka/src/jaka_robot_driver/lib
  791  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/jwq/XRoboToolkit-Teleop-Sample-Python/jaka_robot_driver/lib
  792  source install/setup.bash 
  793  ros2 launch jaka_robot_driver jaka_driver.launch.py
  794  colcon build --packages-select jaka_robot_driver symlink-install
  795  source install/setup.bash
  796  ros2 launch jaka_robot_driver jaka_driver.launch.py
  797  colcon build --packages-select jaka_robot_driver symlink-install
  798  source install/setup.bash
  799  ros2 launch jaka_robot_driver jaka_driver.launch.py
  800  colcon build --packages-select jaka_robot_driver symlink-install
  801  source install/setup.bash
  802  ros2 launch jaka_robot_driver jaka_driver.launch.py
  803  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python && colcon build --packages-select jaka_robot_interfaces --symlink-install
  804  rm -rf /root/jwq/XRoboToolkit-Teleop-Sample-Python/build/jaka_robot_interfaces /root/jwq/XRoboToolkit-Teleop-Sample-Python/install/jaka_robot_interfaces
  805  colcon build --packages-select jaka_robot_interfaces
  806  ls -la /root/jwq/XRoboToolkit-Teleop-Sample-Python/install/jaka_robot_interfaces/lib/libjaka_robot_interfaces__rosidl_generator_py.so
  807  ls -la /root/jwq/XRoboToolkit-Teleop-Sample-Python/install/jaka_robot_interfaces/lib/
  808  chmod +x /root/jwq/XRoboToolkit-Teleop-Sample-Python/run_jaka_teleop.sh
  809  bash -c "source /opt/ros/humble/setup.bash && source /root/jwq/XRoboToolkit-Teleop-Sample-Python/install/setup.bash && echo 'PYTHONPATH=' && echo \$PYTHONPATH && echo '' && echo 'LD_LIBRARY_PATH=' && echo \$LD_LIBRARY_PATH"
  810  mkdir -p /root/jwq/XRoboToolkit-Teleop-Sample-Python/.vscode
  811  ls -la /root/jwq/XRoboToolkit-Teleop-Sample-Python/install/jaka_robot_driver/share/jaka_robot_driver/launch/
  812  conda activate xr-robotics
  813  ros2 topic list
  814  ros2 topic echo /servo_joint_command
  815  source install/setup.bash 
  816  ros2 topic echo /servo_joint_command
  817  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  818  colcon build --packages-select frame_sync
  819  colcon build --packages-select frame_sync_msgs
  820  colcon build --packages-select jiazhua_interfaces
  821  colcon build
  822  colcon build --packages-select jiazhua_driver
  823  colcon build --packages-select frame_sync
  824  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  825  sudo ip link set can1 up
  826  source install/setup.bash
  827  ros2 run jiazhua_driver jiazhua_node
  828  source install/setup.bash 
  829  colcon build
  830  source install/setup.bash 
  831  conda env list
  832  conda activate xr-robotics
  833  source install/setup.bash 
  834  cd /root/jwq/XRoboToolkit-Teleop-Sample-Python && colcon build --packages-select jiazhua_interfaces
  835  colcon build --packages-select jiazhua_driver
  836  ls /root/jwq/XRoboToolkit-Teleop-Sample-Python/install/jiazhua_interfaces/lib/python3.10/site-packages/
  837  find /root/jwq/XRoboToolkit-Teleop-Sample-Python -name "frame_sync_msgs" -type d | head -5
  838  colcon build --packages-select frame_sync
  839  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  840  sudo apt install ros-humble-realsense2-camera
  841  colcon build --symlink-install
  842  colcon build --packages-select orbbec_camera
  843  colcon build --packages-select orbbec_camera_msgs
  844  colcon build --packages-select orbbec_camera
  845  source install/setup.bash
  846  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  847  source install/setup.bash
  848  ros2 launch orbbec_camera gemini_330_series.launch.py     enable_color:=true     enable_depth:=false     enable_pointcloud:=false     color_fps:=30
  849  colcon build --packages-select robot_state_converter
  850  source install/setup.bash 
  851  ros2 run robot_state_converter converter
  852  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  853  conda activate xr-robotics
  854  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  855  conda activate xr-robotics
  856  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  857  source install/setup.bash
  858  ros2 run robot_state_converter converter
  859  source install/setup.bash
  860  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  861  conda deactivate
  862  source install/setup.bash
  863  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  864  source install/setup.bash
  865  ros2 run robot_state_converter converter
  866  conda activate xr-robotics
  867  source install/setup.bash
  868  ros2 launch jaka_robot_driver jaka_driver.launch.py
  869  conda deactivate
  870  source install/setup.bash
  871  ros2 launch jaka_robot_driver jaka_driver.launch.py
  872  conda activate xr-robotics
  873  conda activate xr-robotics
  874  conda activate xr-robotics
  875  conda config --set auto_activate_base false
  876  conda activate xr-robotics
  877  conda config --set auto_activate_base false
  878  conda activate xr-robotics
  879  conda deactivate
  880  conda config --set auto_activate_base false
  881  conda activate xr-robotics
  882  conda activate xr-robotics
  883  conda activate xr-robotics
  884  source install/setup.bash
  885  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/jwq/XRoboToolkit-Teleop-Sample-Python/jaka_robot_driver/lib
  886  ros2 launch jaka_robot_driver jaka_driver.launch.py
  887  source install/setup.bash
  888  ros2 run robot_state_converter converter
  889  conda deactivate
  890  source install/setup.bash
  891  ros2 run robot_state_converter converter
  892  conda activate xr-robotics
  893  source install/setup.bash
  894  python scripts/simulation/teleop_jaka_placo.py
  895  /opt/apps/roboticsservice/runService.sh
  896  python scripts/simulation/teleop_jaka_placo.py
  897  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  898  source install/setup.bash
  899  ros2 launch jaka_robot_driver jaka_driver.launch.py
  900  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/jaka_robot_driver/lib
  901  source install/setup.bash
  902  ros2 launch jaka_robot_driver jaka_driver.launch.py
  903  conda activate xr-robotics
  904  source install/setup.bash
  905  python scripts/ros2/teleop_jaka_ros2.py
  906  conda activate xr-robotics
  907  source install/setup.bash
  908  python scripts/ros2/teleop_jaka_ros2.py
  909  source install/setup.bash
  910  python scripts/ros2/teleop_jaka_ros2.py
  911  source install/setup.bash
  912  python scripts/ros2/teleop_jaka_ros2.py
  913  conda activate xr-robotics
  914  conda activate xr-robotics
  915  source install/setup.bash
  916  export LD_LIBRARY_PATH=/root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/dependencies/XRoboToolkit-PC-Service-Pybind/lib:$LD_LIBRARY_PATH
  917  python scripts/ros2/teleop_jaka_ros2.py
  918  source install/setup.bash
  919  python scripts/ros2/teleop_jaka_ros2.py
  920  conda activate xr-robotics
  921  conda activate xr-robotics
  922  source install/setup.bash
  923  python scripts/ros2/teleop_jaka_ros2.py
  924  source install/setup.bash
  925  ros2 launch jaka_robot_driver jaka_driver.launch.py
  926  conda activate xr-robotics
  927  /opt/apps/roboticsservice/runService.sh
  928  source install/setup.bash
  929  python scripts/ros2/teleop_jaka_ros2.py
  930  colcon build
  931  source install/setup.bash
  932  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  933  git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
  934  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  935  cd /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py serial_no:=_218622278744 camera_name:=left_camera camera_namespace:=camera1 enable_color:=true enable_depth:=false pointcloud.enable:=false
  936  conda activate xr-robotics
  937  source install/setup.bash
  938  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
  939  conda activate xr-robotics
  940  source install/setup.bash
  941  python scripts/ros2/teleop_jaka_ros2.py
  942  /opt/apps/roboticsservice/runService.sh
  943  conda activate xr-robotics
  944  /opt/apps/roboticsservice/runService.sh
  945  conda activate xr-robotics
  946  /opt/apps/roboticsservice/runService.sh
  947  conda activate xr-robotics
  948  source install/setup.bash
  949  sed -i '9s/^/# /' /opt/miniconda3/envs/xr-robotics/etc/conda/activate.d/env_vars.sh
  950  sed -i '9d' /opt/miniconda3/envs/xr-robotics/etc/conda/activate.d/env_vars.sh
  951  sed -i '/^echo /d' /opt/miniconda3/envs/xr-robotics/etc/conda/activate.d/env_vars.sh
  952  conda activate xr-robotics
  953  sudo slcand -o -c -f -s8 /dev/ttyACM1 can1
  954  ls /dev/ttyACM*
  955  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
  956  sudo ip link set can1 up
  957  source install/setup.bash
  958  ros2 run jiazhua_driver jiazhua_node
  959  ls /dev/ttyACM*
  960  sudo slcand -o -c -f -s8 /dev/ttyACM1 can1
  961  source install/setup.bash
  962  ros2 run jiazhua_driver jiazhua_node
  963  source install/setup.bash
  964  ros2 run jiazhua_driver jiazhua_node
  965  source install/setup.bash
  966  ros2 run jiazhua_driver jiazhua_node
  967  source install/setup.bash
  968  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 0.0, speed_left: 0.5, val_right: 0.0, speed_right: 0.5}"
  969  source install/setup.bash
  970  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 1.0, speed_left: 0.5, val_right: 1.0, speed_right: 0.5}"
  971  conda activate xr-robotics
  972  source install/setup.bash
  973  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 0.0, speed_left: 0.5, val_right: 0.0, speed_right: 0.5}"
  974  conda activate xr-robotics
  975  source install/setup.bash
  976  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 0.0, speed_left: 0.5, val_right: 0.0, speed_right: 0.5}"
  977  source install/setup.bash
  978  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 1.0, speed_left: 0.5, val_right: 1.0, speed_right: 0.5}"
  979  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 0.0, speed_left: 0.5, val_right: 0.0, speed_right: 0.5}"
  980  sudo slcand -o -c -f -s8 /dev/ttyACM2 can1
  981  ls /dev/ttyACM*
  982  source install/setup.bash
  983  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 1.0, speed_left: 0.5, val_right: 1.0, speed_right: 0.5}"
  984  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  985  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
  986  touch /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/realsense2_camera/share/realsense2_camera/local_setup.bash
  987  ls -la /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/realsense2_camera/share/realsense2_camera/local_setup.bash
  988  source install/setup.bash
  989  cd /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python && source install/setup.bash && timeout 10 python3 scripts/ros2/teleop_jaka_ros2.py 2>&1 | head -50
  990  conda activate xr-robotics
  991  source install/setup.bash
  992  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 0.0, speed_left: 0.5, val_right: 0.0, speed_right: 0.5}"
  993  sudo ip link set can1 up
  994  ls /dev/ttyACM*
  995  sudo slcand -o -c -f -s8 /dev/ttyACM1 can1
  996  sudo ip link set can1 up
  997  source install/setup.bash
  998  ros2 run jiazhua_driver jiazhua_node
  999  sudo ip link set can1 up
 1000  source install/setup.bash
 1001  ros2 run jiazhua_driver jiazhua_node
 1002  ls /dev/ttyACM*
 1003  sudo slcand -o -c -f -s8 /dev/ttyACM2 can1
 1004  sudo ip link set can1 up
 1005  source install/setup.bash
 1006  ros2 run jiazhua_driver jiazhua_node
 1007  ls /dev/ttyACM*
 1008  sudo slcand -o -c -f -s8 /dev/ttyACM2 can1
 1009  sudo ip link set can1 up
 1010  source install/setup.bash
 1011  ros2 run jiazhua_driver jiazhua_node
 1012  ls /dev/ttyACM*
 1013  sudo slcand -o -c -f -s8 /dev/ttyACM2 can1
 1014  sudo ip link set can1 up
 1015  source install/setup.bash
 1016  ros2 run jiazhua_driver jiazhua_node
 1017  ls /dev/ttyACM*
 1018  sudo slcand -o -c -f -s8 /dev/ttyACM3 can1
 1019  sudo ip link set can1 up
 1020  source install/setup.bash
 1021  ros2 run jiazhua_driver jiazhua_node
 1022  sudo ip link set can1 up
 1023  source install/setup.bash
 1024  ros2 run jiazhua_driver jiazhua_node
 1025  ls /dev/ttyACM*
 1026  sudo ip link set can1 up
 1027  source install/setup.bash
 1028  ros2 run jiazhua_driver jiazhua_node
 1029  ls /dev/ttyACM*
 1030  sudo slcand -o -c -f -s8 /dev/ttyACM4 can1
 1031  sudo ip link set can1 up
 1032  source install/setup.bash
 1033  ros2 run jiazhua_driver jiazhua_node
 1034  conda activate xr-robotics
 1035  source install/setup.bash
 1036  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
 1037  sudo ip link set can1 up
 1038  source install/setup.bash
 1039  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
 1040  colcon build --packages-select frame_sync
 1041  source install/setup.bash
 1042  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
 1043  colcon build --packages-select frame_sync
 1044  source install/setup.bash
 1045  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
 1046  colcon build --packages-select frame_sync
 1047  source install/setup.bash
 1048  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
 1049  conda activate xr-robotics
 1050  source install/setup.bash
 1051  ros2 run robot_state_converter converter
 1052  conda activate xr-robotics
 1053  source install/setup.bash
 1054  ros2 launch orbbec_camera gemini_330_series.launch.py     enable_color:=true     enable_depth:=false     enable_pointcloud:=false     color_fps:=30
 1055  conda activate xr-robotics
 1056  source install/setup.bash
 1057  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=right_camera     camera_namespace:=camera2     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
 1058  conda activate xr-robotics
 1059  source install/setup.bash
 1060  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
 1061  source install/setup.bash
 1062  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
 1063  conda activate xr-robotics
 1064  pkill -f RoboticsServiceProcess
 1065  /opt/apps/roboticsservice/runService.sh
 1066  source install/setup.bash
 1067  python scripts/ros2/teleop_jaka_ros2.py
 1068  source install/setup.bash
 1069  python scripts/ros2/teleop_jaka_ros2.py
 1070  source install/setup.bash
 1071  python scripts/ros2/teleop_jaka_ros2.py
 1072  source install/setup.bash
 1073  python scripts/ros2/teleop_jaka_ros2.py
 1074  source install/setup.bash
 1075  python scripts/ros2/teleop_jaka_ros2.py
 1076  source install/setup.bash
 1077  python scripts/ros2/teleop_jaka_ros2.py
 1078  source install/setup.bash
 1079  python scripts/ros2/teleop_jaka_ros2.py
 1080  source install/setup.bash
 1081  python scripts/ros2/teleop_jaka_ros2.py
 1082  source install/setup.bash
 1083  python scripts/ros2/teleop_jaka_ros2.py
 1084  source install/setup.bash
 1085  ./run_jaka_teleop.sh
 1086  python scripts/ros2/teleop_jaka_ros2.py
 1087  source install/setup.bash
 1088  python scripts/ros2/teleop_jaka_ros2.py
 1089  source install/setup.bash
 1090  python scripts/ros2/teleop_jaka_ros2.py
 1091  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
 1092  head -50 /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data/0036/obs.json
 1093  python3 -c "import json; data = json.load(open('/root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data/0036/obs.json')); print('Keys:', data.keys() if isinstance(data, dict) else 'list'); print('First item keys:', data[0].keys() if isinstance(data, list) else data['data'][0].keys() if 'data' in data else 'unknown'); print('Sample:', json.dumps(data[0] if isinstance(data, list) else data['data'][0], indent=2)[:1000])"
 1094  conda activate xr-robotics
 1095  source install/setup.bash 
 1096  # 启动命令
 1097  python vla/jaka_vla.py     --source-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data     --output-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data_lerobot     --repo-id "jaka_left_arm_only"     --exclude-cameras "right"     --arm-keys "left_arm_joint,left_arm_gripper_state"     --test-episodes 0     --fps 20
 1098  pip install tqdm
 1099  python vla/jaka_vla.py     --source-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data     --output-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data_lerobot     --repo-id "jaka_left_arm_only"     --exclude-cameras "right"     --arm-keys "left_arm_joint,left_arm_gripper_state"     --test-episodes 0     --fps 20
 1100  conda activate xr-robotics
 1101  source install/setup.bash
 1102  ros2 launch jaka_robot_driver jaka_driver.launch.py
 1103  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
 1104  pip install lerobot
 1105  conda info --envs
 1106  conda activate xr-robotics && pip install lerobot
 1107  df -h
 1108  du -sh /root/* 2>/dev/null | sort -hr | head -20
 1109  du -sh ~/.cache 2>/dev/null
 1110  conda clean --all --dry-run
 1111  du -sh ~/.cache/pip 2>/dev/null
 1112  du -sh ~/.cache/huggingface 2>/dev/null
 1113  pip cache purge
 1114  conda clean --all -y
 1115  df -h
 1116  pip install lerobot
 1117  df -h
 1118  du -h --max-depth=1 /root 2>/dev/null | sort -hr | head -20
 1119  du -h --max-depth=2 /root/jwq 2>/dev/null | sort -hr | head -20
 1120  du -h --max-depth=2 /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python 2>/dev/null | sort -hr | head -20
 1121  du -h --max-depth=1 /root/jwq/vr/data_tools 2>/dev/null | sort -hr | head -20
 1122  rm -rf /root/jwq/vr/data_tools/data
 1123  df -h
 1124  df -h | grep -E "overlay|/dev/"
 1125  df -h /
 1126  df -h
 1127  du -sh /root/jwq
 1128  python vla/jaka_vla.py --source-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data --output-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data_lerobot --repo-id "jaka_left_arm_only" --exclude-cameras "right" --arm-keys "left_arm_joint,left_arm_gripper_state" --test-episodes 0 --fps 20
 1129  python vla/jaka_vla.py --source-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data --output-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data_lerobot --repo-id "jaka_left_arm_only" --exclude-cameras "right" --arm-keys "left_arm_joint,left_arm_gripper_state" --test-episodes 0 --fps 20 --no-flip
 1130  python vla/jaka_vla.py     --source-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data     --output-dir /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data_lerobot     --repo-id "jaka_left_arm_only"     --exclude-cameras "right"     --arm-keys "left_arm_joint,left_arm_gripper_state"     --test-episodes 0     --fps 20       --no-flip 
 1131  source install/setup.bash
 1132  ros2 launch jaka_robot_driver jaka_driver.launch.py
 1133  conda activate xr-robotics
 1134  source install/setup.bash
 1135  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
 1136  source install/setup.bash
 1137  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
 1138  source install/setup.bash
 1139  ros2 launch jaka_robot_driver jaka_driver.launch.py
 1140  conda activate xr-robotics
 1141  source install/setup.bash
 1142  ros2 run robot_state_converter converter
 1143  source install/setup.bash
 1144  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
 1145  conda activate xr-robotics
 1146  ls /dev/ttyACM*
 1147  sudo slcand -o -c -f -s8 /dev/ttyACM4 can1
 1148  sudo ip link set can1 up
 1149  source install/setup.bash
 1150  ros2 run jiazhua_driver jiazhua_node
 1151  conda activate xr-robotics
 1152  ros2 topic list
 1153  ros2 topic hz /camera1/left_camera/color/image_rect_raw
 1154  source install/setup.bash 
 1155  ros2 topic hz /camera1/left_camera/color/image_rect_raw
 1156  ros2 topic hzecho camera1/left_camera/color/image_rect_raw
 1157  ros2 topic echo /camera1/left_camera/color/image_rect_raw
 1158  ros2 topic hz /camera1/left_camera/color/image_rect_raw
 1159  ros2 topic echo /camera1/left_camera/color/image_rect_raw
 1160  ros2 topic hz /camera1/left_camera/color/image_rect_raw
 1161  ros2 topic list
 1162  ros2 topic echo /robot_state_dual
 1163  ros2 topic list
 1164  ros2 topic hz /robot_state_dual
 1165  ros2 topic list
 1166  ros2 topic echo /left_arm/jiazhua_state
 1167  ros2 topic hz /left_arm/jiazhua_state
 1168  ros2 topic list
 1169  ros2 topic echo /left_arm/joint_states
 1170  ros2 topic echo /left_arm/jiazhua_state
 1171  ros2 topic list
 1172  ros2 topic hz /camera1/left_camera/color/image_rect_raw
 1173  source install/setup.b
 1174  source install/setup.bash 
 1175  ros2 topic hz /camera1/left_camera/color/image_rect_raw
 1176  source install/setup.bash
 1177  ros2 launch orbbec_camera gemini_330_series.launch.py     enable_color:=true     enable_depth:=false     enable_pointcloud:=false     color_fps:=30
 1178  conda activate xr-robotics
 1179  ros2 topic list
 1180  ros2 topic echo /camera/color/image_raw
 1181  ros2 topic hz /camera/color/image_raw
 1182  source install/setup.bash
 1183  ros2 topic list
 1184  ros2 topic hz /camera1/left_camera/color/image_rect_raw
 1185  ros2 topic echo /camera1/left_camera/color/image_rect_raw
 1186  head -100 /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data/0036/obs.json | python3 -m json.tool 2>/dev/null || head -100 /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/data/0036/obs.json
 1187  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
 1188  conda activate xr-robotics
 1189  source install/setup.bash 
 1190  conda activate xr-robotics
 1191  pip show numpy
 1192  conda activate xr-robotics
 1193  conda deactivate
 1194  conda create -n client
 1195  conda create -n client python=3.11
 1196  conda activate client
 1197  pip install numpy==1.26
 1198  pip install requests
 1199  conda activate client2
 1200  pip show numpy
 1201  conda activate xr-robotics
 1202  conda activate client
 1203  source install/setup.bash 
 1204  conda deactivate
 1205  conda create -n client2 python=3.10 
 1206  conda activate client2
 1207  conda create -n client2 python=3.10 
 1208  conda activate client2
 1209  pip install numpy==1.26
 1210  source install/setup.bash 
 1211  pip install requests
 1212  source install/setup.bash 
 1213  pip install cv2
 1214  source install/setup.bash 
 1215  pip show numpy
 1216  conda install -n client2 numpy=1.26 -y
 1217  pip show numpy
 1218  source install/setup.bash 
 1219  source /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/setup.bash
 1220  pip install json
 1221  pip install json_numpy
 1222  conda activate client2
 1223  source install/setup.bash
 1224  ros2 run robot_state_converter converter
 1225  conda activate client2
 1226  source install/setup.bash 
 1227  ros2 topic echo /left_arm/gripper_state
 1228  ros2 topic list
 1229  ros2 topic echo /left_arm/gripper_state
 1230  ros2 topic echo /left_arm/jiazhua_state
 1231  ros2 topic echo /left_arm/gripper_state
 1232  ros2 topic echo /left_arm/jiazhua_state
 1233  ping 192.168.1.88
 1234  source install/setup.bash
 1235  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 0.0, speed_left: 0.5, val_right: 0.0, speed_right: 0.5}"
 1236  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 1.0, speed_left: 0.5, val_right: 1.0, speed_right: 0.5}"
 1237  ros2 topic list
 1238  ros2 service list
 1239  ros2 topic list
 1240  ros2 topic info /multi_movj_cmd
 1241  source install/setup.bash 
 1242  rqt
 1243  conda activate client2
 1244  ros2 service call /start_capture std_srvs/srv/Trigger "{}"
 1245  conda activate client2
 1246  source install/setup.bash
 1247  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 1.0, speed_left: 0.5, val_right: 1.0, speed_right: 0.5}"
 1248  source install/setup.bash
 1249  ros2 topic pub --once /jiazhua_cmd jiazhua_interfaces/msg/JiaZhuaDualCmd "{val_left: 1.0, speed_left: 0.5, val_right: 1.0, speed_right: 0.5}"
 1250  source install/setup.bash
 1251  ros2 launch jaka_robot_driver jaka_driver.launch.py
 1252  colcon build --packages-select jaka_robot_driver symlink-install
 1253  source install/setup.bash
 1254  ros2 launch jaka_robot_driver jaka_driver.launch.py
 1255  colcon build --packages-select jaka_robot_interfaces jaka_robot_driver
 1256  source install/setup.bash
 1257  ros2 launch jaka_robot_driver jaka_driver.launch.py
 1258  colcon build --packages-select jaka_robot_driver symlink-install
 1259  source install/setup.bash
 1260  ros2 launch jaka_robot_driver jaka_driver.launch.py
 1261  conda activate xr-robotics
 1262  source install/setup.bash
 1263  ros2 launch realsense2_camera rs_launch.py     serial_no:=_218622278744     camera_name:=left_camera     camera_namespace:=camera1     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
 1264  conda activate client2
 1265  source install/setup.bash
 1266  ros2 launch realsense2_camera rs_launch.py     serial_no:=_230322276887     camera_name:=right_camera     camera_namespace:=camera2     enable_color:=true     enable_depth:=false     pointcloud.enable:=false
 1267  conda activate xr-robotics
 1268  source install/setup.bash
 1269  ros2 launch orbbec_camera gemini_330_series.launch.py     enable_color:=true     enable_depth:=false     enable_pointcloud:=false     color_fps:=30
 1270  conda activate xr-robotics
 1271  ls /dev/ttyACM*
 1272  sudo slcand -o -c -f -s8 /dev/ttyACM4 can1
 1273  sudo ip link set can1 up
 1274  source install/setup.bash
 1275  ros2 run jiazhua_driver jiazhua_node
 1276  ls /dev/ttyACM*
 1277  sudo slcand -o -c -f -s8 /dev/ttyACM0 can1
 1278  sudo ip link set can1 up
 1279  source install/setup.bash
 1280  ros2 run jiazhua_driver jiazhua_node
 1281  conda activate client2
 1282  conda create -n client3 python=3.10
 1283  conda deactviate
 1284  conda deactivate
 1285  conda create -n client3 python=3.10
 1286  conda env list
 1287  conda create -n client3 python=3.10
 1288  ros2 topic list
 1289  ros2 topic echo /left_arm/joint_states
 1290  ros2 topic echo /robot_state_dual
 1291  source install/setup.bash 
 1292  ros2 topic echo /robot_state_dual
 1293  ros2 topic list
 1294  ros2 topic echo /left_arm/joint_states
 1295  ros2 topic echo /left_arm/jiazhua_state
 1296  source install/setup.bash 
 1297  ros2 topic echo /left_arm/jiazhua_state
 1298  ros2 topic list
 1299  ros2 topic echo /left_arm/gripper_state
 1300  ros2 topic echo /left_arm/jiazhua\_state
 1301  ros2 topic echo /left_arm/jiazhua_state
 1302  source install/setup.bash 
 1303  ros2 topic echo /left_arm/jiazhua_state
 1304  ros2 topic echo /left_arm/gripper_state
 1305  ros2 topic list
 1306  ros2 topic echo /left_arm/jiazhua_state
 1307  source install/setup.bash
 1308  ros2 run robot_state_converter converter
 1309  conda activate client2
 1310  source install/setup.bash
 1311  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
 1312  source install/setup.bash
 1313  ros2 run frame_sync frame_sync_node   --ros-args   --params-file "$(ros2 pkg prefix frame_sync)/share/frame_sync/config/frame_sync.yaml"   --log-level info
 1314  conda activate client2
 1315  conda deactivate
 1316  conda activate xr-robotics
 1317  source install/setup.bash
 1318  /opt/apps/roboticsservice/runService.sh
 1319  source install/setup.bash
 1320  python scripts/ros2/teleop_jaka_ros2.py
 1321  /opt/apps/roboticsservice/runService.sh
 1322  python scripts/ros2/teleop_jaka_ros2.py
 1323  /opt/apps/roboticsservice/runService.sh
 1324  source install/setup.bash
 1325  python scripts/ros2/teleop_jaka_ros2.py
 1326  cd /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python && colcon build --packages-select jaka_robot_interfaces jiazhua_interfaces --symlink-install
 1327  cd /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python && rm -rf build/jaka_robot_interfaces build/jiazhua_interfaces install/jaka_robot_interfaces install/jiazhua_interfaces
 1328  colcon build --packages-select jaka_robot_interfaces jiazhua_interfaces
 1329  source /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/setup.bash && echo "工作空间已加载"
 1330  ros2 interface show jaka_robot_interfaces/msg/ServoJointCommand
 1331  find /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install -name "libjaka_robot_interfaces__rosidl_generator_py.so"
 1332  chmod +x /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/run_http_client.sh
 1333  cd /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python && python3 -c "import sys; sys.path.insert(0, 'install/jaka_robot_interfaces/lib/python3.10/site-packages'); from jaka_robot_interfaces.msg import ServoJointCommand; print('✅ 导入成功')"
 1334  cd /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python && python3 vla/http_client.py --help 2>&1 | head -20
 1335  python3 test_import.py
 1336  pip install "numpy<2.0"
 1337  strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX
 1338  strings /opt/miniconda3/envs/client2/lib/libstdc++.so.6 | grep GLIBCXX
 1339  mv /opt/miniconda3/envs/client2/lib/libstdc++.so.6 /opt/miniconda3/envs/client2/lib/libstdc++.so.6.backup
 1340  ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /opt/miniconda3/envs/client2/lib/libstdc++.so.6
 1341  ls -la /opt/miniconda3/envs/client2/lib/libstdc++.so.6
 1342  source /opt/miniconda3/bin/activate client2 && python -c "import rclpy; print('✓ rclpy导入成功！')"
 1343  source /opt/miniconda3/bin/activate client2 && timeout 3 python vla/http_client.py 2>&1 || true
 1344  source /opt/miniconda3/bin/activate client2 && pip install opencv-python
 1345  find /opt/ros/humble -name "*jaka_robot*" 2>/dev/null | head -20
 1346  find ~ -name "*jaka_robot*" -o -name "*jiazhua*" 2>/dev/null | head -20
 1347  find ~ -type f -name "setup.bash" | grep -E "(install|devel)" | head -10
 1348  ls -la /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/
 1349  chmod +x /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/run_http_client.sh
 1350  find /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install -name "*jaka_robot_interfaces*.so" 2>/dev/null
 1351  source /opt/ros/humble/setup.bash && source /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/setup.bash && echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH" | grep -o "jaka_robot_interfaces"
 1352  bash -c "source /opt/miniconda3/bin/activate client2 && source /opt/ros/humble/setup.bash && source /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/setup.bash && timeout 2 python vla/http_client.py 2>&1" || true
 1353  source /opt/miniconda3/bin/activate client2 && pip install "numpy<2" --force-reinstall
 1354  source /opt/miniconda3/bin/activate client2 && pip install "numpy<2" --force-reinstall -i https://pypi.tuna.tsinghua.edu.cn/simple
 1355  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
 1356  conda install -n client2 numpy=1.26 -y
 1357  conda activate client2 && pip install numpy==1.26.4
 1358  conda run -n client2 python -c "import numpy; print(f'Numpy版本: {numpy.__version__}')"
 1359  find /opt/ros/humble -name "*jaka_robot_interfaces*" 2>/dev/null | head -20
 1360  ls -la /root/jwq/xr/ 2>/dev/null | grep -i jaka
 1361  ls -la /root/jwq/xr/
 1362  ls -la /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/
 1363  echo $AMENT_PREFIX_PATH
 1364  source /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/setup.bash && echo "工作空间已激活" && echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
 1365  python3 -c "from jaka_robot_interfaces.msg import ServoJointCommand, JointValue; from jiazhua_interfaces.msg import JiaZhuaDualCmd; print('✅ 所有消息类型导入成功！')"
 1366  ls -1 /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/
 1367  . "\root\.cursor-server\bin\6aa7b3af0d578b9a3aa3ab443571e1a51ebb4e80/out/vs/workbench/contrib/terminal/common/scripts/shellIntegration-bash.sh"
 1368  history
 1369  history > history.md
