ros2 launch basic_mobile_robot basic_mobile_bot_v4.launch.py                                                                                                                                                                                                   
ros2 launch slam_toolbox online_async_launch.py params_file:=.//home/rima/roboti_ws/src/basic_mobile_robot/config/mapper_params_online_async.yaml
 
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=True map:=/home/rima/roboti_ws/src/basic_mobile_robot/maps/map.yaml                                                                                                
source /opt/ros/humble/setup.bash                                                                                                                                                                                                                                                                                                                                                                                               
                                                 
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz                                                                                                                                   

"I would like to modify the existing code in the scripts to make the robot move autonomously in the environment and avoid obstacles, please."







