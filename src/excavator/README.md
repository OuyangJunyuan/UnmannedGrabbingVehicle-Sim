# README

## 使用

1. `roslaunch exacavator gazebo_world.launch`

2. 待世界加载完毕后，运行`roslaunch exacavator gazebo_models.launch`

3. 如果第二部中 controller_spawner 启动失败，运行

   ```
   rosrun controller_manager controller_spawner 
    joint_state_controller
             WaistJoint_position_controller
             ArmBaseJoint_position_controller
             Arm12Joint_position_controller
             Arm23Joint_position_controller
   ```

注意：以上过程特别是第二部会比较慢(可能是LiDAR仿真太吃内存，目前占用11G内存)，请耐心加载。