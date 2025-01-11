# Visualize Robot

To test the robot model in `air_description`, run the following command:

```
ros2 launch air_description visualize_robot.launch.py
```

# Launching robot model with MoveGroup


**NOTE: To run this you need the ros2_kortex package in your build**

To launch the move_group object for the starting moveit, you can run the following:

```
ros2 launch air_moveit_config kortex_moveit.launch.py 
```

To add the obstacle to the scene, run the following:

```
ros2 run air_moveit_config kinova_planning_scene
```