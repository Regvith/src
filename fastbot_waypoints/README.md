# This is a simulation for fastbot, here the testing takes place using gtest so the conditions for failing and passing will be listed below

# Firstly source the ros2_ws in every terminal that is being used
    - cd ros2_ws/ && source install/setup.bash  

# launch the gazebo world then using 
    - source ~/ros2_ws/install/setup.bash
    - ros2 launch fastbot_gazebo one_fastbot_room.launch.py

# colcon build and start the action server 
    - cd ros2_ws/ &&colcon build && source install/setup.bash
    - ros2 run fastbot_waypoints fastbot_action_server

# run the test unit 
    - cd ~/ros2_ws && colcon build && source install/setup.bash
    - colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
    - colcon test-result --all

# Conditions for passing and failing
    # For passing
        - The robot must have values with regards to visual inspection. For instance, if robot is at (0,0)
          and there is a sofa' side at (1,1), any values less than 1,1 for x,y in test code will work. 
        - In conclusion, it is essential to make sure that the selcted the values of x,y must be away from obstacles.
        Passing conditions
        TEST_F(WaypointActionTest, FirstWaypoint) { sendAndCheckGoal(0.0, 0.1); }

        TEST_F(WaypointActionTest, SecondWaypoint) { sendAndCheckGoal(0.1, 0.1); }
    # For failing
        - The robot times-out after 40s and the robot moves at a fixed velocity of 0.6 m/s linearly and either 0.65 or -0.65 angularly.
          trot = pi/w = 3.1457/0.65 = 4.83 s ; td = d/v ; Ttot = trot + td 
          so, if Ttot >40 it will fail, and dist of x and y > 21 provided there is no obstacles
        Failing conditions
        TEST_F(WaypointActionTest, FirstWaypoint) { sendAndCheckGoal(1.0, 0.1); }

