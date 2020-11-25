# IVR_Assignment
This is the coursework repository for Introduction to Vision and Robotics

By **Michitatsu Sato s1807428** and **Ivan Sun s1800640**

**Task 2.1**:

To run the Task2.1, first run the **image1.py** and **image2.py**. Next run the **state_estimation.py** for estimating end-effector position and joints angles. You also need to run **move_robot.py** for moving robot trajectory. The estimated joints will be published on "/joints_ang" (or separately on ["/joints_ang2","/joints_ang3","/joints_ang4"]). The actual joint angles will be published on ["/robot/joint2_position_controller/command","/robot/joint3_position_controller/command","/robot/joint4_position_controller/command"] respectively.By using "rqt" command, the estimated joint angles and actual joint angles can be compared by ploting them.

    - Task 2.2:
      To run the Task2.2, first run the **image1.py** and **image2.py**. Next run the **state_estimation.py** for estimating target position. 
      The estimated target position will be published on "/target_pos".

    - Task 3.1:
      To run the Task3.1, just run the "task3.1_forward_kinematics_test.py" with python3 command. It will present the 10 result of the computed coordinate 
      of the end-efffector with 10 different joints angles states. To get the estimated end-effector's position by computer vision, first run the **image1.py** 
      and **image2.py**. Next run the **state_estimation.py**. Using "rostopic pub" command to set the joints angles to arbitrary state (10 test example), and 
      obtain estimated end-effector position of computer vision from "/end_pos" topic published by **state_estimation.py**. 
      Forward kinematics function takes 4 argumants which is correspond to angle (radian) of the 4 joints (joints1, joints2, joints3, joints4 respectively) 

    - Task 3.2:
      To run the task3.2, first run the **image1.py** and **image2.py**. Next run the **state_estimation.py** for estimating end-effector position and target position. 
      Then run the **robot_control.py** to control and move the robot's joints for end-effector to trace the target. For convenient, **task3.2_plot_publisher.py** is
      implemented. By running **task3.2_plot_publisher.py** after running **state_estimation.py**, it will publish the position of the target and end-effector estimated 
      from commputer vision. Each position will be published separately on axis which will make it easier for ploting result (xyz-coordinate of the target will be 
      published on "/target_x","/target_y","target_z", xyz-coordinate of the end-effector will be published on "end-eff_x","end-eff_y","end-eff_z").
      Robot sometimes corrupse and stop moving. Recommended to run the **robot_control.py** when target is in the region of negative x-coordinate and positive y-coordinate.

    - Task 4.3:
      ※ robot.urbf file have to be replaced for black robot.
      To run the Task4.3, first run the **image1_Q4.3.py** and **image2_Q4.3.py**. Next run the **state_estimation.py** for estimating joints angles. 
      You also need to run **move_robot.py** for moving robot trajectory. The estimated joints will be published on "/joints_ang" (or separately on
      ["/joints_ang2","/joints_ang3","/joints_ang4"]). The actual joint angles will be published on ["/robot/joint2_position_controller/command",
      "/robot/joint3_position_controller/command","/robot/joint4_position_controller/command"] respectively.By using "rqt" command, 
      the estimated joint angles and actual joint angles can be compared by ploting them.
      
