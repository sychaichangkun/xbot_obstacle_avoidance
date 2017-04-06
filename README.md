xbot_obstacle_avoidance
===============
To implement obstacle avoidance for xbot, a finite-state machine is designed.

subscribed topics:

    /scan

    /cmd_vel_mux/input/navi

published topics:

    /cmd_vel_mux/input/safety_controller


    ```bash
    roslaunch xbot_obstacle_avoidance xbot_obstacle_avoidance
    ```


![marker](https://raw.githubusercontent.com/sychaichangkun/xbot_obstacle_avoidance/master/xbot_fsm.png)


