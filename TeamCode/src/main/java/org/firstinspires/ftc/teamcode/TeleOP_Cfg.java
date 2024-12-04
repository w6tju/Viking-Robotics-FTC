package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;


@Config
public class TeleOP_Cfg {

    /*
    Nehanth's Configs
    - 1 Wheel speed
    - 3 arm increment
    - 30 viper increment

    Suhas's configs
    - 1 Wheel Speed
    - 10 arm increment
    - UNKNOWN viper increment
     */
    public static double WHEEL_SPEED = 1; //Maximum wheel speed (used to slow down the robot)
    public static int Arm_increment = 3; //Angle (deg) the arm rotates by
    public static int Viper_Increment = 30; //Increment used by viperslide manual extension
}
