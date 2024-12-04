package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;
import java.util.List;

@Config
public class RobotCfg {
    public static int Arm_Pos = 0; //arm setpoint
    public static int Arm_increment = 3; //Angle (deg) the arm rotates by
    public static double WHEEL_SPEED = 1; //Maximum wheel speed (used to slow down the robot)

    //Viper Slide configs
    public static int Viper_maxExtend = 5300; //Maximum encoder tick on the viperslide extension
    public static int Viper_minExtend = 0; //Minimum encoder tick on the viperslide extension
    public static int Viper_Increment = 30; //Increment used by viperslide manual extension
    public static int Viper_Pos = 0; //Current encoder tick position of the viperslide

    //Arm Limiting
    public static final int Min_Arm = 0;
    public static final int Max_Arm = 900;

    //Timeout stuff
    public static int viperslide_Timeout = 0;

    //Presets
    public static int Intake_Viperslide = 0;
    public static int Intake_Arm = 0;

    public static int Basket_High_Viperslide = 0;
    public static int Basket_High_Arm = 0;

    public static int Basket_Low_Viperslide = 0;
    public static int Basket_Low_Arm = 0;

    public static int Specemin_Low_Viperslide = 0;
    public static int Specemin_Low_Arm = 0;

    public static int Specemin_High_Viperslide = 0;
    public static int Specemin_High_Arm = 0;

    public static List<Integer> Intake = Arrays.asList(0, 0);
    public static List<Integer> Basket_High = Arrays.asList(0, 0);
    public static List<Integer> Basket_Low = Arrays.asList(0, 0);
    public static List<Integer> Specemin_Low = Arrays.asList(0, 0);
    public static List<Integer> Specemin_High = Arrays.asList(0, 0);;


}
