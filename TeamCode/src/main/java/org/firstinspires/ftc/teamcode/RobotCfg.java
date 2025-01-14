package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class RobotCfg {
    public static int Arm_Pos = 0; //arm setpoint
    public static int Arm_increment = 10; //Angle (deg) the arm rotates by
    public static double WHEEL_SPEED = 1; //Maximum wheel speed (used to slow down the robot)

    //Viper Slide configs
    public static int Viper_maxExtend = 3118; //6574 under max upwards, Maximum encoder tick on the viperslide extension
    public static int Viper_minExtend = 0; //Minimum encoder tick on the viperslide extension
    public static int Viper_Increment = 30; //Increment used by viperslide manual extension
    public static int Viper_Pos = 0; //Current encoder tick position of the viperslide

    //Arm Limiting
    public static final int Min_Arm = 130;
    public static final int Max_Arm = 1890;

    //Pose Storage
    public static Pose PoseStorage = new Pose(0,0,Math.toRadians(0));

    @Config("RobotCfg")
    //Presets
    public static class Presets {
        @Config("Presets")
        public static class Intake_Sample {
            public static int Arm = 110;
            public static int Viperslide = 0;
            public static double Wrist = 0;
        }

        @Config("Presets")
        public static class Intake_Specemin {
            public static int Arm = 850;
            public static int Viperslide = 0;
            public static double Wrist = 0;
        }

        @Config("Presets")
        public static class Basket_Low {
            public static int Arm = 0;
            public static int Viperslide = 0;
            public static double Wrist = 0;
        }

        @Config("Presets")
        public static class Basket_High {
            public static int Arm = 1650;
            public static int Viperslide = 3118;
            public static double Wrist = 0;
        }

        @Config("Presets")
        public static class Specemin_Low {
            public static int Arm = 0;
            public static int Viperslide = 0;
            public static double Wrist = 0;
        }

        @Config("Presets")
        public static class Specemin_High {
            public static int Arm = 540;
            public static int Viperslide = 3660;
            public static double Wrist = 0;
        }
    }
}
