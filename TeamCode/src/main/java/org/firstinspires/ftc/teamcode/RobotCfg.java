package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Preset;

@Config
public class RobotCfg {
    public static double WHEEL_SPEED = 1; //Maximum wheel speed (used to slow down the robot, or not)

    //arm configs
    public static int Arm_Pos = 0; //arm setpoint
    public static int Min_Arm = 130; //lowest point arm can travel to
    public static int Max_Arm = 1890; //highest point arm can travel too
    public static int Arm_increment = 10; //Angle (tick) the arm rotates by

    //Viper Slide configs
    public static int Viper_Pos = 0; //Current encoder tick position of the viperslide
    public static int Viper_maxExtend = 3118; //6574 under max upwards, Maximum encoder tick on the viperslide extension
    public static int Viper_minExtend = 0; //Minimum encoder tick on the viperslide extension
    public static int Viper_Increment = 30; //Increment used by viperslide manual extension

    //Pose Storage
    public static Pose PoseStorage = new Pose(0,0,Math.toRadians(0)); // used to transfer pose between auto and teleOP

    //Presets
    public static class Presets {
        public static Preset Intake_Sample = new Preset(110,0,0); // preset for intaking a sample (currently certain death)
        public static Preset Intake_Specemin = new Preset(850,0,0); // preset for intaking a specemin (currently unknown death)
        public static Preset Basket_Low = new Preset(0,0,0); // preset for scoring in the low basket
        public static Preset Basket_High = new Preset(1650,3118,0); // preset for scoring in the high basket
        public static Preset Specemin_Low = new Preset(0,0,0); // preset for scoring a specemin on the low bar
        public static Preset Specemin_High = new Preset(540,3660,0); // preset for scoring a specemin on the high bar
    }
}
