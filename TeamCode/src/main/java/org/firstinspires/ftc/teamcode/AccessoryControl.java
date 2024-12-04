package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotCfg.Arm_Pos;
import static org.firstinspires.ftc.teamcode.RobotCfg.Arm_increment;
import static org.firstinspires.ftc.teamcode.RobotCfg.Max_Arm;
import static org.firstinspires.ftc.teamcode.RobotCfg.Min_Arm;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_Increment;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_Pos;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_maxExtend;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_minExtend;
import static org.firstinspires.ftc.teamcode.RobotCfg.viperslide_Timeout;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AccessoryControl {
    //Physical Objects
    DcMotor armMotor; //motor in arm tower
    DcMotor viperSlide; //Motor that runs viperslide
    Servo wrist; //wrist servo
    CRServo intake_Left; //intake motor
    CRServo intake_Right; //intake motor

    //Intake State
    public static boolean Intake_Active = false; //if intake is active or not (off by default)

    //Button toggles to prevent internal "button spam"
    boolean X_Pressed = false;

    public AccessoryControl(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor"); //Motor defined as "armMotor" in driver hub
        intake_Left = hardwareMap.get(CRServo.class, "intake_Left"); //CRservo defined as "intake" in driver hub
        //intake_Right = hardwareMap.get(CRServo.class, "intake_Right"); //CRservo defined as "intake" in driver hub
        viperSlide = hardwareMap.get(DcMotor.class,"viperSlide"); //Motor defined as "viperSlide" in driver hub
        //wrist = hardwareMap.get(Servo.class, "wrist");

        //Define Direction behavior (makes it easier to code movement as you do not have to manually account for it in your code)\\;
        viperSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //Define behavior of motors with no input\\
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Fixes issue where the arm is unresponsive for a period of time until setpoint reaches resting point\\
        Arm_Pos = armMotor.getCurrentPosition()+50;
    }
    public void RunAccessory(Gamepad gamepad1) {
        //Arm movement Pt. 1 angle setpoint\\
        if (gamepad1.left_trigger > 0){Arm_Pos -= Arm_increment;}
        if (gamepad1.right_trigger > 0){Arm_Pos += Arm_increment;}
        if (gamepad1.right_bumper) {Arm_ToPos(850,5000); intake_Left.setPower(0);}
        if (gamepad1.left_bumper) {Arm_ToPos(110,1130); intake_Left.setPower(-1);}

        //Arm Increment Control (Mainly to quick tune the arm movement)\\
        if (gamepad1.dpad_up) {Viper_Pos-=Viper_Increment;}
        if (gamepad1.dpad_down) {Viper_Pos+=Viper_Increment;}

        //Intake\\
        if (gamepad1.x && (!X_Pressed)) {
            X_Pressed = true;
            //if intake is inactive, activate intake\\
            if (!Intake_Active) {
                Intake_Active = true;
                intake_Left.setPower(1);
            }
            //if intake is active, deactivate intake\\
            else {
                intake_Left.setPower(-0.5);
                Intake_Active = false;
            }
        }
        if (!gamepad1.x && X_Pressed) {X_Pressed = false;}
        if (gamepad1.a) {intake_Left.setPower(0);}
    }

    public void Run_Motors() {
        //Arm movement pt.2 Moving to setpoint\\
        if (Arm_Pos > Max_Arm) {Arm_Pos = Max_Arm;}
        if (Arm_Pos < Min_Arm) {Arm_Pos = Min_Arm;}
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setTargetPosition(Arm_Pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (Viper_Pos > Viper_maxExtend) {Viper_Pos = Viper_maxExtend;}
        if (Viper_Pos < Viper_minExtend) {Viper_Pos = Viper_minExtend;}
        ((DcMotorEx)viperSlide).setVelocity(2100);
        viperSlide.setTargetPosition(Viper_Pos);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Arm_ToPos(int Arm, int Viperslide) {
        Viper_Pos = Viperslide;
        while (viperSlide.isBusy() && viperslide_Timeout < 1500) {
            viperslide_Timeout++;
        }
        Arm_Pos = Arm;
    }
}
