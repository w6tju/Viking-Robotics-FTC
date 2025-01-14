package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotCfg.Arm_Pos;
import static org.firstinspires.ftc.teamcode.RobotCfg.Arm_increment;
import static org.firstinspires.ftc.teamcode.RobotCfg.Max_Arm;
import static org.firstinspires.ftc.teamcode.RobotCfg.Min_Arm;
import static org.firstinspires.ftc.teamcode.RobotCfg.Presets;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_Increment;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_Pos;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_maxExtend;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_minExtend;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.CompletableFuture;

public class AccessoryControl {
    //Enum Classes (Arm Modes)
    public enum Mode {
        SPECEMIN,
        SAMPLE
    }

    //Physical Objects
    DcMotor armMotor; //motor in arm tower
    DcMotor viperSlide; //Motor that runs viperslide
    Servo wrist; //wrist servo
    CRServo intake_Left; //intake motor
    CRServo intake_Right; //intake motor

    //Intake State
    public static boolean Intake_Active = false; //if intake is active or not (off by default)
    public static boolean Sweep_Mode = false; //if sweep mode is active or not

    //Mode State
    public static Mode Arm_Mode = Mode.SAMPLE;

    //Button toggles to prevent internal "button spam"
    boolean X_Pressed = false;
    boolean Y_Pressed = false;
    boolean B_Pressed = false;
    boolean Mode_Pressed = false;

    public AccessoryControl(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor"); //Motor defined as "armMotor" in driver hub
        intake_Left = hardwareMap.get(CRServo.class, "intake_Left"); //CRservo defined as "intake_Left" in driver hub
        intake_Right = hardwareMap.get(CRServo.class, "intake_Right"); //CRservo defined as "intake_Right" in driver hub
        viperSlide = hardwareMap.get(DcMotor.class,"viperSlide"); //Motor defined as "viperSlide" in driver hub
        wrist = hardwareMap.get(Servo.class, "wrist"); //Servo defined as "wrist" in driver hub

        //Define Direction behavior (makes it easier to code movement as you do not have to manually account for it in your code)\\
        wrist.setDirection(Servo.Direction.REVERSE);
        viperSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //Define behavior of motors with no input\\
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Fixes issue where the arm is unresponsive for a period of time until setpoint reaches resting point\\
        Arm_Pos = 50;
        Viper_Pos = 0;
    }

    public void atStart() {
        wrist.setPosition(0.05);
    }
    public void RunAccessory(Gamepad gamepad1) {
        //Arm movement Pt. 1 angle setpoint\\
        if (gamepad1.left_trigger > 0){Arm_Pos -= Arm_increment;}
        if (gamepad1.right_trigger > 0){Arm_Pos += Arm_increment;}

        if (Arm_Mode == Mode.SAMPLE) {
            if (gamepad1.right_bumper) {Arm_ToPos(Presets.Basket_High.Arm, Presets.Basket_High.Viperslide);}
            //if (gamepad1.left_bumper) {Arm_ToPos(Presets.Intake_Sample.Arm, Presets.Intake_Sample.Viperslide); Set_intake(1);}
        } else if (Arm_Mode == Mode.SPECEMIN) {
            if (gamepad1.right_bumper) {Arm_ToPos(Presets.Specemin_High.Arm, Presets.Specemin_High.Viperslide);}
            //if (gamepad1.left_bumper) {Arm_ToPos(Presets.Intake_Specemin.Arm, Presets.Intake_Specemin.Viperslide); Set_intake(1);}
        }

        //Arm Increment Control (Mainly to quick tune the arm movement)\\
        if (gamepad1.dpad_up) {Viper_Pos+=Viper_Increment;}
        if (gamepad1.dpad_down) {Viper_Pos-=Viper_Increment;}

        //Intake\\
        if (gamepad1.x && (!X_Pressed)) {
            X_Pressed = true;
            //if intake is inactive, activate intake\
            Intake_Active = true;
            Set_intake(-0.5);
        }
        if (!gamepad1.x && X_Pressed) {X_Pressed = false;}

        if (gamepad1.y && (!Y_Pressed)) {
            Y_Pressed = true;
            Intake_Active = false;
            Set_intake(1);
        }
        if (!gamepad1.y && Y_Pressed) {Y_Pressed = false;}

        if (gamepad1.b && (!B_Pressed)) {
            B_Pressed = true;
            //if intake is inactive, activate intake\\
            if (!Sweep_Mode) {
                Sweep_Mode = true;
                wrist.setPosition(.05);
            }
            //if intake is active, deactivate intake\\
            else {
                wrist.setPosition(0.5);
                Sweep_Mode = false;
            }
        }
        if (!gamepad1.b && B_Pressed) {B_Pressed = false;}
        if (gamepad1.guide && (!Mode_Pressed)) {
            Mode_Pressed = true;
            //if intake is inactive, activate intake\\
            if (Arm_Mode != Mode.SPECEMIN) {
                Arm_Mode = Mode.SPECEMIN;
            }
            //if intake is active, deactivate intake\\
            else {
                Arm_Mode = Mode.SAMPLE;
            }
        }
        if (!gamepad1.guide && Mode_Pressed) {Mode_Pressed = false;}

        Run_Motors();
    }

    public void Run_Motors() {
        //Arm movement pt.2 Moving to setpoint\\
        int Arm_Encoder = armMotor.getCurrentPosition();
        if (Arm_Pos > Max_Arm) {Arm_Pos = Max_Arm;}
        if (Arm_Pos < Min_Arm) {Arm_Pos = Min_Arm;}
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setTargetPosition(Arm_Pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int Viper_Encoder = viperSlide.getCurrentPosition();
        if (Arm_Pos >= 1000) {
            if (Viper_Pos > Viper_maxExtend) {
                Viper_Pos = Viper_maxExtend;
            }
        } else {
            if (Viper_Pos > Viper_maxExtend-1000) {
                Viper_Pos = Viper_maxExtend-1000;
            }
        }
        if (Viper_Pos < Viper_minExtend) {Viper_Pos = Viper_minExtend;}
        if (Viper_Encoder >= Viper_Pos-5 && Viper_Encoder <= Viper_Pos+5) {
            viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            viperSlide.setPower(0);
        } else {
            ((DcMotorEx) viperSlide).setVelocity(2100);
            viperSlide.setTargetPosition(Viper_Pos);
            viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void Arm_ToPos(int Arm, int Viperslide) {
        CompletableFuture.runAsync(() -> {
            boolean Wait_ForViper = Viper_Pos > Viperslide;
            Viper_Pos = Viperslide;
            if (Wait_ForViper) {
                ElapsedTime Viper_Timeout = new ElapsedTime();
                Viper_Timeout.reset();
                while (viperSlide.getCurrentPosition() <= Viper_Pos+5 && viperSlide.getCurrentPosition() >= Viper_Pos-5 || Viper_Timeout.seconds() < 3) {
                    Run_Motors();
                }
            }
            Arm_Pos = Arm;
        });
    }

    public void Set_intake(double Intake_speed) {
        CompletableFuture.runAsync(() -> {
            intake_Left.setPower(Intake_speed);
            intake_Right.setPower(-Intake_speed);

            if (Intake_speed != 0) {
                ElapsedTime Intake_Timeout = new ElapsedTime();
                Intake_Timeout.reset();
                while (Intake_Timeout.seconds() <  0.3) {
                    Intake_Timeout.log("Intake Running for: ");
                }
                intake_Left.setPower(0);
                intake_Right.setPower(0);
            }
        });
    }
}

