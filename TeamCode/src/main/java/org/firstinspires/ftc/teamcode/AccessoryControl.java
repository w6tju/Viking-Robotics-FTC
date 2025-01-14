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
import org.firstinspires.ftc.teamcode.Preset;

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
        //region Hardware
        armMotor = hardwareMap.get(DcMotor.class, "armMotor"); //Motor defined as "armMotor" in driver hub
        intake_Left = hardwareMap.get(CRServo.class, "intake_Left"); //CRservo defined as "intake_Left" in driver hub
        intake_Right = hardwareMap.get(CRServo.class, "intake_Right"); //CRservo defined as "intake_Right" in driver hub
        viperSlide = hardwareMap.get(DcMotor.class,"viperSlide"); //Motor defined as "viperSlide" in driver hub
        wrist = hardwareMap.get(Servo.class, "wrist"); //Servo defined as "wrist" in driver hub
        //endregion

        //region Directions
        wrist.setDirection(Servo.Direction.REVERSE);
        viperSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //endregion

        //region Zero power
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //endregion

        //region Encoder
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //endregion

        //region Inital positions
        Arm_Pos = Min_Arm;
        Viper_Pos = 0;
        //endregion
    }

    public void atStart() {
        wrist.setPosition(.05);
    }
    public void RunAccessory(Gamepad gamepad1) {
        //region Arm position
        if (gamepad1.left_trigger > 0){Arm_Pos -= Arm_increment;}
        if (gamepad1.right_trigger > 0){Arm_Pos += Arm_increment;}
        //endregion

        //region Viperslide
        if (gamepad1.dpad_up) {Viper_Pos+=Viper_Increment;}
        if (gamepad1.dpad_down) {Viper_Pos-=Viper_Increment;}
        //endregion

        //region Presets
        if (Arm_Mode == Mode.SAMPLE) {
            if (gamepad1.right_bumper) {Arm_ToPos(Presets.Basket_High);}
            //if (gamepad1.left_bumper) {Arm_ToPos(Presets.Intake_Sample); Set_intake(1);}
        } else if (Arm_Mode == Mode.SPECEMIN) {
            if (gamepad1.right_bumper) {Arm_ToPos(Presets.Specemin_High);}
            //if (gamepad1.left_bumper) {Arm_ToPos(Presets.Intake_Specemin); Set_intake(1);}
        }
        //endregion

        //region Intake
        if (gamepad1.x && (!X_Pressed)) {
            X_Pressed = true;
            Intake_Active = true;
            Set_intake(1);
        }
        if (!gamepad1.x && X_Pressed) {X_Pressed = false;}

        if (gamepad1.y && (!Y_Pressed)) {
            Y_Pressed = true;
            Intake_Active = false;
            Set_intake(-0.5);
        }
        if (!gamepad1.y && Y_Pressed) {Y_Pressed = false;}
        //endregion

        //region Sweep
        if (gamepad1.b && (!B_Pressed)) {
            B_Pressed = true;
            if (!Sweep_Mode) {
                Sweep_Mode = true;
                wrist.setPosition(.05);
            }
            else {
                wrist.setPosition(0.5);
                Sweep_Mode = false;
            }
        }
        if (!gamepad1.b && B_Pressed) {B_Pressed = false;}
        //endregion

        //region Mode Switching
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
        //endregion

        Run_Motors(); // we call Run_Motors here so that you dont have to in the op-mode loop
    }

    public void Run_Motors() {
        //Handles motor running, useful for AUTO when there is no controller input\\

        //region Arm
        int Arm_Encoder = armMotor.getCurrentPosition();
        if (Arm_Pos > Max_Arm) {Arm_Pos = Max_Arm;}
        if (Arm_Pos < Min_Arm) {Arm_Pos = Min_Arm;}
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setTargetPosition(Arm_Pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //endregion

        //region Viperslide
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
        // Viperslide "deadzone" that prevents motor burnout \\
        if (Viper_Encoder >= Viper_Pos-5 && Viper_Encoder <= Viper_Pos+5) {
            viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            viperSlide.setPower(0);
        } else {
            ((DcMotorEx) viperSlide).setVelocity(2100);
            viperSlide.setTargetPosition(Viper_Pos);
            viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //endregion
    }

    public void Arm_ToPos(Preset PresetTo) {
        CompletableFuture.runAsync(() -> {
            boolean Wait_ForViper = Viper_Pos > PresetTo.Viperslide;
            Viper_Pos = PresetTo.Viperslide;
            if (Wait_ForViper) {
                ElapsedTime Viper_Timeout = new ElapsedTime();
                Viper_Timeout.reset();
                while (viperSlide.getCurrentPosition() <= Viper_Pos+5 && viperSlide.getCurrentPosition() >= Viper_Pos-5 || Viper_Timeout.seconds() < 3) {
                    Run_Motors();
                }
            }
            Arm_Pos = PresetTo.Arm;
        });
    }

    public void Set_intake(double Intake_speed) {
        CompletableFuture.runAsync(() -> {
            intake_Left.setPower(-Intake_speed);
            intake_Right.setPower(Intake_speed);

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

