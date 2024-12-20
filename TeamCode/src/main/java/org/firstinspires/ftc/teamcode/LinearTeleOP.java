/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import static org.firstinspires.ftc.teamcode.TeleOP_Cfg.Arm_increment;
import static org.firstinspires.ftc.teamcode.TeleOP_Cfg.Viper_Increment;
import static org.firstinspires.ftc.teamcode.TeleOP_Cfg.WHEEL_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp()

public class LinearTeleOP extends LinearOpMode {

    //Hybrid Telemetry
    //Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    // Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDrive; //left side motor
    DcMotor rightDrive; //right side motor
    DcMotor armMotor; //motor in arm tower
    DcMotor viperSlide; //Motor that runs viperslide
    Servo wrist;
    CRServo intake_Left; //intake motor
    CRServo intake_Right; //intake motor
    int Arm_Pos = 0; //arm setpoint

    private boolean Intake_Active = false; //if intake is active or not (off by default)

    //Viper Slide configs
    int Viper_maxExtend = 5300; //Maximum encoder tick on the viperslide extension
    int Viper_minExtend = 0; //Minimum encoder tick on the viperslide extension

    int Viper_Pos = 0; //Current encoder tick position of the viperslide

    //Button toggles to prevent internal "button spam"
    boolean Dpad_UpPressed = false;
    boolean Dpad_DownPressed = false;
    boolean X_Pressed = false;
    boolean B_Pressed = false;
    //Auto hang toggle\\
    boolean Hang_Mode = false;

    //Variables to handle Viperslide Limiting
    final double Counts_PerViperRev = 537.7; //Counts of the encoder per revolution of the motor
    final double Distance_PerViperRev = 4.72441; //Length Added by Viperslide per revolution in inches
    final double RobotLength = 18.5; //Robot length in inches
    final double Arm_Length = 0; //Length of arm fully retracted in inches

    //Arm Limiting
    final int Min_Arm = 0;
    final int Max_Arm = 900;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Set motors\\
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive"); //Motor defined as "left_drive" in driver hub
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive"); //Motor defined as "right_drive" in driver hub
        armMotor = hardwareMap.get(DcMotor.class, "armMotor"); //Motor defined as "armMotor" in driver hub
        intake_Left = hardwareMap.get(CRServo.class, "intake_Left"); //CRservo defined as "intake" in driver hub
        //intake_Right = hardwareMap.get(CRServo.class, "intake_Right"); //CRservo defined as "intake" in driver hub
        viperSlide = hardwareMap.get(DcMotor.class,"viperSlide"); //Motor defined as "viperSlide" in driver hub
        //wrist = hardwareMap.get(Servo.class, "wrist");

        //Define Direction behavior (makes it easier to code movement as you do not have to manually account for it in your code)\\;
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        viperSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //Define behavior of motors with no input\\
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Fixes issue where the arm is unresponsive for a period of time until setpoint reaches resting point\\
        Arm_Pos = armMotor.getCurrentPosition()+50;


        //waits for start button on driver hub\\
        waitForStart();
        runtime.reset();
        //Main loop (runs as long as you are in op mode)\\
        while (opModeIsActive()) {
            //define power variables for drive motors\\
            double leftPower;
            double rightPower;

            //Differential (tank) steering code\\
            double drive;
            if (Hang_Mode) {drive = 1; } else {drive = -gamepad1.left_stick_y*WHEEL_SPEED; }
            double turn  =  gamepad1.right_stick_x*WHEEL_SPEED;


            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            //Send power (maxRPM_AtV*Power) to motor\\
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //Arm movement Pt. 1 angle setpoint\\
            if (gamepad1.left_trigger > 0){Arm_Pos -= Arm_increment;}
            if (gamepad1.right_trigger > 0){Arm_Pos += Arm_increment;}
            if (gamepad1.right_bumper) {Arm_Pos = 850; Viper_Pos = 5000; intake_Left.setPower(0);}
            if (gamepad1.left_bumper) {Arm_Pos = 130; Viper_Pos = 1130; intake_Left.setPower(-1);}

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

            //Automatic Hanging
            /*if (gamepad1.b && !B_Pressed) {
                B_Pressed = true;
                Hang_Mode = !Hang_Mode;
            }
            if (!gamepad1.b && B_Pressed) {B_Pressed = false;}*/

            //Arm movement pt.2 Moving to setpoint\\
            if (Arm_Pos > Max_Arm) {Arm_Pos = Max_Arm;}
            if (Arm_Pos < Min_Arm) {Arm_Pos = Min_Arm;}

            if (drive != 0 || turn != 0) {
                if (Arm_Pos < 200) {
                    ((DcMotorEx) armMotor).setVelocity(2100);
                    armMotor.setTargetPosition(200);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            else {
                ((DcMotorEx) armMotor).setVelocity(2100);
                armMotor.setTargetPosition(Arm_Pos);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (Viper_Pos > Viper_maxExtend) {Viper_Pos = Viper_maxExtend;}
            if (Viper_Pos < Viper_minExtend) {Viper_Pos = Viper_minExtend;}
            ((DcMotorEx)viperSlide).setVelocity(2100);
            viperSlide.setTargetPosition(Viper_Pos);
            viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Telemetry (shows up on driver hub)\\
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Wrist",wrist.getPosition());
            telemetry.addData("ArmIncrement", Arm_increment);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Arm",Arm_Pos);
            telemetry.addData("ViperSlide",Viper_Pos);
            if (!Intake_Active) {
               telemetry.addData("Intake", "Intaking");
            }
            else {
              telemetry.addData("Intake","Ejecting");
            }
            telemetry.update();
        }
    }
}