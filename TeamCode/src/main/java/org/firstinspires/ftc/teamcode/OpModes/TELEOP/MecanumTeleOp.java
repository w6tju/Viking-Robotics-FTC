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

package org.firstinspires.ftc.teamcode.OpModes.TELEOP;

import static org.firstinspires.ftc.teamcode.AccessoryControl.Intake_Active;
import static org.firstinspires.ftc.teamcode.RobotCfg.Arm_Pos;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_Pos;
import static org.firstinspires.ftc.teamcode.RobotCfg.WHEEL_SPEED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AccessoryControl;


@TeleOp()
public class MecanumTeleOp extends LinearOpMode {
    //Hybrid Telemetry
    Telemetry Telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    //Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor rearLeftDrive;
    DcMotor rearRightDrive;

    @Override
    public void runOpMode() {
        AccessoryControl accessoryController = new AccessoryControl(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        //Setup Drive motors\\
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //waits for start button on driver hub\\
        waitForStart();
        accessoryController.atStart();
        runtime.reset();
        //Main loop (runs as long as you are in op mode)\\
        while (opModeIsActive()) {
            WHEEL_SPEED = Range.clip(WHEEL_SPEED,0,1);

            double Drive = -gamepad1.left_stick_x * WHEEL_SPEED;
            double Drift = -gamepad1.left_stick_y * WHEEL_SPEED;
            double Turn = gamepad1.right_stick_x * WHEEL_SPEED;

            Mecanum_Movement(Drive,Drift,Turn,gamepad1.dpad_left,gamepad1.dpad_right);
            accessoryController.RunAccessory(gamepad2);

            //Telemetry (shows up on driver hub and FTCDashboard)\\
            Telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Wrist",wrist.getPosition());
            Telemetry.addData("Motors: ",
                    "Front left (%.2f), Front right (%.2f), Rear left (%.2f), Rear right (%.2f)",
                    frontLeftDrive.getPower(),frontRightDrive.getPower(),rearLeftDrive.getPower(),rearRightDrive.getPower());
            Telemetry.addData("Arm: ",Arm_Pos);
            Telemetry.addData("ViperSlide: ",Viper_Pos);
            if (!Intake_Active) {
                Telemetry.addData("Intake: ", "Intaking");
            }
            else {
                Telemetry.addData("Intake: ","Ejecting");
            }
            Telemetry.addData("Arm_Mode:", AccessoryControl.Arm_Mode);
            Telemetry.update();
        }
    }
    public void Mecanum_Movement(double Travel,double Strafe,double Rotate, boolean Axial_Rotation, boolean Concerning) {
        //A Drives
        double FL = 0;
        double RR = 0;

        //B Drives
        double FR = 0;
        double RL = 0;

        if (!Axial_Rotation && !Concerning) {
            FL = (Strafe - Travel - Rotate);
            FR = (Strafe + Travel + Rotate);
            RL = (Strafe + Travel - Rotate);
            RR = (Strafe - Travel + Rotate);
        } else {
            if (Axial_Rotation && !Concerning) {
                FL = (Strafe - Travel - Rotate);
                FR = (Strafe + Travel + Rotate);
                RL = (Strafe + Travel);
                RR = (Strafe - Travel);
            }
            if (Concerning && !Axial_Rotation) {
                if (Rotate > 0) {
                    FL = (Strafe - Travel - Rotate);
                    RL = (Strafe + Travel - Rotate);
                    FR = (Strafe + Travel);
                    RR = (Strafe - Travel);
                } else {
                    FL = (Strafe - Travel);
                    RL = (Strafe + Travel);
                    FR = (Strafe + Travel + Rotate);
                    RR = (Strafe - Travel + Rotate);
                }
            }
        }

        FL = Range.clip(FL,-1,1);
        FR = Range.clip(FR,-1,1);
        RL = Range.clip(RL,-1,1);
        RR = Range.clip(RR,-1,1);

        frontLeftDrive.setPower(FL);
        frontRightDrive.setPower(FR);
        rearLeftDrive.setPower(RL);
        rearRightDrive.setPower(RR);
    }
}