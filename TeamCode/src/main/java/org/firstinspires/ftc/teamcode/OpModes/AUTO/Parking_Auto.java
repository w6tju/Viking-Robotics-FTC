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

package org.firstinspires.ftc.teamcode.OpModes.AUTO;


import static org.firstinspires.ftc.teamcode.RobotCfg.Arm_Pos;
import static org.firstinspires.ftc.teamcode.RobotCfg.Viper_Pos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AccessoryControl;
import org.firstinspires.ftc.teamcode.RobotCfg;

import java.util.concurrent.CompletableFuture;

@Autonomous()
public class Parking_Auto extends LinearOpMode {

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor rearLeftDrive;
    DcMotor rearRightDrive;
    Servo wrist;

    //Hybrid Telemetry
     Telemetry Telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() {
        AccessoryControl accessoryControl = new AccessoryControl(hardwareMap);
        //MecanumDrive Controller = new MecanumDrive(hardwareMap,new Pose2d(60,60,90));
        //PathConstructor pathConstructor = new PathConstructor(Controller,hardwareMap);


        //Setup Drive motors\\
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
        wrist = hardwareMap.get(Servo.class,"wrist");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        CompletableFuture.runAsync(() -> {
            while (opModeIsActive()) {
                accessoryControl.Run_Motors();
            }
        });
        accessoryControl.atStart();
        Arm_Pos = 160;
        Viper_Pos = 0;
        ElapsedTime Move_Timer = new ElapsedTime();
        wrist.setPosition(0.5);
        while (Move_Timer.seconds() < 1) {
            Mecanum_Movement(-1,0,0,false,false);
        }
        Mecanum_Movement(0,0,0,false,false);
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
