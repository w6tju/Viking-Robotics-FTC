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


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous()
public class RoadRunnerAuto extends LinearOpMode {
    DcMotor armMotor; //motor in arm tower
    DcMotor viperSlide; //Motor that runs viperslide
    CRServo intake; //intake motor
    int Viper_Pos = 0; //Current encoder tick position of the viperslide
    int Arm_Pos = 0; //arm setpoint
    SampleTankDrive Controller = new SampleTankDrive(hardwareMap);

    @Override
    public void runOpMode() {
        //import needed components
        armMotor = hardwareMap.get(DcMotor.class, "armMotor"); //Motor defined as "armMotor" in driver hub
        intake = hardwareMap.get(CRServo.class, "intake"); //CRservo defined as "intake" in driver hub
        viperSlide = hardwareMap.get(DcMotor.class,"viperSlide"); //Motor defined as "viperSlide" in driver hub

        TrajectorySequence Move1 = Controller.trajectorySequenceBuilder(new Pose2d())
                .forward(26)
                .waitSeconds(0)//TBD
                //Inject Specimen Release
                .addDisplacementMarker(() -> {
                    Arm_Pos = 0; //High Rung Point
                    intake.setPower(-1);
                    Arm_Pos = 0; //HalfwayPoint
                })
                .back(10)
                //Collect & Deposit Sample 1
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .addDisplacementMarker(() -> { //Collect Sample
                    Arm_Pos = 0; //CollectionPoint
                    intake.setPower(1);
                    Viper_Pos = 0; //Viper Extension
                })
                .lineToLinearHeading(new Pose2d(0,0,0))
                .addDisplacementMarker(()-> { //Deposit Sample
                    Arm_Pos = 0; //ReleasePoint
                    intake.setPower(1);
                    Viper_Pos = 0; //Viper Extension
                })
                //Collect & Deposit Sample 2
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .addDisplacementMarker(() -> { //Collect Sample
                    Arm_Pos = 0; //CollectionPoint
                    intake.setPower(1);
                    Viper_Pos = 0; //Viper Extension
                })
                .lineToLinearHeading(new Pose2d(0,0,0))
                .addDisplacementMarker(()-> { //Deposit Sample
                    Arm_Pos = 0; //ReleasePoint
                    intake.setPower(1);
                    Viper_Pos = 0; //Viper Extension
                })
                //Collect & Deposit Sample 3
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .addDisplacementMarker(() -> { //Collect Sample
                    Arm_Pos = 0; //CollectionPoint
                    intake.setPower(1);
                    Viper_Pos = 0; //Viper Extension
                })
                .lineToLinearHeading(new Pose2d(0,0,0))
                .addDisplacementMarker(()-> { //Deposit Sample
                    Arm_Pos = 0; //ReleasePoint
                    intake.setPower(1);
                    Viper_Pos = 0; //Viper Extension
                })

                //Park in Observation Zone
                .lineTo(new Vector2d(0,0))
                .build();

        waitForStart();
        Controller.followTrajectorySequenceAsync(Move1);
        while (opModeIsActive()) {
            Controller.update();

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setTargetPosition(Arm_Pos);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx)viperSlide).setVelocity(2100);
            viperSlide.setTargetPosition(Viper_Pos);
            viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
