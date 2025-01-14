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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AccessoryControl;

@Disabled
@TeleOp()
public class LinearTeleOp extends LinearOpMode {

    //Hybrid Telemetry
    Telemetry Telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    // Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDrive; //left side motor
    DcMotor rightDrive; //right side motor

    @Override
    public void runOpMode() {
        AccessoryControl accessoryController = new AccessoryControl(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Set motors\\
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive"); //Motor defined as "left_drive" in driver hub
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive"); //Motor defined as "right_drive" in driver hub

        //Define Direction behavior (makes it easier to code movement as you do not have to manually account for it in your code)\\;
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Define behavior of motors with no input\\
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //waits for start button on driver hub\\
        waitForStart();
        runtime.reset();
        //Main loop (runs as long as you are in op mode)\\
        while (opModeIsActive()) {
            //define power variables for drive motors\\
            double leftPower;
            double rightPower;

            //Differential (tank) steering code\\
            double drive = -gamepad1.left_stick_y*WHEEL_SPEED;
            double turn  =  gamepad1.right_stick_x*WHEEL_SPEED;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            //Send power (maxRPM_AtV*Power) to motor\\
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            accessoryController.RunAccessory(gamepad2);
            accessoryController.Run_Motors();

            //Telemetry (shows up on driver hub)\\
            Telemetry.addData("Status", "Run Time: " + runtime.toString());
            Telemetry.addData("Motors:", "left (%.2f), right (%.2f)", leftPower, rightPower);
            Telemetry.addData("Arm:",Arm_Pos);
            Telemetry.addData("ViperSlide:",Viper_Pos);
            if (!Intake_Active) {
                Telemetry.addData("Intake:", "Intaking");
            }
            else {
                Telemetry.addData("Intake:","Ejecting");
            }
            Telemetry.update();
        }
    }
}
