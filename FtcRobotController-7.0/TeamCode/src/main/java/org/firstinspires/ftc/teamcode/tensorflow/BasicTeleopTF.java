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

package org.firstinspires.ftc.teamcode.tensorflow;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic Tensorflow Teleop", group="Iterative Opmode")

public class BasicTeleopTF extends OpMode
{


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotBasicTF robotBasicTF = new RobotBasicTF();
    private ElapsedTime visionTimer = new ElapsedTime();
    private String duckLocation = "";
    private float duckConfidence = -1.0f;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robotBasicTF.init(hardwareMap);

        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        robotBasicTF.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBasicTF.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBasicTF.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBasicTF.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBasicTF.PivotR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robotBasicTF.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotBasicTF.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotBasicTF.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotBasicTF.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotBasicTF.PivotR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotBasicTF.FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotBasicTF.FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotBasicTF.BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotBasicTF.BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotBasicTF.PivotR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotBasicTF.PivotL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robotBasicTF.Arm.setPosition(0.1);

        robotBasicTF.tfod.activate();
        robotBasicTF.tfod.setZoom(1.0, 16.0/9.0);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {




        robotBasicTF.DriveMecanum(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger > 0);
        robotBasicTF.Pivot(gamepad2.dpad_up,gamepad2.dpad_down);
        robotBasicTF.DuckSpinner(gamepad2.x, gamepad2.b);
        robotBasicTF.Intake(gamepad2.right_trigger > 0, gamepad2.left_trigger > 0);
        robotBasicTF.Arm(gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.b);

        robotBasicTF.Element(gamepad2.right_bumper, gamepad2.left_bumper);


        telemetry.addData("FRDrive", robotBasicTF.FRDrive.getCurrentPosition());
        telemetry.addData("FLDrive", robotBasicTF.FLDrive.getCurrentPosition());
        telemetry.addData("BRDrive", robotBasicTF.BRDrive.getCurrentPosition());
        telemetry.addData("BLDrive", robotBasicTF.BLDrive.getCurrentPosition());
        telemetry.addData("PivotR", robotBasicTF.PivotR.getCurrentPosition());


        telemetry.addData("Red Left", robotBasicTF.colorSensor1.red());
        telemetry.addData("Blue Left", robotBasicTF.colorSensor1.blue());
        telemetry.addData("Green Left", robotBasicTF.colorSensor1.green());
        telemetry.addData("Alpha Left", robotBasicTF.colorSensor1.alpha());

        telemetry.addData("Red Middle", robotBasicTF.colorSensor2.red());
        telemetry.addData("Blue Middle", robotBasicTF.colorSensor2.blue());
        telemetry.addData("Green Middle", robotBasicTF.colorSensor2.green());
        telemetry.addData("Alpha Middle", robotBasicTF.colorSensor2.alpha());

        telemetry.addData("Is marker or duck in front of left sensor:" , robotBasicTF.duckPresent(robotBasicTF.colorSensor1));
        telemetry.addData("Is marker or duck in front of middle sensor:" , robotBasicTF.duckPresent(robotBasicTF.colorSensor2));

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        if(gamepad1.a) {
            duckConfidence = -1.0f;
        }
        //Vision stuff. Run once every second
        if(visionTimer.milliseconds() >= 1000) {
            Recognition duck = robotBasicTF.getDuck(robotBasicTF.getUpdatedTFList());
            if(duckConfidence == -1.0f) {
                duckLocation = robotBasicTF.getLocationOfDuck(duck);
                if(duck == null) {
                    duckConfidence = 0.0f;
                } else {
                    duckConfidence = duck.getConfidence();
                }
            } else if (duck != null && duck.getConfidence() > duckConfidence) {
                duckLocation = robotBasicTF.getLocationOfDuck(duck);
                duckConfidence = duck.getConfidence();
            }
            visionTimer.reset();
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Position of Duck: ", duckLocation);
        telemetry.addData("Confidence of Duck in this location: ", duckConfidence);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
