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
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Teleop with Tensorflow", group="Iterative Opmode")

public class TeleopWithTensorFlow extends OpMode
{

    RobotWithTensorFlow robot = new RobotWithTensorFlow();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double RPower;
    double LPower;
    private ElapsedTime visionTimer = new ElapsedTime();
    private String duckLocation = "";
    private float duckConfidence = -1.0f;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        robot.LDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.LDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.tfod.activate();
        robot.tfod.setZoom(1.25, 16.0/9.0);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        robot.LDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        robot.LDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
        // Setup a variable for each drive wheel to save power level for telemetry
        double LPower;
        double RPower;


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        robot.Pivot(gamepad2.dpad_up,gamepad2.dpad_down);
        robot.DuckSpinner(gamepad2.a );
        robot.Intake(gamepad2.right_trigger > 0, gamepad2.left_trigger > 0);
        robot.GameElement(gamepad2.x, gamepad2.y);

        LPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        RPower   = Range.clip(drive - turn, -1.0, 1.0) ;


        if(gamepad1.a) {
            duckConfidence = -1.0f;
        }
        //Vision stuff. Run once every second
        if(visionTimer.milliseconds() >= 1000) {
            Recognition duck = robot.getDuck(robot.getUpdatedTFList());
            if(duckConfidence == -1.0f) {
                duckLocation = robot.getLocationOfDuck(duck);
                if(duck == null) {
                    duckConfidence = 0.0f;
                } else {
                    duckConfidence = duck.getConfidence();
                }
            } else if (duck != null && duck.getConfidence() > duckConfidence) {
                duckLocation = robot.getLocationOfDuck(duck);
                duckConfidence = duck.getConfidence();
            }
            visionTimer.reset();
        }



        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        robot.LDrive.setPower(LPower);
        robot.RDrive.setPower(RPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Position of Duck: ", duckLocation);
        telemetry.addData("Confidence of Duck in this location: ", duckConfidence);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.tfod.deactivate();
    }

}
