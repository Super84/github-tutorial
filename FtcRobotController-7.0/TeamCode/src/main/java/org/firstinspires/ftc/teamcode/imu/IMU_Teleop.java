package org.firstinspires.ftc.teamcode.imu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Teleop with IMU", group="Iterative Opmode")

public class IMU_Teleop extends OpMode {
    IMU_Robot robot = new IMU_Robot();

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void init_loop() {
        //Wait for the gyro to be calibrated in order to grab the angles properly
        if(robot.imu.isGyroCalibrated()) {
            //Store the three angles in an Orientation object
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            //Display the data via telemetry. Heading may be x, y, or z depending how the hub is mounted
            telemetry.addData("Angles", "x: %.2f y: %.2f z: %.2f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
        } else {
            //Display a waiting message while we wait for the gyro to be calibrated
            telemetry.addData("Waiting", "Waiting");
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        //Standard manual drive, see Blue_2021_Teleop for more details
        robot.DriveMecanum(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger > 0);
        if(gamepad1.dpad_up && robot.PivotR.getCurrentPosition() < 1000) {
            robot.Pivot(true,false);
        } else if(gamepad1.dpad_down && robot.PivotR.getCurrentPosition() > 50) {
            robot.Pivot(false,true);
        } else {
            robot.Pivot(gamepad2.dpad_up,gamepad2.dpad_down);
        }
        robot.Intake(gamepad2.right_trigger > 0 || gamepad1.right_trigger > 0, gamepad2.left_trigger > 0 || gamepad1.left_trigger > 0);
        robot.DuckSpinner(gamepad2.x, gamepad2.b);

        //Wait for the gyro to be calibrated in order to grab the angles properly
        if(robot.imu.isGyroCalibrated()) {
            //Store the three angles in an Orientation object
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            //Display the data via telemetry. Heading may be x, y, or z depending how the hub is mounted
            telemetry.addData("Angles", "x: %.2f y: %.2f z: %.2f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
        } else {
            //Display a waiting message while we wait for the gyro to be calibrated
            telemetry.addData("Waiting", "Waiting");
        }
    }

    @Override
    public void stop() {
    }

}
