package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class OpenCV_Robot {
    public DcMotor FRDrive;
    public DcMotor FLDrive;
    public DcMotor BRDrive;
    public DcMotor BLDrive;

    public DcMotor Duck;
    public DcMotor Intake;
    public DcMotor PivotR;
    public DcMotor PivotL;
    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        FRDrive = hwMap.get(DcMotor.class, "FRDrive");
        FLDrive = hwMap.get(DcMotor.class, "FLDrive");
        BRDrive = hwMap.get(DcMotor.class, "BRDrive");
        BLDrive = hwMap.get(DcMotor.class, "BLDrive");

        Duck = hwMap.get(DcMotor.class, "Duck");
        Intake = hwMap.get(DcMotor.class, "Intake");
        PivotR = hwMap.get(DcMotor.class, "PivotR");
        PivotL = hwMap.get(DcMotor.class, "PivotL");

        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);

        Intake.setDirection(DcMotor.Direction.FORWARD);
        PivotL.setDirection(DcMotor.Direction.FORWARD);
        Duck.setDirection(DcMotor.Direction.FORWARD);
        PivotR.setDirection(DcMotor.Direction.FORWARD);

        FRDrive.setPower(0);
        FLDrive.setPower(0);
        BRDrive.setPower(0);
        BLDrive.setPower(0);

        Intake.setPower(0);
        Duck.setPower(0);
        PivotR.setPower(0);
        PivotL.setPower(0);

        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PivotL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PivotR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PivotR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PivotL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void DriveMecanum(double strafe, double drive, double turn, boolean modify) {

        double modifier = 0.90;
        if (modify) {
            modifier = 0.5;
        }

        double FRPower = -strafe + drive - turn;
        double FLPower = strafe + drive + turn;
        double BRPower = strafe + drive - turn;
        double BLPower = -strafe + drive + turn;

        double maxPower = Math.max(FLPower, Math.max(FRPower, Math.max(BLPower, BRPower)));

        if (maxPower > 1) {
            FRPower = FRPower / maxPower;
            FLPower = FLPower / maxPower;
            BRPower = BRPower / maxPower;
            BLPower = BLPower / maxPower;
        }

        FRDrive.setPower(FRPower * modifier);
        FLDrive.setPower(FLPower * modifier);
        BRDrive.setPower(BRPower * modifier);
        BLDrive.setPower(BLPower * modifier);

    }

    public void Pivot(boolean up, boolean down) {

        if (up && !down) {
            PivotR.setPower(0.6);
            PivotL.setPower(0.6);
        } else if (down && !up){
            PivotR.setPower(-0.6);
            PivotL.setPower(-0.6);
        } else {
            PivotR.setPower(0);
            PivotL.setPower(0);
        }
    }

    public void Intake(boolean in, boolean out) {
        if (in && !out) {
            Intake.setPower(1);
        }else if (out && !in){
            Intake.setPower(-0.3);
        } else {
            Intake.setPower(0);
        }
    }

    double SPINMAX = 1; //Maximum speed of spin
    double SPINMIN = 0.55; //Minimum speed of spin
    double spinVal = SPINMIN; //Current speed of spin
    double spinIncrease = 0.05; //Speed increase every 1/50 of a second
    public void DuckSpinner(boolean in, boolean out) {
        if (in) {
            Duck.setPower(spinVal);
            spinVal = Math.min(SPINMAX, spinVal + spinIncrease);
        } else if(out){
            Duck.setPower(-1 * spinVal);
            spinVal = Math.min(SPINMAX, spinVal + spinIncrease);
        } else {
            Duck.setPower(0);
            spinVal = SPINMIN;
        }
    }
}
