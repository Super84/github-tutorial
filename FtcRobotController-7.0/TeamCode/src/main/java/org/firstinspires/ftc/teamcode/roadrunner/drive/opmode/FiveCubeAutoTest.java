package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class FiveCubeAutoTest extends LinearOpMode {
    DcMotor intake, PivotR, PivotL;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "Intake");
        PivotR = hardwareMap.get(DcMotor.class, "PivotR");
        PivotL = hardwareMap.get(DcMotor.class, "PivotL");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PivotL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PivotR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PivotL.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        PivotR.setDirection(DcMotor.Direction.FORWARD);

        intake.setPower(0);
        PivotR.setPower(0);
        PivotL.setPower(0);

        PivotR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PivotL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startPos = new Pose2d(-63, 0,0);
        Pose2d dropPos = new Pose2d(17,0,Math.toRadians(179.9));
        Pose2d cubes[] = {  new Pose2d(42,47,0),
                            new Pose2d(42,23.5,0),
                            new Pose2d(42,0,0),
                            new Pose2d(42,-25.5,0),
                            new Pose2d(42,-47,0)};

        waitForStart();

        drive.setPoseEstimate(startPos);

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(startPos)
                .splineToSplineHeading(cubes[0],0)
                .addDisplacementMarker(72, () -> {
                    intake.setPower(1);
                })
                .build();

        drive.followTrajectory(traj);
        intake.setPower(0);
        raiseArm();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end(),true)
                .splineToSplineHeading(dropPos,Math.toRadians(179.9))
                .build();

        drive.followTrajectory(traj2);

        intake.setPower(-0.3);
        sleep(2000);
        intake.setPower(0);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .lineTo(new Vector2d(dropPos.getX() + 6, dropPos.getY())).build();
        drive.followTrajectory(traj3);

        lowerArm();
        drive.turn(Math.toRadians(179.9));

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0,0, Math.toRadians(179.9))))
                .lineToLinearHeading(cubes[1].plus(new Pose2d(-6,0,0)))
                .build();

        drive.followTrajectory(traj4);
        intake.setPower(1);

        Trajectory traj4a = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(cubes[1])
                .build();

        drive.followTrajectory(traj4a);
        intake.setPower(0);
        raiseArm();

        Trajectory traj5 = drive.trajectoryBuilder(traj4a.end(),true)
                .splineToSplineHeading(dropPos,Math.toRadians(179.9))
                .build();

        drive.followTrajectory(traj5);

        intake.setPower(-0.3);
        sleep(2000);
        intake.setPower(0);

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), true)
                .lineTo(new Vector2d(dropPos.getX() + 6, dropPos.getY())).build();
        drive.followTrajectory(traj6);

        lowerArm();
        drive.turn(Math.toRadians(179.9));

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end().plus(new Pose2d(0,0, Math.toRadians(179.9))))
                .lineToLinearHeading(cubes[2])
                .addDisplacementMarker(10, () -> {
                    intake.setPower(1);
                })
                .build();

        drive.followTrajectory(traj7);
        intake.setPower(0);
        raiseArm();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end(),true)
                .lineToLinearHeading(dropPos)
                .build();

        drive.followTrajectory(traj8);

        intake.setPower(-0.3);
        sleep(2000);
        intake.setPower(0);

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(), true)
                .lineTo(new Vector2d(dropPos.getX() + 6, dropPos.getY())).build();
        drive.followTrajectory(traj9);

        lowerArm();
        drive.turn(Math.toRadians(179.9));

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end().plus(new Pose2d(0,0, Math.toRadians(179.9))))
                .lineToLinearHeading(cubes[3].plus(new Pose2d(-10,0,0)))
                .build();

        drive.followTrajectory(traj10);
        intake.setPower(1);

        Trajectory traj10a = drive.trajectoryBuilder(traj10.end())
                .lineToLinearHeading(cubes[3])
                .build();

        drive.followTrajectory(traj10a);
        intake.setPower(0);
        raiseArm();

        Trajectory traj11 = drive.trajectoryBuilder(traj10a.end(),true)
                .splineToSplineHeading(dropPos,Math.toRadians(179.9))
                .build();

        drive.followTrajectory(traj11);

        intake.setPower(-0.3);
        sleep(2000);
        intake.setPower(0);

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end(), true)
                .lineTo(new Vector2d(dropPos.getX() + 6, dropPos.getY())).build();
        drive.followTrajectory(traj12);

        lowerArm();

        Trajectory traj13 = drive.trajectoryBuilder(traj12.end(), true)
                .splineToSplineHeading(cubes[4],0)
                .addDisplacementMarker(20, () -> {
                    intake.setPower(1);
                })
                .build();

        drive.followTrajectory(traj13);
        intake.setPower(0);
        raiseArm();

        Trajectory traj14 = drive.trajectoryBuilder(traj13.end(),true)
                .splineToSplineHeading(dropPos,Math.toRadians(179.9))
                .build();

        drive.followTrajectory(traj14);

        intake.setPower(-0.3);
        sleep(2000);
        intake.setPower(0);

        Trajectory traj15 = drive.trajectoryBuilder(traj14.end(), true)
                .lineTo(new Vector2d(dropPos.getX() + 10, dropPos.getY())).build();
        drive.followTrajectory(traj15);

        lowerArm();
    }

    void raiseArm() {
        do {
            PivotR.setPower(0.6);
            PivotL.setPower(0.6);
        } while(PivotR.getCurrentPosition() < 1300);
        PivotR.setPower(0);
        PivotL.setPower(0);
    }

    void lowerArm() {
        do {
            PivotR.setPower(-0.6);
            PivotL.setPower(-0.6);
        } while(PivotR.getCurrentPosition() > 50);
        PivotR.setPower(0);
        PivotL.setPower(0);
    }
}
