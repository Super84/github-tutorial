package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Block Finder with OpenCV", group="Iterative Opmode")

public class Block_Finder_OpenCV extends OpMode {
    OpenCV_Robot robot = new OpenCV_Robot();

    //Camera object for OpenCV
    OpenCvCamera camera;
    //Resolutions to try: 1280/720, 640/480, 320/240. 640/480 gives best fps for resolution
    int cameraWidth = 640;
    int cameraHeight = 480;

    //5% tolerance for the block finder used below
    double blockTolerance = 0.05;

    //Set up a pipeline for OpenCV to use
    BlockPipeline pipeline = new BlockPipeline();

    //Determines if the camera has been properly set up
    boolean visionActive = false;
    ElapsedTime blockTime = new ElapsedTime();

    //Block Finder state enum to control the state machine below
    //A state machine allows us to control code in unique ways
    //This state machine allows us to control the robot differently depending on which state we are in
    //This is especially important here because the robot needs to raise the arm for the camera to see the field
    //First we have a default waiting state for when the robot is not trying to find a block
    //Once we want to find a block, then we must raise the arm
    //Once the arm is raised then we position the robot to be able to pick up the block
    //We then lower the arm to be able to grab the block
    //We move forward to pick up the block
    //Then we have a complete state that allows us to return to manual driving mode
    //Implementation of this state machine is in the moveToBlock() method
    enum BlockFinderState {
        WAITING,
        RAISE_ARM,
        POSITION_BLOCK,
        LOWER_ARM,
        MOVE_FORWARD,
        COMPLETE
    }

    //We set the current state to waiting because we are not trying to find the block yet
    BlockFinderState currentState = BlockFinderState.WAITING;

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



        //We initialize the camera monitor view to see the camera from the drivers station
        int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());

        //We get the external camera name from the hardware map
        WebcamName webcamName = robot.hwMap.get(WebcamName.class, "Webcam 1");

        //We set up the camera with the camera name and the camera monitor view id
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //We enable optimization for the drivers station view
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        //We set the pipeline that OpenCV will use on the camera images
        camera.setPipeline(pipeline);

        //This sets up the camera but does not block the thread (the code will continue past this line
        //while setting up the camera so keep that in mind)
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            //This is called when the camera is successfully opened. We put the startStreaming
            //method in here because we want to do that after the camera has opened
            @Override
            public void onOpened()
            {
                //Start streaming images from the camera with the given resolution
                camera.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);

                //Let our code know that the camera is available
                visionActive = true;
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    @Override
    public void init_loop() {
        //Wait for the camera to be available before getting information from the pipeline
        if(visionActive) {

            //The centerOfBox() method returns the middle of the largest block that OpenCV finds
            //Details of this implementation can be found in the pipeline class
            Point blobPoint = pipeline.centerOfBox();

            //If there is a point returned, then send the telemetry update
            if(blobPoint != null) {
                //This gives the center position and various pipeline camera statistics
                telemetry.addData("CenterPos", "x: %s y: %s", blobPoint.x, blobPoint.y);
                telemetry.addData("CV Cycle Time", camera.getTotalFrameTimeMs());
                telemetry.addData("CV Pipeline Time", camera.getPipelineTimeMs());
                telemetry.addData("CV Cycles", camera.getFrameCount());
            }
        } else {
            //Send waiting telemetry if the camera is not active
            telemetry.addData("Waiting", "Waiting");
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        //if gamepad1 a is not pressed then we drive the robot manually. Details on this are in Blue_2021_Teleop
        if(!gamepad1.a) {
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

            //If the button is not pressed, then we want to reset the state machine back to waiting
            currentState = BlockFinderState.WAITING;
        } else {
            //If gamepad1 a is pressed, then we want to run through the state machine in moveToBlock
            moveToBlock();
        }
        if(visionActive) {
            //If the camera is active then we give info on the camera pipeline stats
            telemetry.addData("CV Cycle Time", camera.getTotalFrameTimeMs());
            telemetry.addData("CV Pipeline Time", camera.getPipelineTimeMs());
            telemetry.addData("CV Cycles", camera.getFrameCount());
            telemetry.addData("Current State", currentState);
        }
    }

    @Override
    public void stop() {
        //Once we are done then we stop the camera streaming and close it out
        camera.stopStreaming();
        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });
    }

    //Needs to aim for position x=685, y=640 on a 1280/720 camera resolution
    //This is the implementation of our state machine as declared above
    //Each case in the switch/case statement has two main functions, action(s) and a condition to advance the state machine
    //As an overview, this state machine will raise the arm, position the robot to the closest block,
    //lower the arm, move forward, then stop
    public void moveToBlock() {
        switch (currentState) {
            //The waiting case first advances the state machine then sets everything to 0 to stop any mechanisms currently going
            case WAITING:
                currentState = BlockFinderState.RAISE_ARM;
                robot.DriveMecanum(0, 0, 0, false);
                robot.Pivot(false,false);
                robot.Intake(false,false);
                robot.DuckSpinner(false,false);
                break;
            //The raise_arm case will raise the arm to a given position, then move to the position_block state
            case RAISE_ARM:
                if(robot.PivotR.getCurrentPosition() < 1000) {
                    robot.Pivot(true,false);
                } else {
                    robot.Pivot(false,false);
                    currentState = BlockFinderState.POSITION_BLOCK;
                }
                break;
            //This moves the robot to position the biggest block to x=680 y=640 on a 1280/720 resolution screen
            case POSITION_BLOCK:
                //We get the center of the biggest block on the screen
                Point blockCenter = pipeline.centerOfBox();

                //We set the power of both strafe and forward to 0 to start
                double ypow = 0;
                double xpow = 0;

                //This checks whether the robot needs to move left or right. The current position is a percentage of the current x
                //to the total camera width, and the ideal is the percentage of 680/1280. The block tolerance is how far
                //we are okay with being off by. 5% seems to work well here
                if(!withinTolerance(blockCenter.x/cameraWidth, 680f/1280f, blockTolerance)) {
                    //Either move left or right depending on which side of the block the ideal is
                    if(blockCenter.x/cameraWidth > 680f/1280f) {
                        xpow = 0.3;
                    } else {
                        xpow = -0.3;
                    }
                }

                //This checks whether the robot needs to move forward or back. The current position is a percentage of the current y
                //to the total camera height, and the ideal is the percentage of 640/720. The block tolerance is how far
                //we are okay with being off by. 5% seems to work well here
                if(!withinTolerance(blockCenter.y/cameraHeight, 640f/720f, blockTolerance)) {
                    //Either move forward or backward depending on which side of the block the ideal is
                    if(blockCenter.y/cameraHeight > 640f/720f) {
                        ypow = -0.3;
                    } else {
                        ypow = 0.3;
                    }
                }
                //We then set the x and y power to the values defined above
                robot.DriveMecanum(xpow, ypow, 0, false);
                //If both of these values are 0 (meaning the block is within the tolerance of our ideal for both x and y) then we move on
                if(xpow == 0 && ypow == 0) {
                    currentState = BlockFinderState.LOWER_ARM;
                }
                break;
            //This lowers the arm to an ideal position to be able to pick up a block
            case LOWER_ARM:
                if(robot.PivotR.getCurrentPosition() > 50) {
                    robot.Pivot(false,true);
                } else {
                    robot.Pivot(false,false);
                    currentState = BlockFinderState.MOVE_FORWARD;
                    //We reset the timer for the next state to use
                    blockTime.reset();
                }
                break;
            //Here we move forward and run the intake for 3 seconds to pick up the block, then stop the bot and move to complete
            case MOVE_FORWARD:
                if(blockTime.milliseconds() < 3000) {
                    robot.DriveMecanum(0,0.3,0,false);
                    robot.Intake(true, false);
                } else {
                    robot.DriveMecanum(0,0,0,false);
                    robot.Intake(false, false);
                    currentState = BlockFinderState.COMPLETE;
                }
                break;
            //If the state is complete or not found, then we do nothing
            default:
                break;
        }
    }

    //Helper function to reduce code bloat, returns true if the value is between high and low
    public boolean between(int value, int high, int low) {
        return value <= high && value >= low;
    }

    //Helper function to reduce code bloat, returns true if the value is between high and low
    public boolean between(double value, double high, double low) {
        return value <= high && value >= low;
    }

    //Helper function to reduce code bloat, returns true if the value is between the ideal+tolerance and ideal-tolerance
    public boolean withinTolerance(double value, double ideal, double tolerance) { return between(value, ideal+tolerance, ideal-tolerance);}

    //Helper function to reduce code bloat, returns true if the value is between the ideal+tolerance and ideal-tolerance
    public boolean withinTolerance(int value, int ideal, int tolerance) { return between(value, ideal+tolerance, ideal-tolerance);}
}
