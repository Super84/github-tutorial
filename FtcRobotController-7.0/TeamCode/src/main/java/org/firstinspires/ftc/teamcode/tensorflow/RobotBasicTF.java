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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class RobotBasicTF
{
    /* Public OpMode members. */
    public DcMotor FRDrive = null;
    public DcMotor FLDrive = null;
    public DcMotor BRDrive = null;
    public DcMotor BLDrive = null;

    public DcMotor  PivotR = null;
    public DcMotor  PivotL = null;
    public DcMotor  Duck = null;

    public DcMotor  Intake = null;



    public ColorSensor colorSensor1 = null;
    public ColorSensor colorSensor2 = null;

    public Servo Arm = null;
    public Servo Element = null;


    //Change values after testing the color sensor.
    public int DuckRH = 750;
    public int DuckRL = 98;
    public int DuckGH = 1180;
    public int DuckGL = 158;
    public int DuckBH = 340;
    public int DuckBL = 79;
    public int DuckAlphaH = 760;
    public int DuckAlphaL = 110;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private static final String[] LABELS = {
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AWdllWT/////AAABmaEnazLbyEZTrfTLsn9f/95jTGkKh9q4NE3yVBhEPkNhxmq0Mkz7VTdaheVfbbDmJ3i62+WrLPHdSc5x4l59KVR+o8KzbNO0MnETP7mkznZrZhQe1uxZxAIWGAHgW2WfwwoXYa2fPNIykd3Z1Y+WYLGyCcSR63k8cn8uWx//+dq1wJ6nUSB4gizva37pPzZrIVFk5lLcX4uZhi7Zpnskinj6lcI2TsN+tVvoLtcm3o1iBiEQHGGhwSi7AKRSOMJUSB84PVGQxR29cmH0sNa5kQ2JpH0Hrp0MEIQ5XJDQOe/1V97u+LtDhGDI9JdV+ACypblAibVVzrnxlbIsMZ/LuU6nYxOwpeFMIUi2IlWlvf27";
    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    /* Constructor */
    public RobotBasicTF(){

    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        initVuforia(hwMap);
        initTfod(hwMap);

        // Define and Initialize Motors
        FRDrive = hwMap.get(DcMotor.class, "FRDrive");
        FLDrive = hwMap.get(DcMotor.class, "FLDrive");
        BRDrive = hwMap.get(DcMotor.class, "BRDrive");
        BLDrive = hwMap.get(DcMotor.class, "BLDrive");
        Intake = hwMap.get(DcMotor.class, "Intake");
        PivotR = hwMap.get(DcMotor.class,"PivotR");
        PivotL = hwMap.get(DcMotor.class,"PivotL");
        Duck = hwMap.get(DcMotor.class, "Duck");


        colorSensor1 = hwMap.get(ColorSensor.class, "colorSensor1");
        colorSensor2 = hwMap.get(ColorSensor.class, "colorSensor2");

        Arm = hwMap.get(Servo.class, "Arm");
        Element= hwMap.get(Servo.class, "Element");

        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);

        Intake.setDirection(DcMotor.Direction.FORWARD);
        PivotL.setDirection(DcMotor.Direction.FORWARD);
        Duck.setDirection(DcMotor.Direction.FORWARD);
        PivotR.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        FRDrive.setPower(0);
        FLDrive.setPower(0);
        BRDrive.setPower(0);
        BLDrive.setPower(0);

        Intake.setPower(0);
        Duck.setPower(0);
        PivotR.setPower(0);
        PivotL.setPower(0);


        // Set all motors to run without encoders.
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PivotL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PivotR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PivotR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PivotL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    public void DriveMecanum(double strafe, double drive, double turn, boolean modify) {

        //Calculate needed power.
        double modifier = 0.85;
        if (modify) {
            modifier = 0.5;
        } else {
            modifier = 0.85;
        }

        double FRPower = -strafe + drive - turn;
        double FLPower = strafe + drive + turn;
        double BRPower = strafe + drive - turn;
        double BLPower = -strafe + drive + turn;

        //Ensure that power does not go over 1.
        double maxPower = Math.max(FLPower, Math.max(FRPower, Math.max(BLPower, BRPower)));

        if (maxPower > 1) {

            FRPower = FRPower / maxPower;
            FLPower = FLPower / maxPower;
            BRPower = BRPower / maxPower;
            BLPower = BLPower / maxPower;

        }

        //Apply the power to the wheels.
        FRDrive.setPower(FRPower * modifier);
        FLDrive.setPower(FLPower * modifier);
        BRDrive.setPower(BRPower * modifier);
        BLDrive.setPower(BLPower * modifier);

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

    public void DuckSpinner(boolean in, boolean out) {

        if (in) {
            Duck.setPower(-0.65);


        } else if(out){
            Duck.setPower(0.65);

        }else {
            Duck.setPower(0);

        }
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

    public void Arm(boolean up, boolean down, boolean drop) {
        if (up && !down && !drop) {
            Arm.setPosition(0.1);

        }
        if (down  && !up && !drop) {
            Arm.setPosition(0.45);

        }
        if (drop && !up &&! down){
            Arm.setPosition(0.33);
        }
    }

    public void Element(boolean open, boolean close) {
        if (open && !close) {
            Element .setPosition(0.46);

        }
        if (close && !open) {
            Element.setPosition(0.84);

        }
    }


    public boolean duckPresent(ColorSensor colorSensor) {
        boolean isPresent;
        if (colorSensor.red() < DuckRH && colorSensor.red() > DuckRL) {
            if (colorSensor.green() < DuckGH || colorSensor.green() > DuckGL) {
                if (colorSensor.blue() < DuckBH || colorSensor.blue() > DuckBL) {
                    if (colorSensor.alpha() < DuckAlphaH && colorSensor.alpha() > DuckAlphaL) {
                        isPresent = true;
                    } else {
                        isPresent = false;
                    }
                } else {
                    isPresent = false;
                }
            } else {
                isPresent = false;
            }
        } else {
            isPresent = false;
        }

        return isPresent;
    }

    //Initialize Vuforia to use the camera for Vision
    private void initVuforia(HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    //Initialize TensorFlow Object Detection
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    //We call getUpdatedRecognitions() to update our list of detected objects. Because the call
    //sometimes returns null if the list was not updated, we should return getRecognitions()
    //to always get the list.
    public List<Recognition> getUpdatedTFList() {
        tfod.getUpdatedRecognitions();
        return tfod.getRecognitions();
    }

    //Returns instance of a found duck with the highest confidence, if there is one
    public Recognition getDuck(List<Recognition> list) {
        Recognition ret = null;
        float confidence = 0.0f;
        for(Recognition recognition: list) {
            if(recognition.getLabel().equals("Duck") && recognition.getConfidence() > confidence) {
                ret = recognition;
            }
        }
        return ret;
    }

    //Returns location of the duck.
    public String getLocationOfDuck(Recognition recognition) {
        if(recognition == null) {
            return "Right";
        } else if(recognition.getLeft() > 300) {
            return "Center";
        } else {
            return "Left";
        }
    }
}

