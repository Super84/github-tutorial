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

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RobotWithTensorFlow
{
    /* Public OpMode members. */
    public DcMotor  LDrive   = null;
    public DcMotor  RDrive  = null;
    public DcMotor  Pivot = null;
    public DcMotor  DuckRed = null;
    public DcMotor  DuckBlue = null;
    public DcMotor  Intake = null;

    public ColorSensor colorSensor1 = null;
    public ColorSensor colorSensor2 = null;

    public Servo GameElement = null;

    //Change values after testing the color sensor.
    public int DuckRH = 230;
    public int DuckRL = 170;
    public int DuckGH = 345;
    public int DuckGL = 280;
    public int DuckBH = 130;
    public int DuckBL = 100;
    public int DuckAlphaH = 250;
    public int DuckAlphaL = 180;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    //Vision variables
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
    public RobotWithTensorFlow(){

    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        initVuforia(hwMap);
        initTfod(hwMap);

        // Define and Initialize Motors
        LDrive  = hwMap.get(DcMotor.class, "LDrive");
        RDrive = hwMap.get(DcMotor.class, "RDrive");
        Intake = hwMap.get(DcMotor.class, "Intake");
        Pivot = hwMap.get(DcMotor.class,"Pivot");
        DuckRed = hwMap.get(DcMotor.class, "DuckRed");
        DuckBlue = hwMap.get(DcMotor.class, "DuckBlue");

        colorSensor1 = hwMap.get(ColorSensor.class, "colorSensor1");
        colorSensor2 = hwMap.get(ColorSensor.class, "colorSensor2");

        GameElement = hwMap.get(Servo.class, "GameElement");

        LDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        Intake.setDirection(DcMotor.Direction.FORWARD);



        DuckRed.setDirection(DcMotor.Direction.FORWARD);
        DuckBlue.setDirection(DcMotor.Direction.REVERSE);
        Pivot.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        LDrive.setPower(0);
        RDrive.setPower(0);


        Pivot.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Intake(boolean in, boolean out) {

        if (in && !out) {
            Intake.setPower(1);


        }else if (out && !in){
            Intake.setPower(-0.7);



        } else {
            Intake.setPower(0);

        }
    }

    public void DuckSpinner(boolean in) {

        if (in) {
            DuckRed.setPower(0.50);
            DuckBlue.setPower(0.50);
        } else {
            DuckRed.setPower(0);
            DuckBlue.setPower(0);
        }
    }



    public void Pivot(boolean up, boolean down) {

        if (up && !down) {
            Pivot.setPower(0.4);

        } else if (down && !up){
            Pivot.setPower(-0.4);

        } else {
            Pivot.setPower(0);
        }
    }

    public void GameElement(boolean open, boolean close) {
        if (open && !close) {
            GameElement.setPosition(0.6);

        }
        if (close && !open) {
            GameElement.setPosition(1);

        }
    }
    public boolean ringPresent(ColorSensor colorSensor) {
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

    //A bit clearer ringPresent function. Uses the withinRange helper function to determine
    //if the sensor reading is greater than the low value and less than the high value.
    //Because withinRange() returns true or false, you can use boolean logic (OR ||, AND &&, NOT !)
    //on the value to return another true or false value
    public boolean ringPresent2(ColorSensor colorSensor) {
        return withinRange(colorSensor.red(), DuckRL, DuckRH)
                && withinRange(colorSensor.green(), DuckGL, DuckGH)
                && withinRange(colorSensor.blue(), DuckBL, DuckBH)
                && withinRange(colorSensor.alpha(), DuckAlphaL, DuckAlphaH);
    }

    //Helper function. Returns true if value is greater than low and less than high, false otherwise
    private boolean withinRange(int value, int low, int high) {
        return value > low && value < high;
    }

    //This is equivalent to the code above
//  private boolean withinRange(int value, int low, int high) {
//      if(value > low && value < high) {
//          return true;
//      } else {
//          return false;
//      }
//  }

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

