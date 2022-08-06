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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Blue_2021_Robot
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
    public DigitalChannel redLED;
    public DigitalChannel greenLED;

    public ColorSensor colorSensor1 = null;
    public ColorSensor colorSensor2 = null;
    public ColorSensor colorSensor3 = null;


    //Change values after testing the color sensor.
    public int DuckRH = 2100;
    public int DuckRL = 80;
    public int DuckGH = 3300;
    public int DuckGL = 85;
    public int DuckBH = 890;
    public int DuckBL = 57;
    public int DuckAlphaH = 2100;
    public int DuckAlphaL = 85;

    public int BallRH = 1850;
    public int BallRL = 300;
    public int BallGH = 3200;
    public int BallGL = 540;
    public int BallBH = 2700;
    public int BallBL = 440;
    public int BallAlphaH = 2600;
    public int BallAlphaL = 430;

    //Color values stored in an array for ease of use. This is the order:
    //RH, RL, GH, GL, BH, BL, AlphaH, AlphaL
    public int[] ballCV = {1850, 300, 3200, 540, 2700, 440, 2600, 430};
    public int[] blockCV = {2200, 650, 2800, 800, 740, 240, 1900, 570};


    /* local OpMode members. */
        HardwareMap hwMap           =  null;
        private ElapsedTime period  = new ElapsedTime();

        /* Constructor */
    public Blue_2021_Robot(){

    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

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
        colorSensor3 = hwMap.get(ColorSensor.class, "colorSensor3");

        redLED = hwMap.get(DigitalChannel.class, "red");
        greenLED = hwMap.get(DigitalChannel.class, "green");

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
        Duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);


    }
    public void DriveMecanum(double strafe, double drive, double turn, boolean modify) {

        //Calculate needed power.
        double modifier = 0.90;
        if (modify) {
            modifier = 0.5;
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

    //We want to have a cooldown to slow down the intake after the color sensor stops
    //seeing the ball so it does not speed back up immediately and shoot the ball
    int ballCooldown = 0;
    public void Intake(boolean in, boolean out) {
        if(isBallOrBlockPresent(colorSensor3) == 1) {
            ballCooldown = 0;
        } else {
            ballCooldown++;
        }
        boolean slow = true;
        if(ballCooldown > 10) {
            slow = false;
        }

        if (in && !out) {
            Intake.setPower(1);


        }else if (out && !in){
            if(slow) {
                Intake.setPower(-0.2);
            } else {
                Intake.setPower(-0.3);
            }


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

    public boolean ballPresent(ColorSensor colorSensor) {
        boolean isPresent;
        if (colorSensor.red() < BallRH && colorSensor.red() > BallRL) {
            if (colorSensor.green() <BallGH || colorSensor.green() > BallGL) {
                if (colorSensor.blue() < BallBH || colorSensor.blue() > BallBL) {
                    if (colorSensor.alpha() < BallAlphaH && colorSensor.alpha() > BallAlphaL) {
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

    //Returns true if value is between high and low, false otherwise
    public boolean between(int value, int high, int low) {
        return value <= high && value >= low;
    }

    //Takes a color sensor and outputs 1 for a ball, 2 for a block, and 0 for nothing.
    public int isBallOrBlockPresent(ColorSensor colorSensor) {
        if(between(colorSensor.red(), ballCV[0], ballCV[1]) &&
                between(colorSensor.green(), ballCV[2], ballCV[3]) &&
                between(colorSensor.blue(), ballCV[4], ballCV[5]) &&
                between(colorSensor.alpha(), ballCV[6], ballCV[7])) {
            return 1;
        }
        if(between(colorSensor.red(), blockCV[0], blockCV[1]) &&
                between(colorSensor.green(), blockCV[2], blockCV[3]) &&
                between(colorSensor.blue(), blockCV[4], blockCV[5]) &&
                between(colorSensor.alpha(), blockCV[6], blockCV[7])) {
            return 2;
        }
        return 0;
    }

    public void UpdateLight() {
        int temp = isBallOrBlockPresent(colorSensor3);
        if(temp == 1) {
            greenLED.setState(false);
            redLED.setState(true);
            //Turn on light for ball
        } else if(temp == 2) {
            greenLED.setState(true);
            redLED.setState(false);
            //Turn on light for block
        } else {
            //Turn off light
            greenLED.setState(false);
            redLED.setState(false);
        }
    }

}

