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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//STEVEN'S CODE (written 11/15/19) goodfaithday
//NEVER EVER DELETE WITHOUT ADITYA'S PERMISSION
//OR LIKE NOT AT ALL, THIS IS GOD'S CODE

public class OverflowHardware
{

    //steve odo eyoov
    double xPosition;
    double yPosition;
    double theta;
    double previousTheta;
    double thetaDelta;

    double tankXDelta;
    double tankYDelta;

    double previousXPosition;
    double previousYPosition;

    double updateMagnitudeVector;
    double updateMagnitudeAngle;

    double tankRadius;
    double strafeRadius;
    double robotcentricXDelta;
    double robotcentricYDelta;

    double translationalVectorMagnitude;
    double robotcentricTranslationalVectorAngle;
    double fieldcentricTranslationalVectorAngle;

    /* Public OpMode members. */
    public DcMotor fL   = null;
    public DcMotor fR  = null;
    public DcMotor rL  = null;
    public DcMotor rR  = null;
    public DcMotor intakeLeft = null;
    public DcMotor intakeRight = null;

    public Servo grabberLeft = null;
    public Servo grabberRight = null;
    public Servo intakeLock = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public OverflowHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        fL  = hwMap.get(DcMotor.class, "fL"); //verticalLeft
        fR = hwMap.get(DcMotor.class, "fR"); //verticalRight
        rL  = hwMap.get(DcMotor.class, "rL"); //strafing
        rR = hwMap.get(DcMotor.class, "rR");
        intakeLeft = hwMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hwMap.get(DcMotor.class, "intakeRight");

        // Define and Initialize Servos
        grabberLeft = hwMap.get(Servo.class, "grabberLeft");
        grabberRight = hwMap.get(Servo.class, "grabberRight");
        intakeLock = hwMap.get(Servo.class, "intakeLock");

        fL.setDirection(DcMotor.Direction.REVERSE);
        rL.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        fL.setPower(0);
        fR.setPower(0);
        rL.setPower(0);
        rR.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void updateOdometry() {
        theta = TelemetryCalculator.theta(getLeftVertical(), getRightVertical());
        if (theta == 0) { //going exactly straight at the time of update
            robotcentricYDelta = getRightVertical(); //choose one
            robotcentricXDelta = getStrafe();
        }
        thetaDelta = TelemetryCalculator.thetaDelta(theta, previousTheta);

        tankRadius = TelemetryCalculator.tankRadius(theta, getLeftVertical(), getRightVertical());
        strafeRadius = TelemetryCalculator.strafeRadius(theta, thetaDelta, getStrafe());
        robotcentricXDelta = TelemetryCalculator.calculateXDelta(tankRadius, strafeRadius, theta);
        robotcentricYDelta = TelemetryCalculator.calculateYDelta(tankRadius, strafeRadius, theta);

        tankXDelta = TelemetryCalculator.tankXDelta(tankRadius, theta);
        tankYDelta = TelemetryCalculator.tankYDelta(tankRadius, theta);

        translationalVectorMagnitude = Math.hypot(robotcentricXDelta, robotcentricYDelta); //magnitude of the vector
        robotcentricTranslationalVectorAngle = Math.atan2(robotcentricXDelta, robotcentricYDelta); //angle of the vector relative to the robot
        fieldcentricTranslationalVectorAngle = robotcentricTranslationalVectorAngle + theta; //fieldcentric version

        xPosition = previousXPosition + (translationalVectorMagnitude * Math.cos(fieldcentricTranslationalVectorAngle));
        yPosition = previousYPosition + (translationalVectorMagnitude * Math.sin(fieldcentricTranslationalVectorAngle));

        previousXPosition = xPosition;
        previousYPosition = yPosition;
        previousTheta = theta;
    }

    public double getxPosition() {
        return xPosition;
    }

    public double getyPosition() {
        return yPosition;
    }

    public void setPosition(double xPosition, double yPosition) {
        this.xPosition = xPosition;
        this.yPosition = yPosition;
    }

    public void resetMotorEncoders() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithoutEncoders() {
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getRightVertical() {
        return fR.getCurrentPosition(); //3
    }

    public int getLeftVertical() {
        return fL.getCurrentPosition(); //2
    }

    public double getVerticalAverage() {
        return ((double)(fL.getCurrentPosition() + fR.getCurrentPosition()))/(2);
    }

    public int getStrafe() {
        return rL.getCurrentPosition(); //1
    }

}

