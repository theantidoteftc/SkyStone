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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="encoderbasedauto", group="encoderbasedauto")
public class encoderbasedauto extends LinearOpMode {

    /* Declare OpMode members. */
    OverflowHardware robot = new OverflowHardware();
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.addData("Vertical Average", robot.getVerticalAverage());
        telemetry.addData("Strafe", robot.getStrafe());
        telemetry.update();


        robot.grabberLeft.setPosition(0);
        robot.grabberRight.setPosition(1);

        robot.intakeLock.setPosition(1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.intakeLock.setPosition(0);

        sleep(1000);

        telemetryDrive(0.25,4500,5);
        sleep(150);
        telemetryTurn(0.25,-1250,5);
        sleep(150);
        robot.intakeLeft.setPower(1);
        robot.intakeRight.setPower(1);
        telemetryDrive(0.25,2500,5);
        sleep(200);
        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);
        telemetryDrive(0.35,-2000,5);
        sleep(2000);
        telemetryTurn(0.2,-2100,5);
        sleep(150);
        telemetryDrive(0.65,-13000,5);
        sleep(250);
        telemetryTurn(0.3,-2200,5);
        telemetryDrive(0.2,-900,3);
        sleep(125);
        robot.grabberLeft.setPosition(1);
        robot.grabberRight.setPosition(0);
        sleep(500);
        telemetryDrive(0.4,3000,5);
        telemetryTurn(0.4,3575,8);
        telemetryDrive(0.4,-2000,2);
        robot.grabberLeft.setPosition(0);
        robot.grabberRight.setPosition(1);
        sleep(500);
        telemetryStrafe(0.3,800,4);
        telemetryTurn(0.2,-200,3);
        telemetryDrive(0.25,11200,5);
        telemetryTurn(0.25,1100,3);
        robot.intakeLeft.setPower(1);
        robot.intakeRight.setPower(1);
        telemetryDrive(0.25,500,5);
        sleep(200);
        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);


        telemetry.addData("Vertical Average", robot.getVerticalAverage());
        telemetry.addData("Strafe", robot.getStrafe());
        telemetry.update();
        sleep(5000);
        //telemetryDrive(0.4,2000,2);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void telemetryTurn(double speed, int encoderTicks, double timeoutS) {
        int newLeftEncoderTarget;
        int newRightEncoderTarget;

        if (opModeIsActive()) {
            if (encoderTicks > 0) {
                newLeftEncoderTarget = (int)(robot.getLeftVertical()) + encoderTicks;
                newRightEncoderTarget = (int)(robot.getRightVertical()) - encoderTicks;

                // reset the timeout time and start motion.
                runtime.reset();
                robot.fR.setPower(-speed); //-1
                robot.fL.setPower(speed); //1
                robot.rR.setPower(-speed); //-1
                robot.rL.setPower(speed); //1

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((int)robot.getLeftVertical() < newLeftEncoderTarget) && ((int)robot.getRightVertical() > newRightEncoderTarget)) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d (left), %7d", newLeftEncoderTarget, newRightEncoderTarget);
                    telemetry.addData("Vertical Average", robot.getStrafe());
                    telemetry.update();
                }
            }
            if (encoderTicks < 0) {
                newLeftEncoderTarget = (int)(robot.getLeftVertical()) + encoderTicks;
                newRightEncoderTarget = (int)(robot.getRightVertical()) - encoderTicks;

                // reset the timeout time and start motion.
                runtime.reset();
                robot.fR.setPower(speed); //-1
                robot.fL.setPower(-speed); //1
                robot.rR.setPower(speed); //-1
                robot.rL.setPower(-speed); //1

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((int)robot.getLeftVertical() > newLeftEncoderTarget) && ((int)robot.getRightVertical() < newRightEncoderTarget)) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d (left), %7d", newLeftEncoderTarget, newRightEncoderTarget);
                    telemetry.addData("Vertical Average", robot.getStrafe());
                    telemetry.update();
                }
            }

            // Stop all motion;
            robot.fR.setPower(0);
            robot.fL.setPower(0);
            robot.rR.setPower(0);
            robot.rL.setPower(0);
        }
    }

    public void telemetryStrafe(double speed, int strafeEncoderTicks, double timeoutS) {
        int newStrafeEncoderTarget;

        if (opModeIsActive()) {
            if (strafeEncoderTicks > 0) {
                newStrafeEncoderTarget = (int)(robot.getStrafe()) + strafeEncoderTicks;

                // reset the timeout time and start motion.
                runtime.reset();
                robot.fR.setPower(-speed); //-1
                robot.fL.setPower(speed); //1
                robot.rR.setPower(speed); //1
                robot.rL.setPower(-speed); //-1

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((int)robot.getStrafe() < newStrafeEncoderTarget)) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d", newStrafeEncoderTarget);
                    telemetry.addData("Vertical Average", robot.getStrafe());
                    telemetry.update();
                }

                // Stop all motion;
                robot.fR.setPower(0);
                robot.fL.setPower(0);
                robot.rR.setPower(0);
                robot.rL.setPower(0);
            }
            if (strafeEncoderTicks < 0) {
                newStrafeEncoderTarget = (int)(robot.getStrafe()) + strafeEncoderTicks;

                // reset the timeout time and start motion.
                runtime.reset();
                robot.fR.setPower(speed); //1
                robot.fL.setPower(-speed); //-1
                robot.rR.setPower(-speed); //-1
                robot.rL.setPower(speed); //1

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((int)robot.getStrafe() > newStrafeEncoderTarget)) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d", newStrafeEncoderTarget);
                    telemetry.addData("Vertical Average", robot.getStrafe());
                    telemetry.update();
                }

                // Stop all motion;
                robot.fR.setPower(0);
                robot.fL.setPower(0);
                robot.rR.setPower(0);
                robot.rL.setPower(0);
            }
        }
    }

    public void telemetryDrive(double speed, int verticalEncoderTicks, double timeoutS) {
        int newVerticalEncoderTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            if (verticalEncoderTicks > 0) {
                // Determine new target position, and pass to motor controller
                newVerticalEncoderTarget = (int)robot.getVerticalAverage() + verticalEncoderTicks;

                // reset the timeout time and start motion.
                runtime.reset();
                robot.fR.setPower(speed); //1
                robot.fL.setPower(0.8*speed); //1
                robot.rR.setPower(-speed); //1
                robot.rL.setPower(-speed); //1

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((int)robot.getVerticalAverage() < newVerticalEncoderTarget)) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d", newVerticalEncoderTarget);
                    telemetry.addData("Vertical Average", robot.getVerticalAverage());
                    telemetry.update();
                }
            }
            if (verticalEncoderTicks < 0) {
                // Determine new target position, and pass to motor controller
                newVerticalEncoderTarget = (int)robot.getVerticalAverage() + verticalEncoderTicks;

                // reset the timeout time and start motion.
                runtime.reset();
                robot.fR.setPower(-speed);
                robot.fL.setPower(0.8*-speed);
                robot.rR.setPower(speed);
                robot.rL.setPower(speed);

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((int)robot.getVerticalAverage() > newVerticalEncoderTarget)) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d", newVerticalEncoderTarget);
                    telemetry.addData("Vertical Average", robot.getVerticalAverage());
                    telemetry.update();
                }
            }

            // Stop all motion;
            robot.fR.setPower(0);
            robot.fL.setPower(0);
            robot.rR.setPower(0);
            robot.rL.setPower(0);
        }
    }


}
