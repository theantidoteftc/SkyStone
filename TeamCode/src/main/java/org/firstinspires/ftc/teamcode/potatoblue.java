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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="potato - blue", group="potato")
public class potatoblue extends LinearOpMode {

    /* Declare OpMode members. */
    OverflowHardware robot = new OverflowHardware();
    private ElapsedTime     runtime = new ElapsedTime();

    private int key = 1;

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

        //robot.swivel.setPosition(0.37); //.4
        robot.swivel.setPosition(0.15);
        robot.gripper.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //robot.intakeLock.setPosition(0);
        //robot.gripper.setPosition(1);

        sleep(1000);

        if (key == 1) {
            //robot.swivel.setPosition(0.30);
            sleep(1000);

            /*telemetryStrafe(0.35,-1500,5);
            sleep(150);
            telemetryDrive(0.45,-200,5);
            sleep(150);
            telemetryStrafe(0.45,-5900,5);
            sleep(150);
            robot.intakeLeft.setPower(0.5);
            robot.intakeRight.setPower(0.5);
            telemetryDrive(0.3,1250,5);
            sleep(750);
            robot.intakeLeft.setPower(0);
            robot.intakeRight.setPower(0);
            robot.swivel.setPosition(0.05);
            sleep(250);
            robot.gripper.setPosition(0);
            telemetryStrafe(0.45,3050,5);
            sleep(250);
            telemetryTurn(0.3,40,2);
            telemetryDrive(0.60,-14200,10);
            sleep(250);
            telemetryTurn(0.3,2250,5);*/
            telemetryStrafe(0.4,1500,5);
            sleep(250);
            telemetryDrive(0.3,-6000,5);
            robot.grabberLeft.setPosition(1);
            robot.grabberRight.setPosition(0);
            sleep(1000);
            telemetryDrive(0.4,3600,5);
            telemetryTurn(0.8,-3425,5);
            telemetryDrive(0.6,-2250,2);
            //robot.swivel.setPosition(1);
            sleep(1000);
            robot.gripper.setPosition(1);
            sleep(200);
            //robot.swivel.setPosition(0.05);
            robot.grabberLeft.setPosition(0);
            robot.grabberRight.setPosition(1);
            //telemetryStrafe(0.6,200,4);
            telemetryStrafe(0.5,-800,3);
            telemetryTurn(0.125,25,2);
            telemetryDrive(0.3,7000,5);
        } else if (key == 2) {
            robot.swivel.setPosition(0.37);
            sleep(1000);

            telemetryStrafe(0.35,1500,5);
            sleep(50);
            telemetryDrive(0.25,-2400,5);
            telemetryStrafe(0.45,6000,5);
            robot.intakeLeft.setPower(0.8);
            robot.intakeRight.setPower(0.8);
            telemetryDrive(0.3,1250,5);
            sleep(750);
            robot.intakeLeft.setPower(0);
            robot.intakeRight.setPower(0);
            robot.swivel.setPosition(0.05);
            sleep(250);
            robot.gripper.setPosition(0);
            telemetryStrafe(0.45,-2600,5);
            //telemetryTurn(0.2,-30,2);
            telemetryDrive(0.60,-12850,10);
            sleep(250);
            telemetryTurn(0.3,-2200,5);
            telemetryDrive(0.3,-1625,5);
            robot.grabberLeft.setPosition(1);
            robot.grabberRight.setPosition(0);
            sleep(1000);
            telemetryDrive(0.4,2500,5);
            telemetryTurn(0.8,3425,5);
            telemetryDrive(0.6,-2250,2);
            robot.swivel.setPosition(1);
            sleep(1000);
            robot.gripper.setPosition(1);
            sleep(200);
            robot.swivel.setPosition(0.05);
            robot.grabberLeft.setPosition(0);
            robot.grabberRight.setPosition(1);
            //telemetryStrafe(0.6,200,4);
            telemetryStrafe(0.5,750,3);
            telemetryTurn(0.125,-25,2);
            telemetryDrive(0.3,7000,5);
        } else if (key == 3) {
            telemetryStrafe(0.4,4100,5);
            sleep(50);
            telemetryDrive(0.35,-4600,5);
            telemetryTurn(0.35,1425,5);
            telemetryDrive(0.35,1250,5);
            telemetryStrafe(0.3,150,5);
            robot.swivel.setPosition(0.37);
            sleep(100);
            robot.intakeLeft.setPower(.8);
            robot.intakeRight.setPower(.8);
            telemetryDrive(0.3,1500,5);
            sleep(750);
            robot.intakeLeft.setPower(0);
            robot.intakeRight.setPower(0);
            robot.swivel.setPosition(0.05);
            sleep(250);
            robot.gripper.setPosition(0);
            telemetryStrafe(0.3,-200,5);
            telemetryDrive(0.4,-2400,5);
            telemetryTurn(0.35,-1750,5);
            telemetryDrive(0.60,-8500,10);
            sleep(250);
            telemetryTurn(0.35,-2100,5);
            telemetryDrive(0.45,-1625,5);
            robot.grabberLeft.setPosition(1);
            robot.grabberRight.setPosition(0);
            sleep(1000);
            telemetryDrive(0.4,2750,5);
            telemetryTurn(0.8,3425,5);
            telemetryDrive(0.6,-2250,2);
            robot.swivel.setPosition(1);
            sleep(1000);
            robot.gripper.setPosition(1);
            sleep(200);
            robot.swivel.setPosition(0.05);
            robot.grabberLeft.setPosition(0);
            robot.grabberRight.setPosition(1);
            telemetryStrafe(0.5,1100,3);
            telemetryTurn(0.125,-50,2);
            telemetryDrive(0.3,7000,5);
        }

        /*telemetryDrive(0.8,16250,7);
        sleep(250);
        telemetryStrafe(0.6,3000,5);
        robot.swivel.setPosition(0.37);
        robot.intakeLeft.setPower(0.8);
        robot.intakeRight.setPower(0.8);
        telemetryDrive(0.2,1000,5);
        sleep(750);
        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);
        robot.swivel.setPosition(0.175);
        sleep(250);
        robot.gripper.setPosition(0);
        telemetryStrafe(0.45,-1900,5);
        telemetryDrive(0.65,-13000,5);
        robot.swivel.setPosition(1);
        sleep(1000);
        robot.gripper.setPosition(1);
        sleep(1000);
        robot.swivel.setPosition(0.175);
        telemetryDrive(0.25,3000,4);*/

        sleep(2000);


        /*telemetryDrive(0.25,4500,5);
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
        robot.intakeRight.setPower(0);*/


        telemetry.addData("Vertical Average", robot.getVerticalAverage());
        telemetry.addData("Strafe", robot.getStrafe());
        telemetry.update();
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
                robot.fL.setPower(speed); //1
                robot.rR.setPower(speed); //1
                robot.rL.setPower(speed); //1

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
                robot.fL.setPower(-speed);
                robot.rR.setPower(-speed);
                robot.rL.setPower(-speed);

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
