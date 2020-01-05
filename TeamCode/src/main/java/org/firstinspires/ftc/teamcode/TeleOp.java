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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Lift;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Steven")
//@Disabled
public class TeleOp extends OpMode //steve
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor rL = null;
    private DcMotor rR = null;
    private DcMotor intakeLeft = null;
    private DcMotor intakeRight = null;
    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;

    public Servo grabberLeft = null;
    public Servo grabberRight = null;/*
    public Servo stoneGrabber = null;
    public Servo leftSwivel = null;
    public Servo rightSwivel = null;*/
    public Servo intakeLock = null;
    public ServoImplEx swivel = null;
    public Servo gripper = null;

    Drivetrain myDrivetrain = new Drivetrain();
    Intake myIntake = new Intake();
    Lift myLift = new Lift();

    private boolean toggleSwitch = false;
    private boolean grabbersDown = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fR = hardwareMap.get(DcMotor.class, "fR"); //verticalRight
        fL = hardwareMap.get(DcMotor.class, "fL"); //verticalLeft
        rR = hardwareMap.get(DcMotor.class, "rR");
        rL = hardwareMap.get(DcMotor.class, "rL"); //strafing
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");

        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "grabberRight");
        /*stoneGrabber = hardwareMap.get(Servo.class, "stoneGrabber");
        leftSwivel = hardwareMap.get(Servo.class, "leftSwivel");
        rightSwivel = hardwareMap.get(Servo.class, "rightSwivel");*/
        intakeLock = hardwareMap.get(Servo.class, "intakeLock");
        swivel = hardwareMap.get(ServoImplEx.class, "swivel");
        swivel.setPwmRange(new PwmControl.PwmRange(500.0, 2500.0, 1000000.0/333.0));
        gripper = hardwareMap.get(Servo.class, "gripper");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fL.setDirection(DcMotor.Direction.REVERSE);
        rL.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        intakeLock.setPosition(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //IMPORTANT STEVEN CODE
        myDrivetrain.driveRobotcentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        myIntake.intake(gamepad1.left_trigger, gamepad1.right_trigger);
        myLift.lift(gamepad2.left_trigger, gamepad2.right_trigger);

        //IMPORTANT STEVEN CODE
        fR.setPower(myDrivetrain.getMotorfR());
        fL.setPower(myDrivetrain.getMotorfL());
        rR.setPower(myDrivetrain.getMotorrR());
        rL.setPower(myDrivetrain.getMotorrL());

        intakeLeft.setPower(myIntake.getIntakeLeft());
        intakeRight.setPower(myIntake.getIntakeRight());

        liftLeft.setPower(myLift.getLiftLeft());
        liftRight.setPower(myLift.getLiftRight());

        if (gamepad1.a) {
            grabberLeft.setPosition(1);
            grabberRight.setPosition(0);
        }
        if (gamepad1.b) {
            grabberLeft.setPosition(0);
            grabberRight.setPosition(1);
        }

        if (gamepad2.dpad_up) {
            swivel.setPosition(0.30);
            gripper.setPosition(0);
        }
        if (gamepad2.dpad_right) {
            swivel.setPosition(0.37); //.4
            gripper.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            swivel.setPosition(0.85); //0.88
        }
        if (gamepad2.a) {
            gripper.setPosition(0); //close
        }
        if (gamepad2.b) { //open
            gripper.setPosition(1);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "fR (%.2f), fL (%.2f), rR (%.2f), rL (%.2f)",
                fR.getPower(), fL.getPower(), rR.getPower(), rL.getPower());
        telemetry.addData("Intake", "iL (%.2f), iR (%.2f)",
                intakeLeft.getPower(), intakeRight.getPower());
        telemetry.addData("grabbersDown", grabbersDown);
        telemetry.addData("grabberLeftPos", grabberLeft.getPosition());
        telemetry.addData("grabberRightPos", grabberRight.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        swivel.setPwmDisable();
    }

}
