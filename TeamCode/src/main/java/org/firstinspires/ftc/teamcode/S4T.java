package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class S4T extends LinearOpMode {
    OverflowHardware robot = new OverflowHardware();

    // The IMU sensor object
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.resetMotorEncoders();

        while (opModeIsActive()) {
            telemetry.addData("s4tright (fR)", robot.fR.getCurrentPosition());
            telemetry.addData("s4tleft (fL)", robot.fL.getCurrentPosition());
            telemetry.addData("s4tstrafe (rL)", robot.rL.getCurrentPosition());

            telemetry.addData("OpMode", "running!");
            telemetry.update();
        }
    }

}