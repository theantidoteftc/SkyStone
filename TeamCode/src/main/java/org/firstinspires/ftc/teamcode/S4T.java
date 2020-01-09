package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class S4T extends LinearOpMode {

    /* Public OpMode members. */
    public DcMotor fL   = null;
    public DcMotor fR  = null;
    public DcMotor rL  = null;
    public DcMotor rR  = null;

    // The IMU sensor object
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        fL  = hardwareMap.get(DcMotor.class, "fL"); //verticalLeft
        fR = hardwareMap.get(DcMotor.class, "fR"); //verticalRight
        rL  = hardwareMap.get(DcMotor.class, "rL"); //strafing
        rR = hardwareMap.get(DcMotor.class, "rR");

        fL.setDirection(DcMotor.Direction.REVERSE);
        rL.setDirection(DcMotor.Direction.REVERSE);

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

        //robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //robot.resetMotorEncoders();

        while (opModeIsActive()) {
            telemetry.addData("s4tright (fR)", fR.getCurrentPosition());
            telemetry.addData("s4tleft (fL)", fL.getCurrentPosition());
            telemetry.addData("s4tstrafe (rL)", -rR.getCurrentPosition());

            telemetry.addData("OpMode", "running!");
            telemetry.update();
        }
    }

}