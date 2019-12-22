package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.Locale;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.
If you use our code and see us at competition, come say hello!
*/

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "pranavbrokefieldcentric", group = "TeleOp")
public class pranavbrokefieldcentric extends OpMode {
    private static double JoyStickAngleRad;
    private static double JoyStickAngleDeg;
    private static DcMotor fL;
    private static DcMotor rL;
    private static DcMotor fR;
    private static DcMotor rR;
    private static DcMotor intakeLeft;
    private static DcMotor intakeRight;
    private static double PosXAngPosY;
    private static double PosXAngNegY;
    private static double NegXAng;
    private static double Theta;
    private static double r;
    private static double outputX;
    private static double outputY;
    private static double heading;
    Double Deg = Math.PI;
    BNO055IMU imu;
    Orientation angles;
    double zeroPos = 0;

    OverflowHardware robot = new OverflowHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);

        fL = hardwareMap.dcMotor.get("fL");
        rL = hardwareMap.dcMotor.get("rL");
        fR = hardwareMap.dcMotor.get("fR");
        rR = hardwareMap.dcMotor.get("rR");
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        rL.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        telemetry.addData("Ready", "Good Luck!");
        telemetry.update();
    }

    @Override
    public void loop() {
        double inputY = gamepad1.left_stick_y;
        double inputX = -gamepad1.left_stick_x;
        double inputC = -gamepad1.right_stick_x;
        //the negative signs in front of the gamepad inputs may need to be removed.
        driveMecanum(inputY, inputX, inputC);

        intakeLeft.setPower(-(gamepad1.right_trigger-gamepad1.left_trigger));
        intakeRight.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

        if(gamepad1.left_stick_x >=0 && gamepad1.left_stick_y < 0){
            JoyStickAngleRad = PosXAngPosY;
        }
        else if(gamepad1.left_stick_x >=0 && gamepad1.left_stick_y>=0){
            JoyStickAngleRad = PosXAngNegY;
        }
        else {
            JoyStickAngleRad = NegXAng;
        }
        r = Math.sqrt(gamepad1.left_stick_x* gamepad1.left_stick_x + gamepad1.left_stick_y* gamepad1.left_stick_y);
        if( gamepad1.left_stick_y < 0){ Theta = -Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x); }
        if( gamepad1.left_stick_y >= 0){ Theta = 2*Math.PI - Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x); }
        outputX = -Math.cos(heading - Theta)*r;
        outputY = Math.sin(heading - Theta)*r;
        telemetry.addData("LeftX",gamepad1.left_stick_x);
        telemetry.addData("LeftY", -gamepad1.left_stick_y);
        telemetry.addData("r",r);
        telemetry.addData("Theta", Math.toDegrees(Theta));
        telemetry.addData("outputX",outputX);
        telemetry.addData("outputY",outputY);
        telemetry.addData("angle:",adjustAngle(getAbsoluteHeading()));
        telemetry.addData("s4tright", robot.fR.getCurrentPosition());
        telemetry.addData("s4tleft", robot.fL.getCurrentPosition());
        telemetry.addData("s4tstrafe", robot.rR.getCurrentPosition());
        //telemetry.addData("Angle:",imu.getPosition().z);
        telemetry.update();
        heading = Math.toRadians(getAbsoluteHeading());
        if (gamepad1.a){
            heading = 0;
        }
    }
    public double adjustAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
    public double getAbsoluteHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }
    Double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.valueOf(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public static void driveMecanum(double forwards, double horizontal, double turning) {

        double leftFront = outputY + outputX + turning;
        double leftBack = outputY - outputX + turning;
        double rightFront = outputY - outputX - turning;
        double rightBack = outputY + outputX - turning;

        double[] wheelPowers = {Math.abs(rightFront), Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightBack)};
        Arrays.sort(wheelPowers);
        double biggestInput = wheelPowers[3];
        if (biggestInput > 1) {
            leftFront /= biggestInput;
            leftBack /= biggestInput;
            rightFront /= biggestInput;
            rightBack /= biggestInput;
        }

        fL.setPower(-leftFront/2);
        fR.setPower(-rightFront/2);
        rL.setPower(-leftBack/2);
        rR.setPower(-rightBack/2);

    }

}