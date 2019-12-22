package org.firstinspires.ftc.teamcode.Hardware;

public class Intake {

    double intakeLeft = 0; //intake power variables
    double intakeRight = 0;

    public void intake(double intakePower, double outtakePower) {
        intakeLeft = intakePower - outtakePower;
        intakeRight = intakePower - outtakePower;
    }

    public double getIntakeLeft() {
        return intakeLeft;
    }

    public double getIntakeRight() {
        return intakeRight;
    }

}
