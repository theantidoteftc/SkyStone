package org.firstinspires.ftc.teamcode.Hardware;

import java.util.Arrays;

public class Drivetrain {

    double motorfR = 0; //motor power variables
    double motorfL = 0;
    double motorrR = 0;
    double motorrL = 0;

    public void driveRobotcentric(double forwardPower, double strafePower, double turningPower) {
        motorfR = -forwardPower - strafePower - turningPower; //robotcentric
        motorfL = -forwardPower + strafePower + turningPower;
        motorrR = forwardPower + strafePower - turningPower;
        motorrL = forwardPower - strafePower + turningPower;
    }

    public double getMotorfR() {
        return motorfR;
    }

    public double getMotorfL() {
        return motorfL;
    }

    public double getMotorrR() {
        return motorrR;
    }

    public double getMotorrL() {
        return motorrL;
    }

}
