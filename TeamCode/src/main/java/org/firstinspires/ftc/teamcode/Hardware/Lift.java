package org.firstinspires.ftc.teamcode.Hardware;

public class Lift {

    double liftLeft = 0; //intake power variables
    double liftRight = 0;

    public void lift(double upwardPower, double downwardPower) {
        liftLeft = upwardPower - downwardPower;
        liftRight = upwardPower - downwardPower;
    }

    public double getLiftLeft() {
        return liftLeft;
    }

    public double getLiftRight() {
        return liftRight;
    }

}
