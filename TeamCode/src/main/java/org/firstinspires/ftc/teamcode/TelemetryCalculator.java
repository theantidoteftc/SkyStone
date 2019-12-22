package org.firstinspires.ftc.teamcode;

//STEVEN'S CODE (written 11/15/19) goodfaithday
//NEVER EVER DELETE WITHOUT ADITYA'S PERMISSION
//OR LIKE NOT AT ALL, THIS IS GOD'S CODE

public class TelemetryCalculator {

    private static final double thetaConstant = 48.3668966;
    private static final double tankConstant = 0.1265363444;
    private static final double strafeConstant = -0.1264754666;
    private static final double strafeOffsetConstant = -1000; //187.33;

    public static double theta (int leftEncoder, int rightEncoder) {
        double theta = (leftEncoder - rightEncoder)/thetaConstant;
        return angleWrap(theta);
        //return theta;
    }

    public static double thetaDelta (double theta, double previousTheta) {
        double thetaDelta = theta - previousTheta;
        return angleWrap(thetaDelta);
    }

    public static double angleWrap (double angle) { //spits out an equivalent angle in between -pi and pi
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        /*while (angle > Math.PI) {
            angle -= 2*Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2*Math.PI;
        }*/
        return angle * (Math.PI/180); //convert from degrees to radians
    }

    public static double tankRadius (double theta, int leftEncoder, int rightEncoder) {
        return ((leftEncoder + rightEncoder)/(2 * theta)) * tankConstant;
    }

    public static double strafeRadius (double theta, double thetaDelta, int strafeEncoder) {
        return (((strafeEncoder + (strafeOffsetConstant * thetaDelta)))/(theta)) * strafeConstant;
    }

    public static double calculateXDelta (double tankRadius, double strafeRadius, double theta) {
        return ((tankRadius)*(1 - Math.cos(theta))) + (strafeRadius * Math.sin(theta));
    }

    public static double calculateYDelta (double tankRadius, double strafeRadius, double theta) {
        return (tankRadius * Math.sin(theta)) + ((strafeRadius)*(1 - Math.cos(theta)));
    }

    public static double tankXDelta (double tankRadius, double theta) {
        return ((tankRadius)*(1 - Math.cos(theta)));
    }

    public static double tankYDelta (double tankRadius, double theta) {
        return (tankRadius * Math.sin(theta));
    }

}
