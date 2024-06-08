package org.firstinspires.ftc.teamcode;

public class MecanumKinematics {
    static double x, y, rotationDirection, powerLimiter, frontLeft, frontRight, backLeft, backRight;

    //Power is 0-1
    public static double[] getPowerFromPowerAndDirection(double[] targetPower) {
        x = targetPower[0];
        y = targetPower[1];
        //either 1 or -1(1 is counterclockwise, -1 is counterclockwise
        rotationDirection = targetPower[2];
        powerLimiter = Math.max(Math.abs(x+y+rotationDirection),1);

        frontLeft = (y+x- rotationDirection)/powerLimiter;
        frontRight = (y-x+ rotationDirection)/powerLimiter;
        backLeft= (y-x- rotationDirection)/powerLimiter;
        backRight= (y+x+ rotationDirection)/powerLimiter;

        return new double[]{frontLeft,frontRight,backLeft,backRight};
    }
    //add from ejml matrix, etc.
    //add for vectors and stuff, for path following

}
