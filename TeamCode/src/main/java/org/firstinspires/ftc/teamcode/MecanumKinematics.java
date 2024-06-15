package org.firstinspires.ftc.teamcode;

import org.ejml.simple.SimpleMatrix;

public class MecanumKinematics {
    //this should be tuned, but I think the inverse of 2 pi should be fine for now
    static final double ROTATION_SCALE = 1/(2*Math.PI);
    
    public static double[] getPowerFromPower(double[] targetPower){
        double x, y, rotation, powerLimiter, frontLeft, frontRight, backLeft, backRight;
        x = targetPower[0];
        y = targetPower[1];
        //either 1 or -1(1 is counterclockwise, -1 is counterclockwise
        rotation = targetPower[2];
        powerLimiter = Math.max(Math.abs(x+y+rotation),1);

        frontLeft = (y+x- rotation)/powerLimiter;
        frontRight = (y-x+ rotation)/powerLimiter;
        backLeft= (y-x- rotation)/powerLimiter;
        backRight= (y+x+ rotation)/powerLimiter;

        return new double[]{frontLeft,frontRight,backLeft,backRight};
    }
    public static double[] getPowerFromPowerAndDirection(double[] targetPower) { //Power is 0-1
        double x, y, rotationDirection;
        x = targetPower[0];
        y = targetPower[1];
        rotationDirection = 0;
        //either 1 or -1(1 is counterclockwise, -1 is counterclockwise
        if (targetPower[2] < 0) rotationDirection = -1;
        if (targetPower[2] > 0) rotationDirection = 1;

        return getPowerFromPower(new double[]{x,y,rotationDirection});
    }

    public static double[] getPowerFromVector(SimpleMatrix vector){
        double powerLimiter, frontLeft, frontRight, backLeft, backRight, x, y, r;
        //makes sure its a 3-dimensional column vector, if not, throw an error
        if (vector.getNumCols() != 1 || vector.getNumRows() != 3) throw new IllegalArgumentException("Must be a 3-dimensional column vector");
        SimpleMatrix twoDimensions = new SimpleMatrix(new double[] {vector.get(0),vector.get(1)});
        //checks if magnitude is greater than 1, then normalize
        if (twoDimensions.normF() > 1){
            twoDimensions.divide(twoDimensions.normF());
        }
        x = twoDimensions.get(0);
        y = twoDimensions.get(1);
        r=vector.get(2)*ROTATION_SCALE;

        return getPowerFromPower(new double[] {x, y, r});
    }
}
