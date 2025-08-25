package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.utilities.MathFunctions;

public class MecanumKinematics {
    // Converts radians to power
    // should be tuned, temporary for now
    static final double ROTATION_SCALE = 1/(2*Math.PI);
    
    /*
     * Converts from direction to power levels
     * 
     * Param targetPower is an array which contains the following items in order of ascending index:
     * 0. x position of the left stick
     * 1. y position of the left stick
     * 2. bumper status (=1 if left, =-1 if right)
     * 
     * Returns an array which contains the power level to be directed to each wheel. The wheels are contained in the following order:
     * 0. front left
     * 1. front right
     * 2. back left
     * 3. back right
     */
    public static double[] getPowerFromDirection(double[] targetPower, double maxPower){
        double x, y, rotation, powerLimiter, frontLeft, frontRight, backLeft, backRight;
        x = targetPower[0] * 1.1; //1.1 is to counteract imperfect strafing from both path followers and drivers
        y = targetPower[1];
        
        rotation = targetPower[2];

        //limits power so it isn't larger than either the
        powerLimiter = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rotation), 1);

        frontLeft = (-y-x- rotation)/powerLimiter;
        frontRight = (-y+x+ rotation)/powerLimiter;
        backLeft= (-y+x- rotation)/powerLimiter;
        backRight= (-y-x+ rotation)/powerLimiter;

        double[] power = new double[]{frontLeft,frontRight,backLeft,backRight};
        return MathFunctions.scaleArray(maxPower, power);
    }


    //for weight distribution correction
    public static double[] getPowerFromDirection(double[] targetPower, double maxPower, double weightCorrection, boolean front){

        double x, y, rotation, powerLimiter, frontLeft, frontRight, backLeft, backRight, frontCorrection, backCorrection;
        x = targetPower[0] * 1.1;
        y = targetPower[1];

        if(front){
            frontCorrection = weightCorrection;
            backCorrection = 1;
        } else {
            backCorrection = weightCorrection;
            frontCorrection = 1;
        }

        rotation = targetPower[2];
        powerLimiter = Math.max((front ? frontCorrection : backCorrection) * (Math.abs(x) + Math.abs(y) + Math.abs(rotation)), 1);


        frontLeft = (frontCorrection * (-y-x- rotation))/powerLimiter;
        frontRight = (frontCorrection * (-y+x+ rotation))/powerLimiter;
        backLeft= (backCorrection * (-y+x- rotation))/powerLimiter;
        backRight= (backCorrection * (-y-x+ rotation))/powerLimiter;

        double[] power = new double[]{frontLeft,frontRight,backLeft,backRight};
        return MathFunctions.scaleArray(maxPower, power);
    }

}
