package org.firstinspires.ftc.teamcode;

import org.ejml.simple.SimpleMatrix;

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
    public static double[] getPowerFromDirection(double[] targetPower){
        double x, y, rotation, powerLimiter, frontLeft, frontRight, backLeft, backRight;
        x = targetPower[0];
        y = targetPower[1];
        
        rotation = targetPower[2];
        powerLimiter = Math.max(Math.abs(x+y+rotation),1);

        frontLeft = (y+x- rotation)/powerLimiter;
        frontRight = (y-x+ rotation)/powerLimiter;
        backLeft= (y-x- rotation)/powerLimiter;
        backRight= (y+x+ rotation)/powerLimiter;

        return new double[]{frontLeft,frontRight,backLeft,backRight};
    }

    /*
     * Converts from vector to directions, then uses getPowerFromDirections to return wheel power levels
     * 
     * Param vector contains
     */
    public static double[] getDirectionFromVector(SimpleMatrix vector){
        double powerLimiter, frontLeft, frontRight, backLeft, backRight, x, y, r;

        // Makes sure the matrix has 1 column and 3 rows. If not, throws an error
        if (vector.getNumCols() != 1 || vector.getNumRows() != 3) throw new IllegalArgumentException("Must be a 3-dimensional column vector");

        // Conduct math on the first two rows of the matrix
        SimpleMatrix twoDimensions = new SimpleMatrix(new double[] {vector.get(0),vector.get(1)});
        /*
         * If magnitude is greater than 1, normalize
         * 
         * In math-speak:
         * If the frobenius norm is greater than 1, divide each item in the new matrix by the frobenius norm
         */
        if (twoDimensions.normF() > 1){
            twoDimensions.divide(twoDimensions.normF());
        }

        // Set directions
        x = twoDimensions.get(0);
        y = twoDimensions.get(1);
        r = vector.get(2)*ROTATION_SCALE;

        /*
         * Note from Eleanor:
         * I suggest that we instead use:
         * return new double[] {x, y, r};
         * 
         * if we must nest methods, its neater if they are nested in the opmode in the following manner:
         * getPowerFromDirection(getDirectionFromVector)
         * 
         * as a result, we can return the derived directions via telemetry
         */
        return getPowerFromDirection(new double[] {x, y, r});
    }
}
