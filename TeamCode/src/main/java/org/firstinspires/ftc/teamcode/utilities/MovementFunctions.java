package org.firstinspires.ftc.teamcode.utilities;

public class MovementFunctions {

    //this is to convert the robot's angle to turn to into a power, so the bigger the error, the larger the power
    final static double ANGLE_POWER_CONVERTER = 0.8/Math.PI;

    //creates a "direction" for the robot to go in.
    //also returns an optimal angle to go to but that's optional depending on usage
    public static double[] createMovementVector(double[] robotPose, double[] targetPoint) {
        double relativeX = targetPoint[0] - robotPose[0];
        double relativeY = targetPoint[1] - robotPose[1];

        double absoluteAngle = Math.atan2(relativeY,relativeX);

        double relativeAngle = MathFunctions.angleWrap(Math.PI/2-absoluteAngle+robotPose[2]);

        double divisor = Math.max(Math.abs(relativeX), Math.abs(relativeY));

        return new double[] {
                Math.sin(relativeAngle),
                Math.cos(relativeAngle),
                ANGLE_POWER_CONVERTER * relativeAngle
        };
    }

    public static double[] createMovementVector(double[] robotPose, double[] targetPoint, double optimalAngle) {
        double relativeX = targetPoint[0] - robotPose[0];
        double relativeY = targetPoint[1] - robotPose[1];

        double absoluteAngle = Math.atan2(relativeY,relativeX);

        double relativeAngle = MathFunctions.angleWrap(absoluteAngle - robotPose[2])  - (Math.PI/2);

        double divisor = Math.max(Math.abs(relativeX), Math.abs(relativeY));

        return new double[] {
                Math.sin(relativeAngle),
                Math.cos(relativeAngle),
                ANGLE_POWER_CONVERTER * (relativeAngle + optimalAngle)
        };
    }

    public static double proportionalAngleCorrection(double optimalAngle, double robotAngle) {
        return ANGLE_POWER_CONVERTER * MathFunctions.angleWrap(optimalAngle - robotAngle);
    }

}
