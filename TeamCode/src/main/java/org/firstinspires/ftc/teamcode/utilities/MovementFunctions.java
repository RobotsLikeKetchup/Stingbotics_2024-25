package org.firstinspires.ftc.teamcode.utilities;

public class MovementFunctions {

    //creates a "direction" for the robot to go in.
    //also returns an optimal angle to go to but that's optional depending on usage
    public static double[] createMovementVector(double[] robotPose, double[] targetPoint) {
        double relativeX = targetPoint[0] - robotPose[0];
        double relativeY = targetPoint[1] - robotPose[1];

        double absoluteAngle = Math.atan2(relativeY,relativeX);

        double relativeAngle = MathFunctions.angleWrap(absoluteAngle - robotPose[2]);

        double divisor = Math.max(Math.abs(relativeX), Math.abs(relativeY));

        return new double[] {
                Math.sin(relativeAngle),
                Math.cos(relativeAngle),
                relativeAngle
        };
    }

}
