package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.utilities.MathFunctions;

public class EzraLocalizer implements StingLocalizer{
    //fields
    Robot robot;
    //all values in inches or radians
    double LAT_DISTANCE, FORWARD_DISTANCE;
    double x0, y0, angle0;
    //latDist is the lateral distance between the two parallel dead wheels
    //forward distance of the perpendicular wheel from the center of rotation
    //all variables ending in '0' are variables indicating where the robot started.
    double[] pose, deltaPose;
    double[][] rotationalMatrix, curveMatrix;
    double[] velocity = {0,0,0};
    DeadWheel parLDeadwheel, parRDeadWheel, perpDeadwheel;
    //0 -> parallel Left
    //1 -> parallel right
    //2 -> perpendicular
    double dW01, dW11, dW21, dW02, dW12, dW22, delta1, delta2, delta0, rDeltaY, rDeltaX, rDeltaA;
    /*
    - last number is 1 -> prev position
    - last number is 2 -> current position
    - 'deltas' are just the difference between prev and current position for each time the control loop runs
    - Y is the displacement perpendicular to the heading, NOT the field-relative displacement(only for mecanum drives)
    - X is displacement parallel to the heading, NOT the field-relative displacement
    - These are essentially the change using a robot-centric coordinate system, in alignment with the robots heading
    - we will use a rotation matrix when calculating the pose to convert these to field-centric values
    */

    double startTime;
    ElapsedTime velocityTimer;

    //constructor
    public EzraLocalizer(Robot theRobot, double[] startPose, ElapsedTime timer){
        robot = theRobot;
        pose = startPose;
        x0 = startPose[0];
        y0 = startPose[1];
        angle0 = startPose[2];
        velocityTimer = timer;
    }

    public void init() {
        parLDeadwheel = robot.getDeadwheel("parL");
        parRDeadWheel = robot.getDeadwheel("parR");
        perpDeadwheel = robot.getDeadwheel("per");
    }

    public void setWheelDistances(double lateralDistance, double forwardDisplacement){
        LAT_DISTANCE = lateralDistance;
        FORWARD_DISTANCE = forwardDisplacement;
    }
    //methods
    public double[] getPoseDouble(){
        return pose;
    }

    public double[] getVelocity(){
        return velocity;
    }

    public void start(){
        startTime = velocityTimer.seconds();
        parLDeadwheel.reset();
        parRDeadWheel.reset();
        perpDeadwheel.reset();
        dW01 = parLDeadwheel.getDistanceFromInPerTick();
        dW11 = parRDeadWheel.getDistanceFromInPerTick();
        dW21 = perpDeadwheel.getDistanceFromInPerTick();
    }

    public void calcPose(){//This method should be called once during a while(opModeIsActive) loop
        //encoder deltas
        dW02 = parLDeadwheel.getDistanceFromInPerTick();
        dW12 = parRDeadWheel.getDistanceFromInPerTick();
        dW22 = perpDeadwheel.getDistanceFromInPerTick();
        delta0 = dW02-dW01;
        delta1 = dW12-dW11;
        delta2 = dW22-dW21;

        //robot-relative deltas
        rDeltaA = (delta0-delta1)/ LAT_DISTANCE;
        rDeltaY = delta2 - (FORWARD_DISTANCE * rDeltaA); //perpendicular(strafe) distance, MINUS arclength
        rDeltaX = (delta0 + delta1)/2;

        //rotation matrix, calculated on the pose from each loop
        rotationalMatrix = new double[][]{
                {Math.cos(rDeltaA), -1 * Math.sin(rDeltaA), 0},
                {Math.sin(rDeltaA), Math.cos(rDeltaA), 0},
                {0, 0, 1}
        };
        //if the change in heading is close enough to zero, make the curveMatrix an identity matrix
        if (rDeltaA < 0.05 && rDeltaA > -0.05){
            curveMatrix = new double[][] {{1,0,0},{0,1,0},{0,0,1}};
        } else {
            /*
             * Pose exponential method for calculating change in position, assuming constant curvature of robot path
             * not gonna go through the really complex math here, but ask Umed(me) for more info
             * this takes change in robot heading, rather than the heading itself
             */
            curveMatrix = new double[][] {
                    {Math.sin(rDeltaA)/rDeltaA, (Math.cos(rDeltaA)-1)/rDeltaA, 0},
                    {(1-Math.cos(rDeltaA))/rDeltaA, Math.sin(rDeltaA)/rDeltaA, 0},
                    {0,0,1},
            };
        }
        //multiply matrices to get global change in pose
        deltaPose = new double[] {rDeltaX, rDeltaY, rDeltaA};
        deltaPose = MathFunctions.VectorMatrixMultiplication3d(MathFunctions.VectorMatrixMultiplication3d(deltaPose,curveMatrix),rotationalMatrix);

        //integrate
        pose[0] = pose[0] + deltaPose[0];
        pose[1] = pose[1] + deltaPose[1];
        pose[2] = pose[2] + deltaPose[2];

        //get velocities
        velocity[0] = deltaPose[0]/velocityTimer.seconds() - startTime;
        velocity[1] = deltaPose[1]/velocityTimer.seconds() - startTime;
        velocity[2] = deltaPose[2]/velocityTimer.seconds() - startTime;

        startTime = velocityTimer.seconds();
    }
}
