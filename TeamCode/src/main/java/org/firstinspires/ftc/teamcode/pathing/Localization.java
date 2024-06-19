package org.firstinspires.ftc.teamcode.pathing;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.utilities.NanoTimer;

//localization
public class Localization {
    //fields
    Robot robot;
    //all values in cm or radians
    double latDist, forwardDist, x0, y0, angle0;
    //latDist is the lateral distance between the two parallel dead wheels
    //forward distance of the perpendicular wheel from the center of rotation
    //all variables ending in '0' are variables indicating where the robot started.
    double[] pose;
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

    SimpleMatrix rotationalMatrix, curveMatrix, deltaPose;

    NanoTimer velocityTimer;


    //constructor
    public Localization(Robot theRobot, double[] startPose){
        robot = theRobot;
        pose = startPose;
        x0 = startPose[0];
        y0 = startPose[1];
        angle0 = startPose[2];
        parLDeadwheel = robot.getDeadwheel("parL");
        parRDeadWheel = robot.getDeadwheel("parR");
        perpDeadwheel = robot.getDeadwheel("per");
        velocityTimer = new NanoTimer();
    }
    public void setWheelDistances(float lateralDistance, float forwardDisplacement){
        latDist = lateralDistance;
        forwardDist = forwardDisplacement;
    };
    //methods
    public double[] getPose(){
        return pose;
    }

    public double[] getVelocity(){
        return velocity;
    }

    public void init(){
        velocityTimer.resetTimer();
        parLDeadwheel.reset();
        parRDeadWheel.reset();
        perpDeadwheel.reset();
        dW01 = parLDeadwheel.getDistance();
        dW11 = parRDeadWheel.getDistance();
        dW21 = perpDeadwheel.getDistance();
    };

    public void calcPose(){//This method should be called once during a while(opModeIsActive) loop
        //encoder deltas
        dW02 = parLDeadwheel.getDistance();
        dW12 = parRDeadWheel.getDistance();
        dW22 = perpDeadwheel.getDistance();
        delta0 = dW02-dW01;
        delta1 = dW12-dW11;
        delta2 = dW22-dW21;

        //robot-relative deltas
        rDeltaA = (delta0-delta1)/latDist;
        rDeltaY = delta2 - (forwardDist * rDeltaA); //perpendicular(strafe) distance, MINUS arclength
        rDeltaX = (delta0 + delta1)/2;

        //rotation matrix, calculated on the pose from each loop
        rotationalMatrix = new SimpleMatrix(new double[][]{
                {Math.cos(pose[0]), -1 * Math.sin(pose[0]), 0},
                {Math.sin(pose[0]), Math.cos(pose[0]), 0},
                {0, 0, 1}
        });
        //if the change in heading is close enough to zero, make the curveMatrix an identity matrix
        if (rDeltaA < 0.05 && rDeltaA > -0.05){
            curveMatrix = SimpleMatrix.identity(3);
        } else {
            /*
            * Pose exponential method for calculating change in position, assuming constant curvature of robot path
            * not gonna go through the really complex math here, but ask Umed(me) for more info
            * this takes change in robot heading, rather than the heading itself
             */
            curveMatrix = new SimpleMatrix(new double[][] {
                {Math.sin(rDeltaA)/rDeltaA, (Math.cos(rDeltaA)-1)/rDeltaA, 0},
                {(1-Math.cos(rDeltaA))/rDeltaA, Math.sin(rDeltaA)/rDeltaA, 0},
                {0,0,1},
            });
        }
        //multiply matrices to get global change in pose
        deltaPose = new SimpleMatrix(new double[] {rDeltaX, rDeltaY, rDeltaA});
        deltaPose = deltaPose.mult(curveMatrix).mult(rotationalMatrix);

        //integrate
        pose[0] = pose[0] + deltaPose.get(0);
        pose[1] = pose[1] + deltaPose.get(1);
        pose[2] = pose[2] + deltaPose.get(2);

        //get velocities
        velocity[0] = deltaPose.get(0)/velocityTimer.getElapsedTime();
        velocity[1] = deltaPose.get(1)/velocityTimer.getElapsedTime();
        velocity[2] = deltaPose.get(2)/velocityTimer.getElapsedTime();

        velocityTimer.resetTimer();
    }
}