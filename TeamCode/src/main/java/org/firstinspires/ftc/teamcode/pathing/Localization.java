package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.hardware.Robot;

//localization
public class Localization {
    //fields
    Robot robot;
    //all values in cm or radians
    double fieldX, fieldY, latDist, forwardDist, fieldTheta, x0, y0, angle0;
    //latDist is the lateral distance between the two parallel dead wheels
    //forward distance of the perpendicular wheel from the center of rotation
    //all variables ending in '0' are variables indicating where the robot started.
    double[] pose = {fieldX, fieldY, fieldTheta};
    DeadWheel dW0, dW1, dW2;
    //0 -> parallel Left
    //1 -> parallel right
    //2 -> perpendicular
    double dW01, dW11, dW21, dW02, dW12, dW22, delta1, delta2, delta0, deltaY, deltaX, deltaA;
    /*
    - last number is 1 -> prev position
    - last number is 2 -> current position
    - 'deltas' are just the difference between prev and current position for each time the control loop runs
    - Y is the displacement perpendicular to the heading, NOT the field-relative displacement(only for mechanum drives)
    - X is displacement parallel to the heading, NOT the field-relative displacement
    - These are essentially the change using a robot-centric coordinate system, in alignment with the robots heading
    - we will use a rotation matrix when calculating the pose to convert these to field-centric values
    */

    //constructor
    public Localization(Robot theRobot, double startX, double startY, double startHeading){
        robot = theRobot;
        fieldX = startX;
        fieldY = startY;
        fieldTheta = startHeading;
        x0 = startX;
        y0 = startY;
        angle0 = startHeading;
        dW0 = robot.getDeadwheel("parL");
        dW1 = robot.getDeadwheel("parR");
        dW2 = robot.getDeadwheel("per");
        dW0.reset();
        dW1.reset();
        dW2.reset();
        dW01 = dW0.getDistance();
        dW11 = dW1.getDistance();
        dW21 = dW2.getDistance();
    }
    private void updatePose(){
        pose[0] = fieldX;
        pose[1] = fieldY;
        pose[2] = fieldTheta;
    };
    public void setWheelDistances(float lateralDistance, float forwardDisplacement){
        latDist = lateralDistance;
        forwardDist = forwardDisplacement;
    };
    //methods
    public double[] getPose(){
        updatePose();
        return pose;
    };

    public void calcPose(){//This method should be called once during a while(opModeIsActive) loop
        dW02 = dW0.getDistance();
        dW12 = dW1.getDistance();
        dW22 = dW2.getDistance();
        delta0 = dW02-dW01;
        delta1 = dW12-dW11;
        delta2 = dW22-dW21;

        /*
        I could put in the actual calculations now, but I want to
        make some math classes for matrices and vectors, so that the math
        behind the calculations is clear for future programmers
         */

        updatePose();
    }
}