package org.firstinspires.ftc.teamcode.pathing;

import java.util.ArrayList;

//this class will implement the pure pursuit pathing algorithm
public class PurePursuit {
    double[][] path;
    int lastFoundIndex;

    double distanceFromEnd;

    double[] robotPosition;

    StingLocalizer localization;
    public PurePursuit(double[][] pathPoints, StingLocalizer localizer) {
        path = pathPoints;
        localization = localizer;
        lastFoundIndex = 0;
    }

    // this method returns -1 if the number is <0, and 1 if anything else(including 0). it is for calculating line-circle intersection
    int sgn(double number) {
        if(number < 0){
            return -1;
        } else{
            return 1;
        }
    }

    double twoPointDistance(double[] point1, double[] point2){
        return Math.sqrt(Math.pow(point2[0]-point1[0], 2) + Math.pow(point2[1] - point1[1],2));
    }


    /*
    this method is for calculating the line-circle intersection between the robot's look-ahead radius and the path.
    it returns a array of all solutions. The array can have from 0 to 2 points
    you can find more info about the math used at https://mathworld.wolfram.com/Circle-LineIntersection.html
     */
    double[][] intersection(double[] robotPosition, double[] pt1, double[] pt2, double lookAheadDistance){

        double currentX = robotPosition[0];
        double currentY = robotPosition[1];

        //use an arrayList so I can make different sized arrays and append stuff. just makes things easier
        ArrayList<double[]> solutions = new ArrayList<>();

        //move the line so that the origin is the robot location(to make math simpler)
        double x1 = pt1[0] - currentX;
        double x2 = pt2[0] - currentX;
        double y1 = pt1[1] - currentY;
        double y2 = pt2[1] - currentY;

        //math helper variables
        double xDifference = x2 - x1;
        double yDifference = y2 - y1;
        double differenceRadius = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2));
        //this determinant is of the matrix of the first and second points(treating both points as column vectors)
        double determinant = (x1*y2)-(x2*y1);

        double discriminant = (Math.pow(lookAheadDistance, 2) * Math.pow(differenceRadius, 2)) - Math.pow(determinant, 2);

        //do the math for line-circle intersection if there exist solutions
        if (discriminant >= 0) {
            double solutionX1 = (determinant * yDifference + (sgn(yDifference)*xDifference*Math.sqrt(discriminant)))/Math.pow(differenceRadius, 2);
            double solutionX2 = (determinant * yDifference - (sgn(yDifference)*xDifference*Math.sqrt(discriminant)))/Math.pow(differenceRadius, 2);
            double solutionY1 = (-1*determinant*xDifference + (Math.abs(yDifference)*Math.sqrt(discriminant)))/Math.pow(differenceRadius, 2);
            double solutionY2 = (-1*determinant*xDifference - (Math.abs(yDifference)*Math.sqrt(discriminant)))/Math.pow(differenceRadius, 2);

            //add the previous offset back into the solutions
            double[] solution1 = {solutionX1 + currentX, solutionY1 + currentY};
            double[] solution2 = {solutionX2 + currentX, solutionY2 + currentY};

            //check if the solutions are within bounds of the path
            if(withinBounds(solution1, pt1, pt2) || withinBounds(solution2, pt1, pt2)) {

                //now check which solutions are correct
                if(withinBounds(solution1, pt1, pt2)) {
                    solutions.add(solution1);
                }
                if(withinBounds(solution2, pt1, pt2)){
                    solutions.add(solution2);
                }
            }
        }

        //converts the ArrayList to an array, to make it easier to work with
        return solutions.toArray(new double[solutions.size()][]);
    }

    boolean withinBounds(double[] point, double[] bound1, double[] bound2) {
        double maxX = Math.max(bound1[0], bound2[0]);
        double maxY = Math.max(bound1[1], bound2[1]);
        double minX = Math.min(bound1[0], bound2[0]);
        double minY = Math.min(bound1[1], bound2[1]);

        if((minX <= point[0] && point[0] <= maxX) && (minY <= point[1] && point[1] <= maxY)) {
            return true;
        } else {
            return false;
        }
    }

    public double[] findPointOnPath(){

        robotPosition = localization.getPose();
        //distance to "look ahead" for the pure pursuit algorithm
        double lookAheadDistance = 1.5;
        boolean intersectionFound = false;

        double[] goalPoint = new double[2];

        //start by iterating through the path
        for (int i = lastFoundIndex; i < (path.length - 1); i++) {
            //find two points to create a line segment
            double[] point1 = path[i];
            double[] point2 = path[i+1];

            double[][] solutions = intersection(robotPosition,point1,point2,lookAheadDistance);

            if(solutions.length > 0) { // solution found!
                intersectionFound = true;

                if(solutions.length == 2){
                    if(twoPointDistance(solutions[0], path[i+1]) < twoPointDistance(solutions[1], path[i+1])){
                        goalPoint = solutions[0];
                    } else {
                        goalPoint = solutions[1];
                    }
                } else {
                    goalPoint = solutions[0];
                }

                if(twoPointDistance(goalPoint, path[i+1]) < twoPointDistance(robotPosition, path[i+1])){
                    lastFoundIndex = i;
                    break;
                } else {
                    lastFoundIndex = i+1;
                }

            } else { //no solution found!
                intersectionFound = false;
                goalPoint[0] = path[lastFoundIndex][0];
                goalPoint[1] = path[lastFoundIndex][1];
            }
        }
        return goalPoint;
    }

    public int getLastFoundIndex() {
        return lastFoundIndex;
    }

    public double getDistanceFromEnd(){
        distanceFromEnd = twoPointDistance(robotPosition, path[path.length - 1]);

        return distanceFromEnd;
    }
}
