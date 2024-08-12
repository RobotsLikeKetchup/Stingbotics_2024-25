package org.firstinspires.ftc.teamcode.pathing;

import java.util.ArrayList;

//this class will implement the pure pursuit pathing algorithm
public class purePursuit {
    double[][] path;
    int lastFoundIndex;

    Localization localization;
    public purePursuit(double[][] pathPoints, Localization localizer) {
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
        return Math.sqrt(Math.pow(point2[0]-point1[0], 2) + Math.pow(point2[1] - point1[0],2));
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

            //get minimum and maximum values of the path segment
            double minX = Math.min(x1, x2);
            double minY = Math.min(y1,y2);
            double maxX = Math.max(x1,x2);
            double maxY = Math.max(y1,y2);

            //check if the solutions are within bounds of the path
            if(((minX <= solution1[0] && solution1[0] <= maxX) && (minY <= solution1[1] && solution1[1] <= maxY)) || ((minX <= solution2[0] && solution2[0] <= maxX) && (minY <= solution2[1] && solution2[1] <= maxY))) {

                //now check which solutions are correct
                if((minX <= solution1[0] && solution1[0] <= maxX) && (minY <= solution1[1] && solution1[1] <= maxY)) {
                    solutions.add(solution1);
                }
                if((minX <= solution2[0] && solution2[0] <= maxX) && (minY <= solution2[1] && solution2[1] <= maxY)){
                    solutions.add(solution2);
                }
            }
        }

        //converts the ArrayList to an array, to make it easier to work with
        return solutions.toArray(new double[solutions.size()][]);
    }

    public double[] findPointOnPath(){

        double[] robotPosition = localization.getPose();
        //distance to "look ahead" for the pure pursuit algorithm
        double lookAheadDistance = 1.5;
        double discriminant, x1, y1, x2, y2;
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
}
