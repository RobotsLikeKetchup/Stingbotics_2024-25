package org.firstinspires.ftc.teamcode.utilities;

public class MathFunctions {
    public static int toInt(boolean Boolean) {
        return Boolean ? 1:0;
    }

    public static double angleWrap(double angle) {
        while(angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    //multiply each item in an array by a single number
    public static double[] scaleArray(double scale, double[] array) {
        for (int i = 0; i < array.length; i++) {
            array[i] = array[i] * scale;
        }
        return array;
    }

    public static double[] VectorMatrixMultiplication3d(double[] vector, double[][] matrix) {
        return new double[] {
                matrix[0][0]*vector[0] + matrix[0][1]*vector[1] + matrix[0][2]*vector[2],
                matrix[1][0]*vector[0] + matrix[1][1]*vector[1] + matrix[1][2]*vector[2],
                matrix[2][0]*vector[0] + matrix[2][1]*vector[1] + matrix[2][2]*vector[2]
        };
    }
}
