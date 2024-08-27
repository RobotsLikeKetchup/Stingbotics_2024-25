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
}
