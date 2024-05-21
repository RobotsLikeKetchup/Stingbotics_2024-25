package org.firstinspires.ftc.teamcode.math;

import org.checkerframework.checker.units.qual.A;

import java.util.ArrayList;

/*
* This is a matrix class I threw together, so math is easier
* like when I am dealing with vectors and matrices for localization, path following, and so on
* There's probably a ton of things that aren't optimal so if you guys see anything wrong please fix them
* thanks!! - Umed
* */

public class Matrix {
    int rowDimension, columnDimension;
    double [][] matrix;

    public Matrix(int rows, int columns, double value){ //matrix filled with a number
        rowDimension = rows;
        columnDimension = columns;
        ArrayList<ArrayList<Double>> fillMatrix = new ArrayList<ArrayList<Double>>();
        ArrayList<Double> fillRow = new ArrayList<Double>();
        for (int i=0;i<rows;i++){
            for (int e=0; e<columns;e++){
                //fills a builder array with the value
                fillRow.add(value);
            }
            //sticks that row into the 2d builder array, then clears it for the next iteration
            fillMatrix.add(fillRow);
            fillRow.clear();
        }
        //map the arrayList into the array, through an intermediary stream
        matrix = fillMatrix.stream().map(u->u.stream().mapToDouble(i->i).toArray()).toArray(double[][]::new);
    }

    public Matrix(){ //create a matrix that is zero dimensions
        rowDimension = 0;
        columnDimension = 0;
    }
    public Matrix(double [][] fullMatrix) { //create matrix from 2-d array
        //test to make sure the rows of the matrix match; if not, throw an exception
        double[] prevRow = fullMatrix[0];
        for (int i = 1;i < fullMatrix.length; i++){ // i is 1 here cause we already stored [0] in prevRow
            if(prevRow != fullMatrix[i]){
                throw new IllegalArgumentException("Matrix Rows" + i + "and" + (i-1) + "don't match");
            }
        }
        columnDimension = fullMatrix.length;
        rowDimension = fullMatrix[0].length;
        matrix = fullMatrix;
    }

    public Matrix getTranspose(Matrix aMatrix){
        ArrayList<ArrayList<Double>> transposeBuilder = new ArrayList<ArrayList<Double>>();
        ArrayList<Double> fillRow = new ArrayList<Double>();
        for (int e = 0; e < aMatrix.rowDimension; e++) {
            for (int i = 0; i < aMatrix.columnDimension; i++) {
                fillRow.add(aMatrix.matrix[i][e]);
            }
            transposeBuilder.add(fillRow);
            fillRow.clear();
        }
        return new Matrix(transposeBuilder.stream().map(u->u.stream().mapToDouble(i->i).toArray()).toArray(double[][]::new));
    }

}