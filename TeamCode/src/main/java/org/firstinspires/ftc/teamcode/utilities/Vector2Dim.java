package org.firstinspires.ftc.teamcode.utilities;

public class Vector2Dim {
    public double x;
    public double y;

    public Vector2Dim(double x, double y){
        this.x=x;
        this.y=y;
    }

    public Vector2Dim rotateBy(double angle){
         return new Vector2Dim(x*Math.cos(angle)-y*Math.sin(angle), x*Math.sin(angle)+y*Math.cos(angle));
    }
}
