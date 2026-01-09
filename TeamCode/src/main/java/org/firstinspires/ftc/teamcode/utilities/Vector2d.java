package org.firstinspires.ftc.teamcode.utilities;

public class Vector2d {
    public double x;
    public double y;

    public Vector2d(double x, double y){
        this.x=x;
        this.y=y;
    }

    public Vector2d rotateBy(double angle){
         return new Vector2d(x*Math.cos(angle)-y*Math.sin(angle), x*Math.sin(angle)+y*Math.cos(angle));
    }
}
