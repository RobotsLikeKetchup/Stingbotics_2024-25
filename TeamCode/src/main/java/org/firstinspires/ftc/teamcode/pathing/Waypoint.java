package org.firstinspires.ftc.teamcode.pathing;

public class Waypoint {

    public enum WaypointType {STOP, PATH, START};

    WaypointType pointType;
    double x, y, lookahead;

    public Waypoint(double x, double y, WaypointType type){
        this.x = x;
        this.y = y;
        this.pointType = type;
    }

    public Waypoint(double x, double y){
        this.x = x;
        this.y = y;
        pointType = WaypointType.PATH;
    }

}
