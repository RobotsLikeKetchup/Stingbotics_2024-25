package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DeadWheel {
    //fields
    float radius; // in cm
    DcMotor encoder;
    float tpr; //ticks per revolution, depends on the encoder we use

    double inPerTick;

    DcMotorSimple.Direction direction;

    //constructor
    public DeadWheel(float wheelRadius, DcMotor motor, float encoderTpr){
        radius = wheelRadius; //"effective" radius might not be the same as what manufacturer says
        encoder = motor;
        tpr = encoderTpr;

        direction = DcMotorSimple.Direction.FORWARD;
    }

    public DeadWheel(double inPerTick, DcMotor motor){
        this.inPerTick = inPerTick;
        encoder =  motor;

        direction = DcMotorSimple.Direction.FORWARD;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.direction = direction;
    }

    int applyDirection(){
        int x;
        if(direction == DcMotorSimple.Direction.REVERSE) {
            x = -1;
        } else {
            x = 1;
        }
        return x;
    }

    //methods
    public double getDistanceFromRadius() {
        //all measurements are in cm
        float inputVal = encoder.getCurrentPosition();
        float revolutions = inputVal/tpr;
        return applyDirection()*revolutions*2*Math.PI*radius;
    }
    public double getTicks() {
        return applyDirection()*encoder.getCurrentPosition();
    }
    public double getDistanceFromInPerTick() {
        return applyDirection()*getTicks() * inPerTick;
    }
    public void reset(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}