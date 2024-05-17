package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DeadWheel {
    //fields
    float radius; // in cm
    DcMotor encoder;
    float tpr; //ticks per revolution, depends on the encoder we use

    //constructor
    public DeadWheel(float wheelRadius, DcMotor motor, float encoderTpr){
        radius = wheelRadius; //"effective" radius might not be the same as what manufacturer says
        encoder = motor;
        tpr = encoderTpr;
    }

    //methods
    public double getDistance() {
        //all measurements are in cm
        float inputVal = encoder.getCurrentPosition();
        float revolutions = inputVal/tpr;
        return revolutions*2*Math.PI*radius;
    }
    public void reset(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}