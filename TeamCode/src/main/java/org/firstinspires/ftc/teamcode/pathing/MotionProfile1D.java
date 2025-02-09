package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.util.ElapsedTime;

//this motion profile ONLY generates for 1d speed, NOT velocity or position
public class MotionProfile1D {
    public enum Phase {
        STOPPED,
        SPEED_UP,
        CONSTANT_SPEED,
        SLOW_DOWN
    }

    public Phase currentPhase = Phase.STOPPED;

    double maxSpeed, maxAcceleration, currentSpeed, lastTime, currentTime, elapsedTime, startSpeed;

    ElapsedTime timer;

    public MotionProfile1D(double maxSpeed, double maxAcceleration, ElapsedTime timer){
        this.maxAcceleration = maxAcceleration;
        this.maxSpeed = maxSpeed;
        this.timer = timer;
        currentSpeed = 0;
        startSpeed = 0;
    }

    public MotionProfile1D(double startSpeed, double maxSpeed, double maxAcceleration, ElapsedTime timer){
        this.maxAcceleration = maxAcceleration;
        this.maxSpeed = maxSpeed;
        this.timer = timer;
        this.startSpeed = startSpeed;
        currentSpeed = 0;
    }

    public void startSpeedUp() {
        currentPhase = Phase.SPEED_UP;
        lastTime = timer.seconds();
        currentSpeed = startSpeed;
    }

    public void startSlowDown() {
        currentPhase = Phase.SLOW_DOWN;
    }

    public double getTargetSpeed() {
        currentTime = timer.seconds();
        elapsedTime = currentTime - lastTime;

        if(currentPhase == Phase.STOPPED) {
            currentSpeed = 0;
        }
        else if(currentPhase == Phase.SPEED_UP) {
            if(currentSpeed >= maxSpeed){
                currentPhase = Phase.CONSTANT_SPEED;
                currentSpeed = maxSpeed;
            } else {
                currentSpeed += maxAcceleration * (elapsedTime);
            }
        } else if(currentPhase == Phase.CONSTANT_SPEED){
            currentSpeed = maxSpeed;
        } else if(currentPhase == Phase.SLOW_DOWN) {
            if(currentSpeed < 0) {
                currentPhase = Phase.STOPPED;
                currentSpeed = 0;
            }
            else{
                currentSpeed = currentSpeed - (maxAcceleration * (elapsedTime));
            }
        }

        lastTime = currentTime;
        return currentSpeed;
    }

    public void reset(){
        currentSpeed = 0;
        startSpeedUp();
    }
}
