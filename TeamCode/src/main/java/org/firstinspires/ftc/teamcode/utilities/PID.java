package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double kP, kI, kD, error, lastError, integralSum, derivative;
    ElapsedTime timer;

    public PID(double kP, double kI, double kD, ElapsedTime timer){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.timer = timer;

        lastError = 0;
    }

    public double loop(double reference, double state){
        error = reference - state;

        derivative = (error - lastError) / timer.seconds();

        integralSum += error * timer.seconds();

        return (kP * error) + (kD * derivative) + (kI * integralSum);
    }

}
