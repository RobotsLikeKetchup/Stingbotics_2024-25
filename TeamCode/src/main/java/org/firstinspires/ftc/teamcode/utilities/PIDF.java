package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    public double kP, kI, kD, kF, error, lastError, integralSum, derivative;
    ElapsedTime timer;

    public PIDF(double kP, double kI, double kD, ElapsedTime timer){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
        this.timer = timer;

        lastError = 0;
    }
    public PIDF(double kP, double kI, double kD, double kF, ElapsedTime timer){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.timer = timer;

        lastError = 0;
    }

    public double loop(double reference, double state){
        error = reference - state;

        derivative = (error - lastError) / timer.seconds();

        integralSum += error * timer.seconds();

        return (kP * error) + (kD * derivative) + (kI * integralSum) + (kF * reference);
    }

    public void setConstants(double P, double I, double D, double F) {
        kP = P;
        kI = I;
        kD = D;
        kF = F;
    }

}
