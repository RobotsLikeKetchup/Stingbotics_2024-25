package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.angleWrap;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    public double kP, kI, kD, kF, error, lastError, integralSum, derivative;
    public Double lastTime, thisTime;
    //if this is true, its for controlling direction, or rotation, where the math is more complex
    boolean directional = false;
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
    public PIDF(double[] constants, ElapsedTime timer){
        if(constants.length == 3){
            kP = constants[0];
            kI = constants[1];
            kD = constants[2];
            kF = 0;
        }
        if(constants.length == 4){
            kP = constants[0];
            kI = constants[1];
            kD = constants[2];
            kF = constants[3];
        }

        this.timer = timer;
    }
    public PIDF(double kP, double kI, double kD, double kF, ElapsedTime timer, boolean directional){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.timer = timer;
        this.directional = directional;

        lastError = 0;
    }

    public double loop(double reference, double state){
        thisTime = timer.seconds();

        //if its the first loop, just set the previous time to the current time
        if(lastTime == null) {
            lastTime = thisTime;
        }

        if(directional == false) {
            error = reference - state;
        } else {
            //for RADIANS!!!
            error = angleWrap(reference - state);
        }

        derivative = (error - lastError) / (thisTime - lastTime);

        integralSum += error * (thisTime - lastTime);

        lastTime = thisTime;

        return (kP * error) + (kD * derivative) + (kI * integralSum) + (kF * reference);
    }

    public void setConstants(double P, double I, double D, double F) {
        kP = P;
        kI = I;
        kD = D;
        kF = F;
    }
    public void resetIntegral() {
        integralSum = 0;
    }

}
