package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

//combines two motors into one, for things like two motors driving a shooter or arm
public class DcMotorCombined{
    double power = 0;
    double velocity;

    DcMotorEx motor1;
    DcMotorEx motor2;

    public DcMotorCombined(DcMotorEx motor1, DcMotorEx motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
    }

    public double getPower() {
        return power;
    }
    public void setPower(double power) {
        this.power = power;
        motor1.setPower(power);
        motor2.setPower(power);
    }

    //gets whichever velocity is larger, in case one of them has no encoder
    public double getVelocity() {
        if (Math.abs(motor1.getVelocity()) > Math.abs(motor2.getVelocity())) {
            velocity = motor1.getVelocity();
        } else {
            velocity = motor2.getVelocity();
        }
        return velocity;
    }
}
