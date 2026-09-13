package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class test extends OpMode {

    public DcMotor shooter;



    @Override
    public void init(){

        shooter = hardwareMap.get(DcMotor.class, "motor_bl");



    }

    @Override
    public void loop() {

        shooter.setPower(0.8);

    }
}
