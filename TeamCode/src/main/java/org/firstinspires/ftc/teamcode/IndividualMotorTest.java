package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class IndividualMotorTest extends OpMode {
    Robot robot =  new Robot();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap, timer);
    }

    @Override
    public void loop() {
        if(gamepad1.x){
            robot.frontLeft.setPower(0.4);
        } else robot.frontLeft.setPower(0);
        if(gamepad1.a){
            robot.backLeft.setPower(0.4);
        } else robot.backLeft.setPower(0);
        if(gamepad1.y) {
            robot.frontRight.setPower(0.4);
        } else robot.frontRight.setPower(0);
        if(gamepad1.b) {
            robot.backRight.setPower(0.4);
        } else robot.backRight.setPower(0);
    }
}
