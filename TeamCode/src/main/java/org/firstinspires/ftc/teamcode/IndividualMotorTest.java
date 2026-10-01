package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
@TeleOp
public class IndividualMotorTest extends OpMode {
    Robot robot =  new Robot();
    ElapsedTime timer = new ElapsedTime();

    private DcMotor frontLeft;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class,"motor_fl");
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
            robot.backRight.setPower(0.8);
        } else robot.backRight.setPower(0);
    }
}
