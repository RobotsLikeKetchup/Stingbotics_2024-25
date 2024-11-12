package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp
public class TestingOpMode extends OpMode {
    double[] motorPowers;

    Robot robot = new Robot();

    ElapsedTime timer= new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addLine("initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {
        if(gamepad1.a){robot.backLeft.setPower(0.8);} else {robot.backLeft.setPower(0);}
        if(gamepad1.b){robot.backRight.setPower(0.8);} else {robot.backRight.setPower(0);}
        if(gamepad1.x){robot.frontLeft.setPower(0.8);} else {robot.frontLeft.setPower(0);}
        if(gamepad1.y){robot.frontRight.setPower(0.8);} else {robot.frontRight.setPower(0);}

        //int encoderValue = robot.driveMotors[3].getCurrentPosition();
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.addData("right stick y", gamepad1.right_stick_y);

        //telemetry.addData("encoder value", encoderValue);
        telemetry.update();
    }

}
