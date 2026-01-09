package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous
public class EZAuton extends LinearOpMode {
    Robot robot = new Robot();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, timer);
        waitForStart();
        robot.aim.setPosition(0.85);
        for(DcMotor motor : robot.driveMotors) {
            motor.setPower(0.8);
        }
        sleep(2500);
        for(DcMotor motor : robot.driveMotors) {
            motor.setPower(0);
        }
    }
}
