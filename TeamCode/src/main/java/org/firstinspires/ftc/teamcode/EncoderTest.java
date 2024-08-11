package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderTest extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class,"motor_fl");
        frontRight = hardwareMap.get(DcMotor.class, "motor_fr");
        backLeft = hardwareMap.get(DcMotor.class, "motor_bl");
        backRight = hardwareMap.get(DcMotor.class, "motor_br");

        telemetry.addLine("initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {
        int encoderValue = backRight.getCurrentPosition();
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.addData("right stick y", gamepad1.right_stick_y);

        telemetry.addData("encoder value", encoderValue);
        telemetry.update();
    }

}
