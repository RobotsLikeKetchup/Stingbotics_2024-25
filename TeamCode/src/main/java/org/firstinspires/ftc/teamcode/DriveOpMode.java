package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveOpMode extends OpMode {
    DcMotor frontLeft,frontRight,backLeft,backRight;
    double[] motorPowers;
    DcMotor[] driveMotors = {frontLeft, frontRight, backLeft, backRight};
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        for (DcMotor i : driveMotors) i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {
        motorPowers = MecanumKinematics.getPowerFromPowerAndDirection(new double[] {
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                toInt(gamepad1.left_bumper) - toInt(gamepad1.right_bumper)
        });
        for (int i=0; i < motorPowers.length; i++) {
            driveMotors[i].setPower(motorPowers[i]);
        };
    }

}
