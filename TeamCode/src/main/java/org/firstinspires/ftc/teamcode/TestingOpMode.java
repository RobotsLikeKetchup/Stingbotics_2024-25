package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;

import java.util.Arrays;

@TeleOp
public class TestingOpMode extends OpMode {
    double[] motorPowers;
    RoadrunnerThreeWheelLocalizer localization;

    Robot robot = new Robot();

    ElapsedTime timer= new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);


        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap);
        localization.pose = new Pose2d(0 ,0, Math.PI / 2);

        robot.init(hardwareMap);

        telemetry.addLine("initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {
        localization.updatePoseEstimate();
        motorPowers = MecanumKinematics.getPowerFromDirection(new double[] {
                gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                toInt(gamepad1.right_bumper) - toInt(gamepad1.left_bumper)
        }, 0.7);

        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        }


        //if(gamepad1.a){robot.backLeft.setPower(0.8);} else {robot.backLeft.setPower(0);}
        //if(gamepad1.b){robot.backRight.setPower(0.8);} else {robot.backRight.setPower(0);}
       // if(gamepad1.x){robot.frontLeft.setPower(0.8);} else {robot.frontLeft.setPower(0);}
        //if(gamepad1.y){robot.frontRight.setPower(0.8);} else {robot.frontRight.setPower(0);}

        robot.armRotate.setPower(gamepad1.b ? 0.8 : (gamepad1.x ? -0.8 : 0));


        if(gamepad1.dpad_down) {
            robot.intakeElbow.setPosition(-0.2);//test out to find correct position
        }
        if(gamepad1.dpad_up) {
            robot.intakeElbow.setPosition(0.2);//test out to find correct position
        }

        //int encoderValue = robot.driveMotors[3].getCurrentPosition();
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.addData("right stick y", gamepad1.right_stick_y);

        //telemetry.addLine("Pose" + Arrays.toString(localization.getPose()));

        telemetry.addLine("Encoder Positions: " + Arrays.toString(new double[]{robot.backRight.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition()}));

        telemetry.addData("Claw position", robot.intakeClaw.getPosition());
        telemetry.addData("Pose", localization.getPose());
        telemetry.addData("Arm Rotation", robot.armRotate.getCurrentPosition());


        telemetry.addData("x", localization.getPose()[0]);
        telemetry.addData("y", localization.getPose()[1]);
        telemetry.addData("angle", Math.toDegrees(localization.getPose()[2]));
        //telemetry.addData("encoder value", encoderValue);
        telemetry.update();
    }

}
