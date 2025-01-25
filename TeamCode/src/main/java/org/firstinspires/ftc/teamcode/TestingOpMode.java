package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    FtcDashboard dashboard;

    final double robotWidth = 15;
    final double r = (robotWidth/2) * Math.sqrt(2);

    @Override
    public void init() {
        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();


        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap, new Pose2d(0 ,0, Math.PI / 2));

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

        double[] pose = localization.getPose();


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

        //Draw on the field view of FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                .setFill("blue")
                .fillPolygon(new double[] {
                        pose[0] - (r * Math.sin(pose[2]+(Math.PI/4))),
                        pose[0] - (r * Math.sin(pose[2]-(Math.PI/4))),
                        pose[0] - (r * Math.sin(pose[2]-(3*Math.PI/4))),
                        pose[0] - (r * Math.sin(pose[2]+(3*Math.PI/4)))
                }, new double[] {
                        pose[1] - (r*Math.cos(pose[2]+(Math.PI/4))),
                        pose[1] - (r*Math.cos(pose[2]-(Math.PI/4))),
                        pose[1] - (r*Math.cos(pose[2]-(3*Math.PI/4))),
                        pose[1] - (r*Math.cos(pose[2]+(3*Math.PI/4)))
                }); //all the stuff above are the points, rotated based on the robot's angle


        dashboard.sendTelemetryPacket(packet);
    }

}
