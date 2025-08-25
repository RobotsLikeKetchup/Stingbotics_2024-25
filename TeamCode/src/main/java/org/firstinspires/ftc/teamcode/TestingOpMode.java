package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

    MultipleTelemetry telemetryA;

    @Override
    public void init() {

        dashboard = FtcDashboard.getInstance();

        telemetryA = new MultipleTelemetry(telemetry,dashboard.getTelemetry());

        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap, new Pose2d(0 ,0, Math.PI / 2));

        robot.init(hardwareMap, timer);

        telemetryA.addLine("initialized!");
        telemetryA.update();
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




        //int encoderValue = robot.driveMotors[3].getCurrentPosition();
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.addData("right stick y", gamepad1.right_stick_y);

        //telemetry.addLine("Pose" + Arrays.toString(localization.getPose()));

        telemetryA.addLine("Encoder Positions: " + Arrays.toString(new double[]{robot.backRight.getCurrentPosition(), robot.frontLeft.getCurrentPosition(), robot.backLeft.getCurrentPosition()}));
        
        telemetryA.addData("x", localization.getPose()[0]);
        telemetryA.addData("y", localization.getPose()[1]);
        telemetryA.addData("angle", Math.toDegrees(localization.getPose()[2]));
        //telemetry.addData("encoder value", encoderValue);
        telemetryA.update();

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
