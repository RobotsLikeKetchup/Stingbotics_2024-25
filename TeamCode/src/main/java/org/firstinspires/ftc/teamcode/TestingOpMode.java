package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.PIDF;

import java.util.Arrays;


@TeleOp
@Config
public class TestingOpMode extends OpMode {
    double[] motorPowers;
    RoadrunnerThreeWheelLocalizer localization;

    Robot robot = new Robot();

    ElapsedTime timer= new ElapsedTime();

    FtcDashboard dashboard;

    public final double SPIN_MOTOR_TPR = 537.7;
    public final double SPIN_GEAR_RATIO = 180/49.5;

    final double robotWidth = 15;
    final double r = (robotWidth/2) * Math.sqrt(2);

    MultipleTelemetry telemetryA;

    public double turretBearing = 0;

    public static double kP = 0.0055;
    public static double kI = 0.00000023;
    public static double kD = 0.012;
    public static double kF = 0.000012;

    PIDF shooterpid = new PIDF(kP,kI,kD,kF, timer);
    public double shooterPower = 0;

    public static double shooterRef = 0;
    public static double targetBearing = 0;

    public static double hoodpos = 0.5;

    public DriveOpMode.state intCopy = DriveOpMode.state.OFF;

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
        shooterpid.setConstants(kP, kI, kD, kF);

        localization.updatePoseEstimate();
        motorPowers = MecanumKinematics.getPowerFromDirection(new double[] {
                gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                toInt(gamepad1.right_bumper) - toInt(gamepad1.left_bumper)
        }, 0.7);

        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        }
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        turretBearing = 360 * ((robot.spin.getCurrentPosition() / SPIN_MOTOR_TPR)/SPIN_GEAR_RATIO);

        if(currentGamepad1.y && !previousGamepad1.y){
            if(intCopy == DriveOpMode.state.OFF){
                robot.intake.setPower(.8);
                intCopy = DriveOpMode.state.ON;
            }
            else{
                robot.intake.setPower(0);
                intCopy = DriveOpMode.state.OFF;
            }
        }

        double[] pose = localization.getPose();

        if(shooterRef != 0) {
            shooterPower = shooterpid.loop(shooterRef, robot.shooter.getVelocity());
        } else shooterPower = 0;

        robot.shooter.setPower(shooterPower);
        
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        double ayush = robot.aim.getPosition();
        robot.aim.setPosition(hoodpos);

        double bearingError = targetBearing - turretBearing;
        if(Math.abs(bearingError) > 2) {
            robot.spin.setPower(0.03 * bearingError);
        } else {
            robot.spin.setPower(0);
        }

        //int encoderValue = robot.driveMotors[3].getCurrentPosition();
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.addData("right stick y", gamepad1.right_stick_y);

        //telemetry.addLine("Pose" + Arrays.toString(localization.getPose()));

        telemetryA.addLine("Encoder Positions: " + Arrays.toString(new double[]{robot.backRight.getCurrentPosition(), robot.frontLeft.getCurrentPosition(), robot.backLeft.getCurrentPosition()}));

        telemetryA.addData("x", localization.getPose()[0]);
        telemetryA.addData("y", localization.getPose()[1]);
        telemetryA.addData("angle", Math.toDegrees(localization.getPose()[2]));
        telemetryA.addData("shooter", robot.shooter.getVelocity());
        telemetryA.addData("shooterpower", shooterPower);
        telemetryA.addData("aim position", robot.aim.getPosition());
        telemetryA.addData("aim target", ayush);
        telemetryA.addData("our bearing", turretBearing);
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



        // FRICK EZRA- AYUSH BARUA



    }

}
