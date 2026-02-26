package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.AprilTag;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.utilities.MathFunctions;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;


@Autonomous
@Config
public class AutonBack extends OpMode {

    // Create variables
    Robot robot = new Robot();

    ElapsedTime timer = new ElapsedTime();

    public static double[][] path = {
            {10, 0, 0},
    };


    Pose2D pose;

    MultipleTelemetry telemetryA;

    //the robot width and r are just used for drawing the robot on FTC Dashboard
    final double robotWidth = 15;
    final double r = (robotWidth/2) * Math.sqrt(2);

    FtcDashboard dashboard;
    boolean actionRunning = true;

    public enum side {BLUE, RED}
    public DriveOpMode.side currentSide = DriveOpMode.side.BLUE;

    public int shooter_target;

    public VectorF targetAprilTagPos;

    Action autoAction;

    double targetBearing = 0;
    double turretBearing = 0;
    public final double SPIN_MOTOR_TPR = 537.7;
    public final double SPIN_GEAR_RATIO = 180 / 49.5;
    AprilTag aprilTag = new AprilTag();

    public double shooterVel;
    public double hoodPos;


// Create a new Builder

    @Override
    public void init() {
        robot.init(hardwareMap, timer);

        dashboard = FtcDashboard.getInstance();
        //this sends stuff to both Driver Station and ftc dashboard, for convenience
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        autoAction = new SequentialAction(
                robot.PIDtoPt(path[0], 0.1, 2),
                robot.stop()
        );

        if (currentSide == DriveOpMode.side.BLUE) {
            shooter_target = 20;
        } else {
            shooter_target = 24;
        }

        aprilTag.init(hardwareMap, telemetry);

        //get the location of the aprilTag on the field
        targetAprilTagPos = AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(shooter_target).fieldPosition;

    }

    @Override
    public void start() {
        robot.odometry.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS,0));
    }

    @Override
    public void loop() {
        robot.odometry.update();
        pose = robot.odometry.getPosition();
        turretBearing = 360 * ((robot.spin.getCurrentPosition() / SPIN_MOTOR_TPR) / SPIN_GEAR_RATIO);

        robot.aim.setPosition(0.81);

        TelemetryPacket packet = new TelemetryPacket();

        //this is a little wack cause the ftc field coordinates are super different and weird
        //also, Math.atan2 accepts (y, x) <--- IMPORTANT that its not (x,y)
        double robotToGoalAngle = Math.atan2((-targetAprilTagPos.get(0)) - pose.getY(DistanceUnit.INCH), targetAprilTagPos.get(1) - pose.getX(DistanceUnit.INCH));
        //subtract robotToGoalAngle since its from x axis
        targetBearing = 0;

        if (targetBearing < Robot.TURRET_LIMITS[0]) {
            targetBearing = 360 + targetBearing;
        }
        if (targetBearing > Robot.TURRET_LIMITS[1]) {
            targetBearing = -360 + targetBearing;
        }

        double bearingError = targetBearing - turretBearing;
        if (Math.abs(bearingError) > 2) {
            robot.spin.setPower(0.015 * bearingError);
        } else {
            robot.spin.setPower(0);
        }


        //AprilTag Detection: update target location
        aprilTag.update();
        //goal is the actual apriltag
        AprilTagDetection goal = aprilTag.getTagByID(shooter_target);
        if (goal != null && goal.ftcPose != null) {
            telemetryA.addLine("aprilTag found!!");

            //convert the lens pose to the robot's pose
            pose = Robot.cameraPoseCalc(goal.robotPose, turretBearing);
            robot.odometry.setPosition(pose);

        }

        if(actionRunning){
            actionRunning = autoAction.run(packet);
        }

        dashboard.sendTelemetryPacket(packet);

        telemetryA.addData("x" , pose.getX(DistanceUnit.INCH));
        telemetryA.addData("y" , pose.getY(DistanceUnit.INCH));
        telemetryA.addData("rotation" , pose.getHeading(AngleUnit.RADIANS));
        telemetryA.addData("action running" , autoAction.run(packet));
        telemetryA.update();

    }

    @Override
    public void stop() {
        Global.pose = pose;
        Global.turretBearing = turretBearing;
        super.stop();
    }
}

