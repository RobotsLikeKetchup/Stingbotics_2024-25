package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.hardware.AprilTag;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.MathFunctions;
import org.firstinspires.ftc.teamcode.utilities.PIDF;
import org.firstinspires.ftc.teamcode.utilities.Vector2Dim;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;


@Autonomous
@Config
public class AutonTesting extends OpMode {

    // Create variables
    Robot robot = new Robot();

    ElapsedTime timer = new ElapsedTime();

    public static double[][] path = {
            {-28, 38, 2.51},
            {-10, 18, Math.PI},
            {-45, 18, Math .PI},
            {-28, 38, 2.51},
            {-10, -5, Math.PI},
            {-45, -5, Math.PI},
            {-28, 38, 2.51},
            {-20, 10, Math.PI}
    };


    Pose2d pose;

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
        robot.localization.setPose(new Pose2d(-50.1,63.83,2.51));

        dashboard = FtcDashboard.getInstance();
        //this sends stuff to both Driver Station and ftc dashboard, for convenience
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        autoAction = new SequentialAction(
                robot.PIDtoPt(path[0], 0.1, 1.5),
                robot.stop(),
                robot.shoot(-1400, 3),
                robot.PIDtoPt(path[1], 0.1, 4),
                robot.stop(),
                new RaceAction(
                        new ParallelAction(
                                robot.startIntake(),
                                robot.PIDtoPt(path[2], 0.1, 5, 0.5)
                        ),
                        robot.ballDown()
                ),
                robot.stop(),
                robot.PIDtoPt(path[3], 0.1, 1.5),
                robot.stop(),
                robot.shoot(-1400, 3),
                robot.PIDtoPt(path[4], 0.1, 2),
                robot.stop(),
                new RaceAction(
                        new ParallelAction(
                                robot.startIntake(),
                                robot.PIDtoPt(path[5], 0.1, 5, 0.5)
                        ),
                        robot.ballDown()
                ),
                robot.stop(),
                robot.PIDtoPt(path[6], 0.1, 1),
                robot.stop(),
                robot.shoot(-1400, 3),
                robot.PIDtoPt(path[7], 0.1, 2),
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

    }

    @Override
    public void loop() {
        robot.localization.update();
        pose = robot.localization.getPose();
        turretBearing = 360 * ((robot.spin.getCurrentPosition() / SPIN_MOTOR_TPR) / SPIN_GEAR_RATIO);

        robot.aim.setPosition(0.81);

        TelemetryPacket packet = new TelemetryPacket();

        //this is a little wack cause the ftc field coordinates are super different and weird
        //also, Math.atan2 accepts (y, x) <--- IMPORTANT that its not (x,y)
        double robotToGoalAngle = Math.atan2((-targetAprilTagPos.get(0)) - pose.position.y, targetAprilTagPos.get(1) - pose.position.x);
        //subtract robotToGoalAngle since its from x axis
        targetBearing = Math.toDegrees(robotToGoalAngle - MathFunctions.angleWrap(pose.heading.toDouble())) - 5;

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
            pose = RoadrunnerThreeWheelLocalizer.cameraToRobotPose(goal.robotPose, turretBearing);
            robot.localization.setPose(pose);

        }

        if(actionRunning){
            actionRunning = autoAction.run(packet);
        }

        dashboard.sendTelemetryPacket(packet);

        telemetryA.addData("x" , pose.position.x);
        telemetryA.addData("y" , pose.position.y);
        telemetryA.addData("rotation" , pose.heading.toDouble());
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

