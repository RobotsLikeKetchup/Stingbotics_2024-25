package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.PIDF;


@Autonomous
public class AutonTesting extends OpMode {
    enum Steps {
        ACCELERATION,
        DRIVE,
        DECELERATION
    };

    // Create variables
    Robot robot = new Robot();

    ElapsedTime timer = new ElapsedTime();
    RoadrunnerThreeWheelLocalizer localization;
    double[][] path = {
            {0,0},
            {0,20},
            {20,20},
            {20,0}
    };


    double[] pose = {0,0, Math.PI/2};

    PIDF velocityControl = new PIDF(0.01, 0, 0, timer);

    MotionProfile1D motionProfile = new MotionProfile1D(0.8, 0.4, timer);

    double velocityCoeff;

    MultipleTelemetry telemetryA;

    enum Stages {DRIVING, ARM_UP, ARM_DOWN, RELEASE, COMPLETE};

    Stages currentStage = Stages.DRIVING;

    //the robot width and r are just used for drawing the robot on FTC Dashboard
    final double robotWidth = 15;
    final double r = (robotWidth/2) * Math.sqrt(2);

    int pathNumber = 1;
    boolean endingPath = false;

    FtcDashboard dashboard;
    boolean actionRunning = true;

    Action autoAction;
// Create a new Builder

    @Override
    public void init() {
        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap, new Pose2d(0 ,0, Math.PI / 2));

        robot.init(hardwareMap, timer);

        dashboard = FtcDashboard.getInstance();
        //this sends stuff to both Driver Station and ftc dashboard, for convenience
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        autoAction = new SequentialAction(
                robot.PIDtoPt(new double[]{0,0,1}, 0.1, 0.2)
        );
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        localization.updatePoseEstimate();
        pose = localization.getPoseDouble();

        TelemetryPacket packet = new TelemetryPacket();

        if(actionRunning){
            actionRunning = autoAction.run(packet);

        }

        dashboard.sendTelemetryPacket(packet);

        telemetryA.addData("x: " , pose[0]);
        telemetryA.addData("y: " , pose[1]);
        telemetryA.addData("rotation: " , pose[2]);
        telemetryA.addData("action running" , autoAction.run(packet));
        telemetryA.update();

    }


}

