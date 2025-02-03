package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;


@Autonomous
public class AutonClip extends OpMode {
    Robot robot = new Robot();
    ElapsedTime timer = new ElapsedTime();
    RoadrunnerThreeWheelLocalizer localization;
    FtcDashboard dashboard;
    MultipleTelemetry telemetryA;
    boolean actionRunning = true;

    double[][] path1 = {
            {0,0},
            {0,-28}
    };

    double[][] path2 = {
            {0,-20},
            {-30, -25}
    };

    Action autoAction;

    @Override
    public void init() {
        robot.init(hardwareMap,timer);
        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap, new Pose2d(0 ,0, Math.PI / 2));

        robot.intakeElbow.setPosition(0.80);
        robot.intakeClaw.setPosition(0.9);

        dashboard = FtcDashboard.getInstance();
        //this sends stuff to both Driver Station and ftc dashboard, for convenience
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        autoAction = new SequentialAction(
            robot.followPath(path1, Math.PI, false),
            robot.clip()
        );
    }

    @Override
    public void loop() {
        localization.updatePoseEstimate();

        TelemetryPacket packet = new TelemetryPacket();

        if(actionRunning){
            actionRunning = autoAction.run(packet);
        }

        dashboard.sendTelemetryPacket(packet);
    }
}
