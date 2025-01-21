package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.EzraLocalizer;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.MathFunctions;
import org.firstinspires.ftc.teamcode.utilities.MovementFunctions;
import org.firstinspires.ftc.teamcode.utilities.PID;


@Autonomous
public class Auton extends OpMode {
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
            {0, 20}
    };

    PurePursuit pathing;

    double[] goalPoint, motorPowers, direction;

    PID velocityControl = new PID(0, 0, 0, timer);

    MotionProfile1D motionProfile = new MotionProfile1D(0.5, 0.2, timer);

    double velocityCoeff, currentVelocity;

    //MultipleTelemetry telemetry;

    enum STAGES {START};

    @Override
    public void init() {
        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap);

        robot.init(hardwareMap);

        robot.intakeElbow.setPosition(0.75);

        //this sends stuff to both Driver Station and ftc dashboard, for convenience
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void start() {
        motionProfile.startSpeedUp();
        localization.pose = new Pose2d(0 ,0, 0);
        pathing = new PurePursuit(path, localization);

    }

    @Override
    public void loop() {
        localization.updatePoseEstimate();
        goalPoint = pathing.findPointOnPath();
        direction = MovementFunctions.createMovementVector(localization.getPose(), goalPoint);

        //changes the angle to a simple "direction" for the robot to turn in(negative is right, positive is left)
        if(direction[2] <= 0.1 && direction[2] >= -.1){
            direction[2] = 0;
        } else if(direction[2] > 0.1) {
            direction[2] = 0.5;
        } else if(direction[2] < -0.1) {
            direction[2] = -0.5;
        }

        velocityCoeff = velocityControl.loop(motionProfile.getTargetSpeed(), Math.hypot(localization.getVelocity()[0], localization.getVelocity()[1]));

        motorPowers = MecanumKinematics.getPowerFromDirection(direction, velocityCoeff);

        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        };

        if(pathing.getLastFoundIndex() >= path.length - 2) {
            motionProfile.startSlowDown();
        }

        if (motionProfile.currentPhase == MotionProfile1D.Phase.STOPPED) {
            robot.intakeRoller.setPower(-1);
        }



        //send telemetry to rc phone/driver station and/or ftc dashboard
        telemetry.addData("x", localization.getPose()[0]);
        telemetry.addData("y", localization.getPose()[1]);
        telemetry.addData("angle", Math.toDegrees(localization.getPose()[2]));
        telemetry.addData("targetPoint-x", goalPoint[0]);
        telemetry.addData("targetPoint-y", goalPoint[1]);
        telemetry.addData("speed", motionProfile.getTargetSpeed());
        telemetry.addData("direction x", direction[0]);
        telemetry.addData("direction y", direction[1]);
        telemetry.addData("direction angle", Math.toDegrees(direction[2]));

        telemetry.update();
    }
}
