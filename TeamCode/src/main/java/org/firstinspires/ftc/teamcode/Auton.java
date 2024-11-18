package org.firstinspires.ftc.teamcode;

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
            {0, 20},
            {-24,20},
            {-38, 17}
    };

    PurePursuit pathing;

    double[] goalPoint, motorPowers, direction;

    PID velocityControl = new PID(0, 0, 0, timer);

    MotionProfile1D motionProfile = new MotionProfile1D(0.9, 0.3, timer);

    double velocityCoeff, currentVelocity;

    enum STAGES {START};

    @Override
    public void init() {
        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap, 0.072018);

        robot.init(hardwareMap);

        robot.intakeElbow.setPosition(0.75);

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void start() {
        motionProfile.startSpeedUp();
        localization.pose = new Pose2d(0 ,0, Math.PI / 2);
        pathing = new PurePursuit(path, localization);
    }

    @Override
    public void loop() {
        localization.updatePoseEstimate();
        goalPoint = pathing.findPointOnPath();
        direction = MovementFunctions.createMovementVector(localization.getPose(), goalPoint);

        velocityCoeff = velocityControl.loop(motionProfile.getTargetSpeed(), Math.hypot(localization.getVelocity()[0], localization.getVelocity()[1]));

        motorPowers = MecanumKinematics.getPowerFromDirection(MathFunctions.scaleArray(velocityCoeff,direction), 0.7);

        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        };

        if(pathing.getLastFoundIndex() >= path.length - 2) {
            motionProfile.startSlowDown();
        }

        if (motionProfile.currentPhase == MotionProfile1D.Phase.STOPPED) {
            robot.intakeRoller.setPower(-1);
        }


    }
}
