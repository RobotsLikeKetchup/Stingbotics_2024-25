package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.EzraLocalizer;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.PurePursuit;
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
    EzraLocalizer localization = new EzraLocalizer(robot, new double[] {0,0,0}, timer);
    double[][] path = {
            {0,0},
            {-1,4},
            {-3,8},
            {5, 10}
    };

    PurePursuit pathing = new PurePursuit(path, localization);

    double[] goalPoint, motorPowers, direction;

    PID velocityControl = new PID(0, 0, 0, timer);

    MotionProfile1D motionProfile = new MotionProfile1D(0.9, 0.3, timer);

    double velocityCoeff, currentVelocity;

    @Override
    public void init() {
        robot.init(hardwareMap);

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void start() {
        localization.start();
        motionProfile.startSpeedUp();
    }

    @Override
    public void loop() {
        localization.calcPose();
        goalPoint = pathing.findPointOnPath();
        direction = MovementFunctions.createMovementVector(localization.getPose(), goalPoint);

        velocityCoeff = velocityControl.loop(motionProfile.getTargetSpeed(), Math.hypot(localization.getVelocity()[0], localization.getVelocity()[1]));

        motorPowers = MecanumKinematics.getPowerFromDirection(MathFunctions.scaleArray(velocityCoeff,direction));

        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        };

        if(pathing.getLastFoundIndex() >= path.length - 2) {
            motionProfile.startSlowDown();
        }

    }
}
