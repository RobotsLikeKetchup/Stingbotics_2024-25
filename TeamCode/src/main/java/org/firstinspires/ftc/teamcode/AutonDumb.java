package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.MathFunctions;
import org.firstinspires.ftc.teamcode.utilities.MovementFunctions;
import org.firstinspires.ftc.teamcode.utilities.PID;



//this is pretty much just for if our autonomous breaks for some reason and we want to just have the robot drive to the side and park
@Autonomous
public class AutonDumb extends OpMode {
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

        robot.init(hardwareMap, timer);

        robot.intakeElbow.setPosition(0.75);

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        motorPowers = MecanumKinematics.getPowerFromDirection(new double[] {-1,0,0}, 0.7);

        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        };



    }
}
