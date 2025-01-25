package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
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
            {0,-28}
    };

    double[][] path2 = {
            {0,-20},
            {-30, -25}
    };

    PurePursuit pathing, pathing2, currentPathing;

    double[] goalPoint, motorPowers, direction, pose;

    PID velocityControl = new PID(0.01, 0, 0, timer);

    MotionProfile1D motionProfile = new MotionProfile1D(1, 0.2, timer);

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

    @Override
    public void init() {
        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap, new Pose2d(0 ,0, Math.PI / 2));

        robot.init(hardwareMap);

        robot.intakeElbow.setPosition(0.80);
        robot.intakeClaw.setPosition(0.9);

        dashboard = FtcDashboard.getInstance();
        //this sends stuff to both Driver Station and ftc dashboard, for convenience
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

    }

    @Override
    public void start() {
        motionProfile.startSpeedUp();
        pathing = new PurePursuit(path, localization);
        pathing2 = new PurePursuit(path2, localization);


    }

    @Override
    public void loop() {
        localization.updatePoseEstimate();
        pose = localization.getPose();

        if(currentStage == Stages.DRIVING) {
            if (pathNumber == 1){
                currentPathing = pathing;
            } else if(pathNumber==2) {
                currentPathing = pathing2;
            }
            goalPoint = currentPathing.findPointOnPath();
            direction = MovementFunctions.createMovementVector(pose, goalPoint, Math.PI);


            //changes the angle to a simple "direction" for the robot to turn in(negative is right, positive is left)
            if (direction[2] <= 0.1 && direction[2] >= -.1) {
                direction[2] = 0;
            } else if (direction[2] > 0.1) {
                direction[2] = 0.5;
            } else if (direction[2] < -0.1) {
                direction[2] = -0.5;
            }

            //uses a PID to make sure the velocity stays consistent
            //velocityCoeff = velocityControl.loop(motionProfile.getTargetSpeed(), Math.hypot(localization.getVelocity()[0], localization.getVelocity()[1]));

            velocityCoeff = motionProfile.getTargetSpeed();
            motorPowers = MecanumKinematics.getPowerFromDirection(direction, velocityCoeff);

            for (int i = 0; i < motorPowers.length; i++) {
                robot.driveMotors[i].setPower(-motorPowers[i]);//negative is temporary until I figure out what is making the robot go backward
            }
            ;

            if ((currentPathing.getDistanceFromEnd() <= 8) && (motionProfile.currentPhase == MotionProfile1D.Phase.CONSTANT_SPEED || motionProfile.currentPhase == MotionProfile1D.Phase.SPEED_UP)) {
                motionProfile.startSlowDown();
                endingPath = true;
            }

            if (motionProfile.currentPhase == MotionProfile1D.Phase.STOPPED && endingPath) {
                endingPath = false;
                currentStage = Stages.ARM_UP;
            }
        }



        if(currentStage == Stages.ARM_UP) {
            robot.intakeElbow.setPosition(0.2);
            if(!robot.backArmLimitSwitch.isPressed()) {
                robot.armRotate.setPower(-0.7);
            } else {
                robot.armRotate.setPower(0);
            }
            if(robot.armExtend.getCurrentPosition() <= 4900) {
                robot.armExtend.setPower(1);
            } else {
                robot.armExtend.setPower(0);
            }
            if(robot.backArmLimitSwitch.isPressed() && (robot.armExtend.getCurrentPosition() >= 4900)) {
                currentStage = Stages.ARM_DOWN;
            }
        } else if(currentStage == Stages.ARM_DOWN) {
            if(robot.armExtend.getCurrentPosition() <= 3600) {
                currentStage = Stages.RELEASE;
            } else {
                robot.armExtend.setPower(-0.8);
            }
        } else if(currentStage == Stages.RELEASE) {
            if(robot.slideLimitSwitch.isPressed()) {robot.armExtend.setPower(0);};
            robot.intakeClaw.setPosition(0.1);
            currentStage = Stages.COMPLETE;
        } if(currentStage == Stages.COMPLETE) {
            robot.intakeElbow.setPosition(0.80);
            if(!robot.frontArmLimitSwitch.isPressed()) {
                robot.armRotate.setPower(1);
            } else {
                robot.armRotate.setPower(0);
                pathNumber++;
                motionProfile.startSpeedUp();
                currentStage = Stages.DRIVING;
            }
        }


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
                }) //all the stuff above are the points, rotated based on the robot's angle
                .strokeCircle(pose[0], pose[1], 20) //draw lookAhead radius
                .fillCircle(goalPoint[0], goalPoint[1], 1); //draw lookahead point

        dashboard.sendTelemetryPacket(packet);

        //send telemetry to rc phone/driver station and/or ftc dashboard
        telemetryA.addData("x", pose[0]);
        telemetryA.addData("y", pose[1]);
        telemetryA.addData("angle", Math.toDegrees(pose[2]));
        telemetryA.addData("targetPoint-x", goalPoint[0]);
        telemetryA.addData("targetPoint-y", goalPoint[1]);
        telemetryA.addData("speed", motionProfile.getTargetSpeed());
        telemetryA.addData("direction x", direction[0]);
        telemetryA.addData("direction y", direction[1]);
        telemetryA.addData("direction angle", Math.toDegrees(direction[2]));

        telemetryA.addData("arm extend pos", robot.armExtend.getCurrentPosition());
        telemetryA.addData("arm rotate power", robot.armRotate.getPower());
        telemetryA.addData("arm extend power", robot.armExtend.getPower());

        telemetryA.addData("stage", currentStage);
        telemetryA.addData("endingPath?", endingPath);
        telemetryA.addData("motion Profile stage", motionProfile.currentPhase);
        telemetryA.addData("path number", pathNumber);
        telemetryA.update();
    }
}
