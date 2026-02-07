// Import FTC package
package org.firstinspires.ftc.teamcode;
// Import FTC classes

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import custom-made classes/methods
import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.hardware.AprilTag;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.MathFunctions;
import org.firstinspires.ftc.teamcode.utilities.PIDF;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;


@TeleOp

@Config
public class DriveOpMode extends OpMode {
    // Create variables
    double[] motorPowers;
    //ur nt shkspr vro -sebastian
    Robot robot = new Robot();
    //dame un grr un que -sebastian
    ElapsedTime timer = new ElapsedTime();

    //initial position of robot: MAKE SURE TO CHANGE FOR COMP
    Pose2d pose = new Pose2d(0, 0, 0);


    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();

    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    public final double SPIN_MOTOR_TPR = 537.7;
    public final double SPIN_GEAR_RATIO = 180 / 49.5;
    public final double TURRET_RADIUS = 6.49;
    public double turretBearing = 0;
    PIDF shooterpid = new PIDF(Robot.shooterConstants, timer);

    MotionProfile1D rampFunction = new MotionProfile1D(0.8, 1, 0.4, timer);

    // list of colors and variables
    public enum colors {PURPLE, GREEN, UNKNOWN}

    public enum side {BLUE, RED}

    public side currentSide = side.BLUE;

    public enum state {
        ON,
        OFF,
        REVERSE
    }

    //I am NOT unemployed

    colors detectedColor = colors.UNKNOWN;

    public state shooter = state.OFF;
    public state autoAim = state.ON;


    FtcDashboard dashboard;
    TelemetryPacket packet;
    public int shooter_target;

    public double autoSpeed = -1500;

    public double targetBearing = 0;
    public double target_range = 30;
    public double prev_target_range;


    double shooterVelocity = 0;
    double target_spin = -1900;
    double target_aim = 0.78;
    double aprilTagBearingError = 0;

    double[] cameraPos = {0, 0};

    AprilTag aprilTag = new AprilTag();

    MultipleTelemetry telemetryA;

    VectorF targetAprilTagPos;

    double prevTurret = 0;

    @Override
    // Set starting values for variable
    public void init() {
        robot.init(hardwareMap, timer);
        robot.aim.setPosition(0.95);
        robot.localization.setPose(pose);

        dashboard = FtcDashboard.getInstance();

        telemetryA = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        telemetry.update();
        aprilTag.init(hardwareMap, telemetry);

        if (currentSide == side.BLUE) {
            shooter_target = 20;
        } else {
            shooter_target = 24;
        }

        //get the location of the aprilTag on the field
        targetAprilTagPos = AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(shooter_target).fieldPosition;

        if(Global.pose != null) {
            pose = Global.pose;
            robot.localization.setPose(pose);
        }
        if(Global.turretBearing != null) {
            prevTurret = Global.turretBearing;
        }
    }

    /*@Override
    public void start() {
        localization.pose
    } */

    @Override
    public void loop() {
        //all this stuff MUST be at the beginning of the loop
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
        prev_target_range = target_range;
        double test = robot.ballStop.getPosition();
        robot.localization.update();
        pose = robot.localization.getPose();

        turretBearing = (360 * ((robot.spin.getCurrentPosition() / SPIN_MOTOR_TPR) / SPIN_GEAR_RATIO)) + prevTurret;


        //toggle shooter
        if (gamepad1.xWasPressed()) {
            if (shooter != state.ON) {
                shooter = state.ON;
            } else if (shooter != state.OFF) {
                robot.shooter.setPower(0);
                shooter = state.OFF;
                //robot.ballStop.setPosition(1);
            }
        }
        if (gamepad1.bWasPressed()) {
            if (shooter != state.REVERSE) {
                shooter = state.REVERSE;
            } else if (shooter != state.OFF) {
                shooter = state.OFF;
            }
        }

        //setting the target velocity
        if (shooter == state.ON) shooterVelocity = autoSpeed;
        else if (shooter == state.OFF) shooterVelocity = 0;
        else if (shooter == state.REVERSE) shooterVelocity = 900;

        //enacting the velocity
        if (shooterVelocity != 0) {
            robot.shooter.setPower(shooterpid.loop(shooterVelocity, robot.shooter.getVelocity()));
        } else robot.shooter.setPower(0);

        //setting intake power
        if (currentGamepad1.y) {
            robot.intake.setPower(.8);
        } else if (currentGamepad1.a) {
            robot.intake.setPower(-.6);
        } else {
            robot.intake.setPower(0);
        }
        //hood angle
        double hoodAngle = robot.aim.getPosition();

        double[] tempPose;


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


        //toggling autoAim
        if (gamepad1.dpadUpWasPressed()) {
            if (autoAim == state.ON) {
                autoAim = state.OFF;
            } else {
                autoAim = state.ON;
            }
        }


        //ramp  function: if sebastian has just started moving, beat his ass.
        //it basically prevents jerky movement by making sure it speeds up slower.
        // This way also if the driver is making precise adjustments they can go slowly
        if ((Math.abs(currentGamepad1.left_stick_x) > 0.05 || Math.abs(currentGamepad1.left_stick_y) > 0.05) && !(Math.abs(previousGamepad1.left_stick_x) > 0.05 || Math.abs(previousGamepad1.left_stick_y) > 0.05)) {
            rampFunction.reset();
        }

        if(autoAim == state.ON) {
            //this is a little wack cause the ftc field coordinates are super different and weird
            //also, Math.atan2 accepts (y, x) <--- IMPORTANT that its not (x,y)
            double robotToGoalAngle = Math.atan2((-targetAprilTagPos.get(0)) - pose.position.y, targetAprilTagPos.get(1) - pose.position.x);
            double distanceFromGoal = Math.hypot((-targetAprilTagPos.get(0)) - pose.position.y, targetAprilTagPos.get(1) - pose.position.x);
            //subtract robotToGoalAngle since its from x axis
            targetBearing = Math.toDegrees(robotToGoalAngle - MathFunctions.angleWrap(pose.heading.toDouble())) - 5; //5 degree offset for camera lens

            //picking where to shoot aim and bearing
            for (double[] item : Robot.lookup) {
                if (item[0] >= distanceFromGoal) {
                    autoSpeed = item[1];
                    target_aim = item[2];
                    break;
                }
            }
        } else{
            targetBearing = 0;
            autoSpeed = -1500;
            target_aim = 0.7;
        }

        robot.aim.setPosition(target_aim);


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


        // Gets power levels for each motor, using gamepad inputs as directions
        // The third item in the array dictates which trigger is being pressed (=1 if left, =-1 if right, =0 if none or both).
        motorPowers = MecanumKinematics.getPowerFromDirection(new double[]{
                        -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                        -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                        -(toInt(gamepad1.right_bumper) - toInt(gamepad1.left_bumper))
                },
                rampFunction.getTargetSpeed()
        );

        telemetry.addData("gamepadx", currentGamepad1.left_stick_x);
        telemetry.addData("gamepady", currentGamepad1.left_stick_y);
        // Sets power levels
        // Works because each index corresponds with the same wheel in both arrays
        for (int i = 0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        }

        telemetry.addData("Encoder R", robot.getDeadwheel("parR").getTicks());
        telemetry.addData("Encoder L", robot.getDeadwheel("parL").getTicks());
        telemetry.addData("Encoder perp", robot.getDeadwheel("per").getTicks());

        telemetry.addData("frontLeft", robot.frontLeft.getPower());
        telemetry.addData("backLeft", robot.backLeft.getPower());
        telemetry.addData("frontRight", robot.frontRight.getPower());
        telemetry.addData("backRight", robot.backRight.getPower());

        telemetry.addData("range", target_range);

        telemetry.addData("shooter", robot.shooter.getVelocity());
        telemetry.addData("taim", target_aim);
        telemetryA.addData("ourbearing", turretBearing);
        telemetryA.addData("prev(auton) bearing", prevTurret);
        telemetryA.addData("x", pose.position.x);
        telemetryA.addData("y", pose.position.y);
        telemetryA.addData("angle", MathFunctions.angleWrap(pose.heading.toDouble()));

        telemetry.addData("number", robot.ballStop.getPosition());
        telemetry.addData("aimpos", robot.aim.getPosition());

        //These things MUST be at the end of each loop. DO NOT MOVE
        telemetry.update();
        telemetryA.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
