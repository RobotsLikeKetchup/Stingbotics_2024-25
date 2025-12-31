// Import FTC package
package org.firstinspires.ftc.teamcode;
// Import FTC classes
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import custom-made classes/methods
import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import org.firstinspires.ftc.teamcode.hardware.AprilTag;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.PIDF;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;


@TeleOp

@Config
public class DriveOpMode extends OpMode {
    // Create variables
    double[] motorPowers;
//ur nt shkspr vro -sebastian
    Robot robot = new Robot();
    //dame un grr un que -sebastian
    ElapsedTime timer= new ElapsedTime();

    RoadrunnerThreeWheelLocalizer localizer;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();

    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    public static double kP = 0.0055;
    public static double kI = 0.00000023;
    public static double kD = 0.012;
    public static double kF = 0.000012;

    PIDF shooterpid = new PIDF(kP,kI,kD,kF, timer);

    MotionProfile1D rampFunction = new MotionProfile1D(0.8,1, 0.4, timer);
    // list of colors and variables
    public enum colors{ PURPLE, GREEN, UNKNOWN }

    public enum side { BLUE, RED }

    public side currentSide = side.BLUE;

    public enum ezraUnemployed{ //ezraUnemployed is state
        ON,
        OFF,
        REVERSE
    }

    //I am NOT unemployed

    colors detectedColor = colors.UNKNOWN;

    public ezraUnemployed shooter = ezraUnemployed.OFF;
    public ezraUnemployed intCopy = ezraUnemployed.OFF;


    FtcDashboard dashboard;
    TelemetryPacket packet;
    public int shooter_target;

    public double shooterSpeed = -1800;

    public double target_bearing;
    public double target_range = 40;
    public double prev_target_range;

    List<AprilTagDetection> aprilTagDetections;

    double shooterVelocity = 0;
    double target_spin = -1900;
    double target_aim = 0.78;


    double[][] lookup = {
            {33,-1800,0.78},
            {46,-1900,0.78},
            {58,-1950,0.64},
            {90,-2200,0.54}
    };
    AprilTag aprilTag = new AprilTag();

    @Override
    // Set starting values for variable
    public void init() {
        robot.init(hardwareMap, timer);
        //localizer = new RoadrunnerThreeWheelLocalizer(hardwareMap);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.aim.setPosition(0.5);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        //telemetry.addData("X pos",localizer.getPose()[0]);
        //telemetry.addData("y pos",localizer.getPose()[1]);
        //telemetry.addData("Angle pos",localizer.getPose()[2]);
        telemetry.update();
        aprilTag.init(hardwareMap, telemetry);

        if (currentSide == side.BLUE) {shooter_target = 20;} else {shooter_target = 24;}

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

        // gives robot loving parents
        shooterVelocity = shooterpid.loop(shooterSpeed, robot.shooter.getVelocity());

        //toggle
        if (currentGamepad1.x && !previousGamepad1.x) {
            if (shooter != ezraUnemployed.ON) {
                shooter = ezraUnemployed.ON;
            } else if (shooter != ezraUnemployed.OFF) {
                robot.shooter.setPower(0);
                shooter = ezraUnemployed.OFF;
            }
        }
        if (currentGamepad1.b && !previousGamepad1.b) {
            if (shooter != ezraUnemployed.REVERSE) {
                shooter = ezraUnemployed.REVERSE;
            } else if (shooter != ezraUnemployed.OFF) {
                shooter = ezraUnemployed.OFF;
            }
        }


        if (shooter == ezraUnemployed.ON) shooterVelocity = target_spin;
        else if (shooter == ezraUnemployed.OFF) shooterVelocity = 0;
        else if (shooter == ezraUnemployed.REVERSE) shooterVelocity = -target_spin / 2;


        if (shooterVelocity != 0) {
            robot.shooter.setPower(shooterpid.loop(shooterVelocity, robot.shooter.getVelocity()));
        } else robot.shooter.setPower(0);

        if (currentGamepad1.y && !previousGamepad1.y) {
            if (intCopy != ezraUnemployed.ON) {
                robot.intake.setPower(.8);
                intCopy = ezraUnemployed.ON;
            } else if (intCopy != ezraUnemployed.OFF) {
                robot.intake.setPower(0);
                intCopy = ezraUnemployed.OFF;
            }
        }
        if (currentGamepad1.a && !previousGamepad1.a) {
            if (intCopy != ezraUnemployed.REVERSE) {
                robot.intake.setPower(-.8);
                intCopy = ezraUnemployed.REVERSE;
            } else if (intCopy != ezraUnemployed.OFF) {
                robot.intake.setPower(0);
                intCopy = ezraUnemployed.OFF;
            }
        }

        double ayush = robot.aim.getPosition();

        robot.aim.setPosition(target_aim);
        //servo location
        if (currentGamepad1.dpad_up) {
            ayush -= 0.05;
        }
        if (currentGamepad1.dpad_down) {
            ayush += 0.05;
        }
        robot.aim.setPosition(ayush);
        if (ayush > 1) {
            robot.aim.setPosition(1);
        }
        if (ayush < 0.4) {
            robot.aim.setPosition(0.4);
        }
//manual spin
        //if (currentGamepad1.dpad_left) {
            //robot.spin.setPower(1);
        //} else if (currentGamepad1.dpad_right) {
            //robot.spin.setPower(-1);
        //} else {
            //robot.spin.setPower(0);
        //}
        // FRICK EZRA- AYUSH BARUA


        //totr
        double red = robot.ballColor.red();
        double green = robot.ballColor.green();
        double blue = robot.ballColor.blue();
        double finalRed = red / green;
        double finalBlue = blue / green;
        if (green != 0 && finalRed > 0.05 && finalRed < 0.5 && finalBlue > 0.6 && finalBlue < 0.9) {
            detectedColor = colors.GREEN;

        } else if (green != 0 && finalRed > 0.5 && finalRed < 1.3 && finalBlue > 1 && finalBlue < 2.8) {
            detectedColor = colors.PURPLE;
        } else {
            detectedColor = colors.UNKNOWN;
        }


        //ramp  function: if sebastian has just started moving, beat his ass.
        //it basically prevents jerky movement by making sure it speeds up slower.
        // This way also if the driver is making precise adjustments they can go slowly
        if ((Math.abs(currentGamepad1.left_stick_x) > 0.05 || Math.abs(currentGamepad1.left_stick_y) > 0.05) && !(Math.abs(previousGamepad1.left_stick_x) > 0.05 || Math.abs(previousGamepad1.left_stick_y) > 0.05)) {
            rampFunction.reset();
        }

        //update
        aprilTag.update();
        AprilTagDetection id20 = aprilTag.getTagByID(20);
        //display
        aprilTag.displayTelemetry(id20);
        if(id20 != null && id20.ftcPose != null){
            double bearingError = id20.ftcPose.bearing;
            if(bearingError > 5){
                robot.spin.setPower(0.3);
            } else if (bearingError < -5) {
                robot.spin.setPower(-0.3);
            }else {
                robot.spin.setPower(0);
            }
        }else{
            robot.spin.setPower(0);
        }

        //AprilTag Detection: update target location
        aprilTagDetections = robot.aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : aprilTagDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(" " + detection.metadata.id);
                if (detection.metadata.id == shooter_target) {
                    target_range = detection.ftcPose.range;
                }
            }
        }

        for (double[] item : lookup) {
            if (item[0] >= target_range) {
                target_spin = item[1];
                target_aim = item[2];
            }
        }



        // Gets power levels for each motor, using gamepad inputs as directions
        // The third item in the array dictates which trigger is being pressed (=1 if left, =-1 if right, =0 if none or both).
        motorPowers = MecanumKinematics.getPowerFromDirection(new double[] {
                gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                - gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                toInt(gamepad1.right_bumper) - toInt(gamepad1.left_bumper)
                },
            rampFunction.getTargetSpeed(), 1, //this all just correcting for our shitty weight distribution
            true
        );

        telemetry.addData("aimer position", ayush);
        telemetry.addData("gamepadx", currentGamepad1.left_stick_x);
        telemetry.addData("gamepady", currentGamepad1.left_stick_y);
        // Sets power levels
        // Works because each index corresponds with the same wheel in both arrays
        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        }

        telemetry.addData("Encoder R", robot.getDeadwheel("parR").getTicks());
        telemetry.addData("Encoder L", robot.getDeadwheel("parL").getTicks());
        telemetry.addData("Encoder perp", robot.getDeadwheel("per").getTicks());

        telemetry.addData("frontLeft", robot.frontLeft.getPower());
        telemetry.addData("backLeft", robot.backLeft.getPower());
        telemetry.addData("frontRight", robot.frontRight.getPower());
        telemetry.addData("backRight", robot.backRight.getPower());
        telemetry.addData("ballDetected", detectedColor);

        telemetry.addData("range", target_range);

        telemetry.addData("shooter", robot.shooter.getVelocity());
        telemetry.addData("taim", target_aim);
        telemetry.addData("tspin", target_spin);
        //These things MUST be at the end of each loop. DO NOT MOVE
        telemetry.update();
    }

}
