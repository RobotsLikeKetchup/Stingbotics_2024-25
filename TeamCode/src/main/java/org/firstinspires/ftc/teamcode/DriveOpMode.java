// Import FTC package
package org.firstinspires.ftc.teamcode;
// Import FTC classes
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.utilities.Vector2d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
    double[] pose;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();

    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    public static double kP = 0.0061;
    public static double kI = 0.00000023;
    public static double kD = 0.015;
    public static double kF = 0.00015;

    public final double SPIN_MOTOR_TPR = 537.7;
    public final double SPIN_GEAR_RATIO = 180/49.5;
    public final double TURRET_RADIUS = 6.49;
    public double turretBearing = 0;
    PIDF shooterpid = new PIDF(kP,kI,kD,kF, timer);

    MotionProfile1D rampFunction = new MotionProfile1D(0.8,1, 0.4, timer);
    // list of colors and variables
    public enum colors{ PURPLE, GREEN, UNKNOWN }

    public enum side { BLUE, RED }

    public side currentSide = side.BLUE;

    public enum state {
        ON,
        OFF,
        REVERSE
    }

    //I am NOT unemployed

    colors detectedColor = colors.UNKNOWN;

    public state shooter = state.OFF;
    public state autoAim = state.OFF;


    FtcDashboard dashboard;
    TelemetryPacket packet;
    public int shooter_target;

    public double shooterSpeed = -1800;

    public double targetBearing = 0;
    public double target_range = 30;
    public double prev_target_range;


    double shooterVelocity = 0;
    double target_spin = -1900;
    double target_aim = 0.78;
    double aprilTagBearingError = 0;

    double[] cameraPos = {0,0};


    double[][] lookup = {
            {33,-1800,0.95},
            {46,-1900,0.95},
            {58,-1950,0.81},
            {78,-2200,0.71}
    };
    AprilTag aprilTag = new AprilTag();

    MultipleTelemetry telemetryA;

    @Override
    // Set starting values for variable
    public void init() {
        robot.init(hardwareMap, timer);
        robot.aim.setPosition(0.95);

        dashboard = FtcDashboard.getInstance();

        telemetryA = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
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
        //could be hard coded
        turretBearing = 360 * ((robot.spin.getCurrentPosition() / SPIN_MOTOR_TPR)/SPIN_GEAR_RATIO);

        //calculate the position of the camera from the straight-forward position
        cameraPos[0] = TURRET_RADIUS*Math.sin(-Math.toRadians(turretBearing));
        cameraPos[1] = TURRET_RADIUS*(-1+Math.cos(Math.toRadians(turretBearing)));

        //calculating distance between cameraPosition and center of rotation, done with calculations on the robot
        double tempX = cameraPos[0] + 4.62 - 3.75 - (RoadrunnerThreeWheelLocalizer.PARAMS.perpXTicks * RoadrunnerThreeWheelLocalizer.inPerTick);
        double tempY = RoadrunnerThreeWheelLocalizer.PARAMS.par1YTicks * RoadrunnerThreeWheelLocalizer.inPerTick;

        Vector2d tempsubtract = new Vector2d(tempX, tempY);

        //toggle shooter
        if (currentGamepad1.x && !previousGamepad1.x) {
            if (shooter != state.ON) {
                shooter = state.ON;
            } else if (shooter != state.OFF) {
                robot.shooter.setPower(0);
                shooter = state.OFF;
            }
        }
        if (currentGamepad1.b && !previousGamepad1.b) {
            if (shooter != state.REVERSE) {
                shooter = state.REVERSE;
            } else if (shooter != state.OFF) {
                shooter = state.OFF;
            }
        }

        //setting the target velocity
        if (shooter == state.ON) shooterVelocity = target_spin;
        else if (shooter == state.OFF) shooterVelocity = 0;
        else if (shooter == state.REVERSE) shooterVelocity = -target_spin / 2;

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


        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            if (autoAim == state.ON) {
                autoAim = state.OFF;
                target_aim = 0.95;
                robot.aim.setPosition(target_aim);
            }
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
            if (autoAim == state.OFF) {
                autoAim = state.ON;
                targetBearing = 0;
            }

        }

        double[] tempPose;

        if(autoAim == state.OFF){
            if(currentSide == side.BLUE) {
                targetBearing = -85;
            } else {
                targetBearing = 78;
            }
            target_spin = 2100;
            target_aim = 0.95;
        } else {
            //AprilTag Detection: update target location
            aprilTag.update();
            //goal is the actual apriltag
            AprilTagDetection goal = aprilTag.getTagByID(shooter_target);
            if (goal != null && goal.ftcPose != null) {
                target_range = goal.ftcPose.range;
                aprilTagBearingError = goal.ftcPose.bearing;
                for (double[] item : lookup) {
                    if (item[0] >= target_range) {
                        target_spin = item[1];
                        target_aim = item[2];
                    }
                }

                //take april tag robot pose and turn it into actual pose with the camera positions
                tempPose = new double[] {
                        goal.robotPose.getPosition().x,
                        goal.robotPose.getPosition().y,
                        goal.robotPose.getOrientation().getYaw()
                };

                Vector2d subtraction = tempsubtract.rotateBy(tempPose[2]);

                pose = new double[] {
                        tempPose[0] - subtraction.x,
                        tempPose[1] - subtraction.y,
                        tempPose[2] - turretBearing
                };

                robot.aim.setPosition(target_aim);
            }

            //picking where to shoot aim and bearing
            for (double[] item : lookup) {
                if (item[0] >= target_range) {
                    target_spin = item[1];
                    target_aim = item[2];
                }
            }


            targetBearing = turretBearing + aprilTagBearingError;

            if (targetBearing > 30) {
                targetBearing = 30;
            } else if (targetBearing < -30){
                targetBearing = -30;
            }
        }

        //toggling autoAim off
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            if (autoAim == state.ON) {
                autoAim = state.OFF;
            }
        }
        //toggle autoAim on
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
            if (autoAim == state.OFF) {
                autoAim = state.ON;
                targetBearing = 0;
            }
        }

//        double red = robot.ballColor.red();
//        double green = robot.ballColor.green();
//        double blue = robot.ballColor.blue();
//        double finalRed = red / green;
//        double finalBlue = blue / green;
//        if (green != 0 && finalRed > 0.05 && finalRed < 0.5 && finalBlue > 0.6 && finalBlue < 0.9) {
//            detectedColor = colors.GREEN;
//
//        } else if (green != 0 && finalRed > 0.5 && finalRed < 1.3 && finalBlue > 1 && finalBlue < 2.8) {
//            detectedColor = colors.PURPLE;
//        } else {
//            detectedColor = colors.UNKNOWN;
//        }


        //ramp  function: if sebastian has just started moving, beat his ass.
        //it basically prevents jerky movement by making sure it speeds up slower.
        // This way also if the driver is making precise adjustments they can go slowly
        if ((Math.abs(currentGamepad1.left_stick_x) > 0.05 || Math.abs(currentGamepad1.left_stick_y) > 0.05) && !(Math.abs(previousGamepad1.left_stick_x) > 0.05 || Math.abs(previousGamepad1.left_stick_y) > 0.05)) {
            rampFunction.reset();
        }

        double bearingError = targetBearing - turretBearing;
        if(Math.abs(bearingError) > 2) {
            robot.spin.setPower(0.015 * bearingError);
        } else {
            robot.spin.setPower(0);
        }



        // Gets power levels for each motor, using gamepad inputs as directions
        // The third item in the array dictates which trigger is being pressed (=1 if left, =-1 if right, =0 if none or both).
        motorPowers = MecanumKinematics.getPowerFromDirection(new double[] {
                        - gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                        gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                        - (toInt(gamepad1.right_bumper) - toInt(gamepad1.left_bumper))
                },
                rampFunction.getTargetSpeed(), 1, //this all just correcting for our shitty weight distribution
                true
        );

        telemetry.addData("aimer position", hoodAngle);
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
        telemetryA.addData("ourbearing", turretBearing);
        telemetry.addData("target_bearing", targetBearing);
        telemetry.addData("apriltagbearingerror", aprilTagBearingError);
        telemetry.addData("autoAim", autoAim);
        telemetryA.addData("cameraPos-x", cameraPos[0]);
        telemetryA.addData("cameraPos-y", cameraPos[1]);
        telemetryA.addData("x", pose[0]);
        telemetryA.addData("y", pose[1]);
        telemetryA.addData("angle", pose[2]);


        //These things MUST be at the end of each loop. DO NOT MOVE
        telemetry.update();
        telemetryA.update();
    }

}
