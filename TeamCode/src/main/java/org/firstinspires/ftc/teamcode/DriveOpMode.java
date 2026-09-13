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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.AprilTag;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.MathFunctions;
import org.firstinspires.ftc.teamcode.utilities.PIDF;
import org.firstinspires.ftc.teamcode.utilities.Vector2Dim;
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
    //Pose2D pose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);


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
    public enum colors {YELLOW, RED, BLUE, UNKNOWN}

    public enum side {BLUE, RED}

    public side currentSide = side.BLUE;

    public enum state {
        ON,
        OFF,
        REVERSE
    }

    //I am NOT unemployed

    colors detectedColor = colors.UNKNOWN;

    public double blueVal;
    public double greenVal;
    public double redVal;
    public double position = 0;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    AprilTag aprilTag = new AprilTag();

    MultipleTelemetry telemetryA;

    VectorF targetAprilTagPos;


    @Override
    // Set starting values for variable
    public void init() {
        robot.init(hardwareMap, timer);
        //robot.aim.setPosition(0.85);

        dashboard = FtcDashboard.getInstance();

        telemetryA = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        telemetry.update();

        aprilTag.init(hardwareMap, telemetry);
        //get the location of the Tag on the field

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
       // robot.odometry.update();
      //  pose = robot.odometry.getPosition();
        redVal = robot.achintaSensor.red()+0.0000000000000001;
        blueVal = robot.achintaSensor.blue()/redVal;
        greenVal = robot.achintaSensor.green()/redVal;

        //turretBearing = (360 * ((robot.spin.getCurrentPosition() / SPIN_MOTOR_TPR) / SPIN_GEAR_RATIO)) + prevTurret;
        //ramp  function: if sebastian has just started moving, beat his ass.
        //it basically prevents jerky movement by making sure it speeds up slower.
        // This way also if the driver is making precise adjustments they can go slowly
        if ((Math.abs(currentGamepad1.left_stick_x) > 0.05 || Math.abs(currentGamepad1.left_stick_y) > 0.05) && !(Math.abs(previousGamepad1.left_stick_x) > 0.05 || Math.abs(previousGamepad1.left_stick_y) > 0.05)) {
            rampFunction.reset();
        }
        robot.angle1.setPosition(position);
        if (gamepad1.dpad_left){
            position += .1;
        }
        if(gamepad1.dpad_right){
            position +=.1;
        }

           // Vector2Dim fieldVelocity = new Vector2Dim(robot.odometry.getVelX(DistanceUnit.INCH), robot.odometry.getVelY(DistanceUnit.INCH));
            //Vector2Dim goalVelocity = fieldVelocity.rotateBy((currentSide==side.BLUE ? 1:-1) * (Math.PI - robotToGoalAngle));
        if (blueVal>0.2 && blueVal<0.9 && greenVal > 1.4 && greenVal < 1.6){
            detectedColor = colors.YELLOW;
        } else if (greenVal > 1.5 && greenVal < 2.5 && blueVal > 4 && blueVal < 10) {
            detectedColor = colors.BLUE;

        } else if (greenVal > 0.4 && greenVal < 1.2 && blueVal > 0.2 && blueVal < 0.6)  {
            detectedColor = colors.RED;
        }else{
            detectedColor = colors.UNKNOWN;
        }

        if (gamepad1.x){
            robot.shooter.setPower(0.8);

        } else{
            robot.shooter.setPower(0);
        }

        if (gamepad1.a){
            robot.intake.setPower(1);
        } else{
            robot.intake.setPower(0);
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
        telemetry.addData("blue#", robot.achintaSensor.blue());
        telemetry.addData("red#", robot.achintaSensor.red());
        telemetry.addData("green#", robot.achintaSensor.green());
        telemetry.addData("Detected Color: ", detectedColor);



       // telemetryA.addData("x-encoder", robot.odometry.getEncoderX());
       // telemetryA.addData("y-encoder", robot.odometry.getEncoderY());


        //These things MUST be at the end of each loop. DO NOT MOVE
        telemetry.update();
        telemetryA.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
