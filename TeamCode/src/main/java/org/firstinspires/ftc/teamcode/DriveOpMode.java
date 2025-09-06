// Import FTC package
package org.firstinspires.ftc.teamcode;
// Import FTC classes
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import custom-made classes/methods
import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.EzraLocalizer;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;

//This opMode uses the standardconfig configuration file.
@TeleOp

@Config
public class DriveOpMode extends OpMode {
    // Create variables
    double[] motorPowers;
//ur nt shkspr vro
    Robot robot = new Robot();

    ElapsedTime timer= new ElapsedTime();

    RoadrunnerThreeWheelLocalizer localizer;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();

    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    MotionProfile1D rampFunction = new MotionProfile1D(0.8,1, 0.4, timer);


    //config variables
    public static double UP_WEIGHT_CORRECTION = 0.65;
    public static double DOWN_WEIGHT_CORRECTION = 1;

    public static double ELBOW_INTERCEPT = 0.3;
    public static double ELBOW_SLOPE = 0.000055;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    // Set starting values for variable
    public void init() {
        robot.init(hardwareMap, timer);
        //localizer = new RoadrunnerThreeWheelLocalizer(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        //telemetry.addData("X pos",localizer.getPose()[0]);
        //telemetry.addData("y pos",localizer.getPose()[1]);
        //telemetry.addData("Angle pos",localizer.getPose()[2]);
        telemetry.update();
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


        //ramp  function: if sebastian has just started moving, start the motion profile.
        if ((Math.abs(currentGamepad1.left_stick_x) > 0.05 || Math.abs(currentGamepad1.left_stick_y) > 0.05) && !(Math.abs(previousGamepad1.left_stick_x) > 0.05 || Math.abs(previousGamepad1.left_stick_y) > 0.05)) {
            rampFunction.reset();
        }


        // Gets power levels for each motor, using gamepad inputs as directions
        // The third item in the array dictates which trigger is being pressed (=1 if left, =-1 if right, =0 if none or both).
        motorPowers = MecanumKinematics.getPowerFromDirection(new double[] {
                gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                toInt(gamepad1.right_bumper) - toInt(gamepad1.left_bumper)
            },
            rampFunction.getTargetSpeed(), 1, //this all just correcting for our shitty weight distribution
            true
        );

        telemetry.addData("gamepadx", currentGamepad1.left_stick_x);
        telemetry.addData("gamepady", currentGamepad1.left_stick_y);

        // Sets power levels
        // Works because each index corresponds with the same wheel in both arrays
        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        }


        // Servo Controller            RT is in , LT is out
        //this implements a simple toggle system


        telemetry.addData("Encoder 1", robot.getDeadwheel("parR").getTicks());
        telemetry.addData("Encoder 2", robot.getDeadwheel("parL").getTicks());
        telemetry.addData("Encoder 3", robot.getDeadwheel("per").getTicks());

        telemetry.addData("frontLeft", robot.frontLeft.getPower());
        telemetry.addData("backLeft", robot.backLeft.getPower());
        telemetry.addData("frontRight", robot.frontRight.getPower());
        telemetry.addData("backRight", robot.backRight.getPower());

        telemetry.addData("red", robot.ballColor.red());
        telemetry.addData("green", robot.ballColor.green());
        telemetry.addData("blue", robot.ballColor.blue());


        //These things MUST be at the end of each loop. DO NOT MOVE
        telemetry.update();
    }

}
