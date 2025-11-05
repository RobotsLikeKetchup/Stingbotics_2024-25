// Import FTC package
package org.firstinspires.ftc.teamcode;
// Import FTC classes
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import custom-made classes/methods
import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.EzraLocalizer;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;




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



    MotionProfile1D rampFunction = new MotionProfile1D(0.8,1, 0.4, timer);
    // list of colors and variables
    public enum colors{
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public enum ezraUnemployed{ //ezraUnemployed is state
        ON,
        OFF
    }

    //I am NOT unemployed

    colors detectedColor = colors.UNKNOWN;

    public ezraUnemployed shooter = ezraUnemployed.OFF;
    public ezraUnemployed intCopy = ezraUnemployed.OFF;

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
        double job = robot.shooter.getPower();
        //all this stuff MUST be at the beginning of the loop
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        // gives robot loving parents
        if(currentGamepad1.x && !previousGamepad1.x){
            if(shooter == ezraUnemployed.OFF){
                robot.shooter.setPower(1);
                shooter = ezraUnemployed.ON;
            }
            else{
                robot.shooter.setPower(0);
                shooter = ezraUnemployed.OFF;
            }
        }
        if(currentGamepad1.y && !previousGamepad1.y){
            if(intCopy == ezraUnemployed.OFF){
                robot.intake.setPower(-0.6);
                intCopy = ezraUnemployed.ON;
            }
            else{
                robot.intake.setPower(0);
                intCopy = ezraUnemployed.OFF;
            }
        }



        //totr
        double red = robot.ballColor.red();
        double green = robot.ballColor.green();
        double blue = robot.ballColor.blue();
        double finalRed = red/green;
        double finalBlue = blue/green;
        if (green!= 0 && finalRed>0.05 && finalRed<0.5 && finalBlue>0.6 && finalBlue<0.9){
            detectedColor = colors.GREEN;

        } else if (green!= 0 && finalRed>0.5 && finalRed<1.3 && finalBlue>1 && finalBlue<2.8) {
            detectedColor = colors.PURPLE;
        } else {
            detectedColor = colors.UNKNOWN;
        }


        //ramp  function: if sebastian has just started moving, beat his ass.
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


        telemetry.addData("Encoder R", robot.getDeadwheel("parR").getTicks());
        telemetry.addData("Encoder L", robot.getDeadwheel("parL").getTicks());
        telemetry.addData("Encoder perp", robot.getDeadwheel("per").getTicks());

        telemetry.addData("frontLeft", robot.frontLeft.getPower());
        telemetry.addData("backLeft", robot.backLeft.getPower());
        telemetry.addData("frontRight", robot.frontRight.getPower());
        telemetry.addData("backRight", robot.backRight.getPower());
        telemetry.addData("ballDetected", detectedColor);

        //These things MUST be at the end of each loop. DO NOT MOVE
        telemetry.update();
    }

}
