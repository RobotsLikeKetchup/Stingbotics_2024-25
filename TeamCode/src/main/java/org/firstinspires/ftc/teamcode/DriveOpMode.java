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

    double slidePower = 0;

    Robot robot = new Robot();

    ElapsedTime timer= new ElapsedTime();

    RoadrunnerThreeWheelLocalizer localizer;

    enum IntakeState {IN, OUT, OFF};
    enum HangState{UP, DOWN};
    enum ArmState{UP, DOWN, MIDDLE};
    enum ElbowState{UP,DOWN};
    IntakeState currentIntakeState = IntakeState.OFF;

    HangState currentHangState = HangState.DOWN;

    ArmState currentArmState = ArmState.MIDDLE;
    ArmState weightCorrectionArmState = ArmState.MIDDLE;
    int upArmVal;
    int downArmVal;
    ElbowState currentElbowState = ElbowState.UP;

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


        //the limit switch stuff should be at the start of the loop cause if it is not, then you run the risk of
        //the robot powering the arm for a little bit even like a milisecond which makes the arm go too far and break stuff
        if(robot.frontArmLimitSwitch.isPressed()) {
            telemetry.addLine("front limit switch pressed");
            currentArmState = ArmState.DOWN;
            downArmVal = robot.armRotate.getCurrentPosition();
            weightCorrectionArmState = ArmState.DOWN;
        } else if(robot.backArmLimitSwitch.isPressed()){
            telemetry.addLine("back limit switch pressed");
            currentArmState = ArmState.UP;
            upArmVal = robot.armRotate.getCurrentPosition();
            weightCorrectionArmState = ArmState.UP;
        }
        else {
            currentArmState = ArmState.MIDDLE;
        }
        telemetry.addData("arm position", currentArmState);

        //now everything else

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
            rampFunction.getTargetSpeed(),
            (weightCorrectionArmState==ArmState.UP ? UP_WEIGHT_CORRECTION : DOWN_WEIGHT_CORRECTION), //this all just correcting for our shitty weight distribution
            true
        );

        telemetry.addData("gamepadx", currentGamepad1.left_stick_x);
        telemetry.addData("gamepady", currentGamepad1.left_stick_y);

        // Sets power levels
        // Works because each index corresponds with the same wheel in both arrays
        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        }


        //rotating arm
        robot.armRotate.setPower(currentGamepad1.b && !(currentArmState == ArmState.DOWN) ? 1 : (currentGamepad1.x && !(currentArmState == ArmState.UP) ? -1 : 0));

        /*if(currentGamepad1.b) {
            currentArmState = ArmState.DOWN;
        } if(currentGamepad1.a) {
            currentArmState=ArmState.UP;
        } */


        //linear slide
        if(currentGamepad1.a){
            slidePower = -1;
        } else if(currentGamepad1.y){
            //to prevent the robot from extending past the 42 inch limit specified by the rules
            if(currentArmState == ArmState.DOWN) {
                if (robot.armExtend.getCurrentPosition() <= 3000) {
                    slidePower = 1;
                } //change value to max encoder position of arm
            } else {
                if (robot.armExtend.getCurrentPosition() <= 5900) {
                    slidePower = 1;
                } //change value to max encoder position of arm
            }
        } else {
            slidePower = 0;
        }

        // Servo Controller            RT is in , LT is out
        //this implements a simple toggle system
        if(currentGamepad1.right_trigger >= 0.4 && previousGamepad1.right_trigger <= 0.4) {
            telemetry.addLine("Right Trigger Released");
            if (currentIntakeState == IntakeState.OFF || currentIntakeState == IntakeState.OUT){
                currentIntakeState = IntakeState.IN;
            } else {
                currentIntakeState = IntakeState.OFF;
            }
        }

        if(currentGamepad1.left_trigger >= 0.4 && previousGamepad1.left_trigger <= 0.4) {
            telemetry.addLine("Left Trigger Released");
            if (currentIntakeState == IntakeState.OFF || currentIntakeState == IntakeState.IN) {
                currentIntakeState = IntakeState.OUT;
            } else {
                currentIntakeState = IntakeState.OFF;
            }
        }

        //there is both a roller and a claw on the robot to pick up game pieces in different ways
        if(currentIntakeState == IntakeState.OFF) {
            robot.intakeRoller.setPower(0);
        }
        else if (currentIntakeState == IntakeState.IN) {
            robot.intakeRoller.setPower(1);
            robot.intakeClaw.setPosition(0.9); //change to measured value
        }
        else {
            robot.intakeRoller.setPower(-1);
            robot.intakeClaw.setPosition(0.1); //change to measured value
            robot.openClaw().run(packet);
        }

        //moves the elbow
        if(currentGamepad1.dpad_down) {
            currentElbowState = ElbowState.DOWN;
        }
        if(currentGamepad1.dpad_up) {
            currentElbowState = ElbowState.UP;
        }
        if(currentElbowState == ElbowState.UP) {
            robot.intakeElbow.setPosition(0.7);//test out to find correct position
        } else {
            if(currentArmState == ArmState.DOWN) {
                //this is so that when the arm is in the down position, the roller can grab stuff at the exact right angle
                robot.intakeElbow.setPosition(ELBOW_INTERCEPT + (ELBOW_SLOPE * robot.armExtend.getCurrentPosition())); //fine-tune this
            } else {
                robot.intakeElbow.setPosition(ELBOW_INTERCEPT);//test out to find correct position
            }
        }

        telemetry.addData("Encoder 1", robot.getDeadwheel("parR").getTicks());
        telemetry.addData("Encoder 2", robot.getDeadwheel("parL").getTicks());
        telemetry.addData("Encoder 3", robot.getDeadwheel("per").getTicks());
        telemetry.addData("Extension arm position", robot.armExtend.getCurrentPosition());
        telemetry.addData("Rotation arm position", robot.armRotate.getCurrentPosition());


        telemetry.addData("intake state", currentIntakeState);
        telemetry.addData("arm state", currentArmState);
        telemetry.addData("frontLeft", robot.frontLeft.getPower());
        telemetry.addData("backLeft", robot.backLeft.getPower());
        telemetry.addData("frontRight", robot.frontRight.getPower());
        telemetry.addData("backRight", robot.backRight.getPower());



        //These things MUST be at the end of each loop. DO NOT MOVE

        if(robot.slideLimitSwitch.isPressed()) {
            if (!currentGamepad1.y){slidePower = 0;}
            robot.armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addLine("Limit Switch Pressed!");
        }
        if(currentArmState == ArmState.DOWN && robot.armExtend.getCurrentPosition() >=3000){
            if(!currentGamepad1.a){
                slidePower=0;
            }
        }

        robot.armExtend.setPower(slidePower);


        telemetry.update();
    }

}
