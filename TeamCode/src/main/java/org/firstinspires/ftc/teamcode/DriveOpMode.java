// Import FTC package
package org.firstinspires.ftc.teamcode;

// Import FTC classes
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

//This opMode uses the standardconfig configuration file.
@TeleOp


public class DriveOpMode extends OpMode {
    // Create variables
    double[] motorPowers;

    double slidePower = 0;

    Robot robot = new Robot();

    ElapsedTime timer= new ElapsedTime();

    EzraLocalizer localizer = new EzraLocalizer(robot,new double[] {0,0,0},timer);

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

    MotionProfile1D rampFunction = new MotionProfile1D(1, 0.4, timer);

    @Override
    // Set starting values for variables
    public void init() {
        robot.init(hardwareMap);
        localizer.setWheelDistances(16.5,8);
        localizer.init();

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        telemetry.addData("X pos",localizer.getPose()[0]);
        telemetry.addData("y pos",localizer.getPose()[1]);
        telemetry.addData("Angle pos",localizer.getPose()[2]);
        telemetry.update();
    }

    /*@Override
    public void start() {

    }*/

    @Override
    public void loop() {
        //all this stuff MUST be at the beginning of the opMode
        localizer.calcPose();
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

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
        }, rampFunction.getTargetSpeed(), (weightCorrectionArmState==ArmState.UP ? 1.5 : 1), //this all just correcting for our shitty weight distribution
                true);

        // Sets power levels
        // Works because each index corresponds with the same wheel in both arrays
        for (int i=0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        }


        //rotating arm
        robot.armRotate.setPower(currentGamepad1.b && !(currentArmState == ArmState.DOWN) ? 1 : (currentGamepad1.x && !(currentArmState == ArmState.UP) ? -1 : 0));

        //if(gamepad1.b){robot.armRotate.setTargetPosition(-2404);};
        //if(gamepad1.x){robot.armRotate.setTargetPosition(480);};


        //linear slide
        if(currentGamepad1.a){
            slidePower = -0.8;
        } else if(currentGamepad1.y){
            if(robot.armExtend.getCurrentPosition() <= 5900) {
                slidePower = 0.8;
            } //change value to max encoder position of arm
        } else {
            slidePower = 0;
        }

        // Servo Controller            RT is in , LT is out

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
        }

        if(currentGamepad1.dpad_down) {
            currentElbowState = ElbowState.DOWN;
        }
        if(currentGamepad1.dpad_up) {
            currentElbowState = ElbowState.UP;
        }
        if(currentElbowState == ElbowState.UP) {
            robot.intakeElbow.setPosition(0.5);//test out to find correct position
        } else {
            if(currentArmState == ArmState.DOWN) {
                robot.intakeElbow.setPosition(0.1 - (0.0001 * robot.armExtend.getCurrentPosition()));
            } else {
                robot.intakeElbow.setPosition(0.08);//test out to find correct position
            }
        }


        //hang
        if(currentGamepad2.x && !previousGamepad2.x) {
            if(currentHangState == HangState.UP) {
                currentHangState = HangState.DOWN;
            } else {
                currentHangState = HangState.UP;
            }
        }
        if(currentHangState == HangState.UP){
            slidePower = -0.8;
            robot.armRotate.setPower(-0.8);
        }


        /* telemetry.addData("X pos",localizer.getPose()[0]);
        telemetry.addData("y pos",localizer.getPose()[1]);
        telemetry.addData("Angle pos",localizer.getPose()[2]); */
        telemetry.addData("Encoder 1", robot.getDeadwheel("parR").getTicks());
        telemetry.addData("Encoder 2", robot.getDeadwheel("parL").getTicks());
        telemetry.addData("Encoder 3", robot.getDeadwheel("per").getTicks());
        telemetry.addData("Extension arm position", robot.armExtend.getCurrentPosition());
        telemetry.addData("Rotation arm position", robot.armRotate.getCurrentPosition());


        telemetry.addData("left trigger", gamepad1.left_trigger);
        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.addData("intake state", currentIntakeState);
        telemetry.addData("arm state", currentArmState);



        //These things MUST be at the end of each loop. DO NOT MOVE

        if(robot.slideLimitSwitch.isPressed()) {
            if (!currentGamepad1.y){slidePower = 0;}
            robot.armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addLine("Limit Switch Pressed!");
        }

        robot.armExtend.setPower(slidePower);


        telemetry.update();
    }

}
