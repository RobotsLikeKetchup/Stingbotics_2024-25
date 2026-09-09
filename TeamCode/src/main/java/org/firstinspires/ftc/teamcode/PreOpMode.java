package org.firstinspires.ftc.teamcode;

//FTC classes or something

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import custom-made classes/methods
import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
public class PreOpMode extends OpMode {
    // Create variables
    double[] motorPowers;
    Robot robot = new Robot();
    ElapsedTime timer = new ElapsedTime();
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor intake;
    public Servo servo1;
    public Servo servo2;

    //initial position of robot: MAKE SURE TO CHANGE FOR COMP
    Pose2D pose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);


    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();

    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    FtcDashboard dashboard;
    TelemetryPacket packet;
    MultipleTelemetry telemetryA;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry.addLine("Initialized!");
        telemetry.update();

        frontLeft = hardwareMap.get(DcMotor.class,"motor_fl");
        frontRight = hardwareMap.get(DcMotor.class, "motor_fr");
        backLeft = hardwareMap.get(DcMotor.class, "motor_bl");
        backRight = hardwareMap.get(DcMotor.class, "motor_br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo2.setDirection(Servo.Direction.REVERSE);

        servo1.setPosition(0);
        servo2.setPosition(0);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        telemetry.addData("gamepadx", currentGamepad1.left_stick_x);
        telemetry.addData("gamepady", currentGamepad1.left_stick_y);
        // Sets power levels
        // Works because each index corresponds with the same wheel in both arrays

        motorPowers = MecanumKinematics.getPowerFromDirection(new double[]{
                        -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                        -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                        -(toInt(gamepad1.right_bumper) - toInt(gamepad1.left_bumper))
                },
                1.0
        );

        frontLeft.setPower(motorPowers[0]);
        frontRight.setPower(motorPowers[1]);
        backLeft.setPower(motorPowers[2]);
        backRight.setPower(motorPowers[3]);

        //servo1.setPosition(gamepad1.right_trigger);
        //servo2.setPosition(-gamepad1.right_trigger);
        if(gamepad1.dpadDownWasPressed()){
            servo1.setPosition(0);
            servo2.setPosition(0);
        }
        if(gamepad1.dpadUpWasPressed()){
            servo1.setPosition(0.05);
            servo2.setPosition(0.4);
        }
        //DO NOT CHANGE THE SHIT ON TOP OF THIS if it works it works
        if(currentGamepad1.a){
            intake.setPower(1);
        }else if(currentGamepad1.y){
            intake.setPower(-1);
        }else{
            intake.setPower(0);
        }

        double position1 = servo1.getPosition();
        double position2 = servo2.getPosition();

        telemetry.addData("frontLeft", frontLeft.getPower());
        telemetry.addData("backLeft", backLeft.getPower());
        telemetry.addData("frontRight", frontRight.getPower());
        telemetry.addData("backRight", backRight.getPower());
        telemetry.addData("servo1", position1);
        telemetry.addData("servo2", position2);
        telemetry.addData("intake", intake.getPower());
        telemetry.update();
        telemetryA.update();
    }
    @Override
    public void stop() {
        super.stop();
    }
}
