package org.firstinspires.ftc.teamcode;

//FTC classes or something

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
        robot.init(hardwareMap, timer);
        dashboard = FtcDashboard.getInstance();
        telemetryA = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        telemetry.addLine("Initialized!");
        telemetry.update();
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
        for (int i = 0; i < motorPowers.length; i++) {
            robot.driveMotors[i].setPower(motorPowers[i]);
        }
        telemetry.addData("frontLeft", robot.frontLeft.getPower());
        telemetry.addData("backLeft", robot.backLeft.getPower());
        telemetry.addData("frontRight", robot.frontRight.getPower());
        telemetry.addData("backRight", robot.backRight.getPower());
        telemetry.update();
        telemetryA.update();
    }
    @Override
    public void stop() {
        super.stop();
    }
}
