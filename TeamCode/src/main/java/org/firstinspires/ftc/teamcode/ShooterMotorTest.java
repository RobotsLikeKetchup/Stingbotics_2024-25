package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.PIDF;

import java.util.Arrays;


@TeleOp
@Config
public class ShooterMotorTest extends OpMode {
    Robot robot =  new Robot();
    double power;
    ElapsedTime timer = new ElapsedTime();
    FtcDashboard dashboard;
    TelemetryPacket packet;
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    @Override
    public void init() {
        robot.init(hardwareMap, timer);
        power = 1;
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }+
    @Override
    public void loop() {
        if(gamepad1.dpad_up) {
            robot.shooter1.setPower(-1);
            robot.shooter2.setPower(-1);
        }
        if(gamepad1.dpad_down){
            robot.shooter1.setPower(0);
            robot.shooter2.setPower(0);
        }
        telemetry.addData("shooter1Power", robot.shooter1.getPower());
        telemetry.addData("shooter2Power", robot.shooter2.getPower());
        telemetry.addData("shooter1Velocity",robot.shooter1.getVelocity());
        telemetry.addData("shooter2Velocity", robot.shooter2.getVelocity());
        telemetry.update();

    }
    @Override
    public void stop() {
        super.stop();
    }
}