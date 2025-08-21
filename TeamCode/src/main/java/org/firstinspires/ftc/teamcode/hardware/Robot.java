package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumKinematics;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.MovementFunctions;

public class Robot {
    //fields
    public DcMotor frontRight, armRotate, armExtend;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    public CRServo intakeRoller;
    public Servo intakeElbow, intakeClaw;

    //parallel dead wheels (measuring x-coord and heading)
    DeadWheel parL;
    DeadWheel parR;
    //perpendicular dead wheel (measuring y-coord)
    DeadWheel per;

    public TouchSensor slideLimitSwitch;
    public TouchSensor frontArmLimitSwitch;
    public TouchSensor backArmLimitSwitch;
    public DcMotor[] driveMotors;

    public RoadrunnerThreeWheelLocalizer localization;

    ElapsedTime timer;

    //constructor
    public Robot(DcMotor fR, DcMotor fL, DcMotor bR, DcMotor bL){
        frontRight = fR;
        frontLeft = fL;
        backRight = bR;
        backLeft = bL;
    }

    public Robot(){};

    public void init(HardwareMap hardwareMap, ElapsedTime timer){

        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap, new Pose2d(0 ,0, Math.PI / 2));

        this.timer=timer;

        //TUNE THIS
        double inPerTick = 0.0004;

        // Map variables to motors
        frontLeft = hardwareMap.get(DcMotor.class,"motor_fl");
        frontRight = hardwareMap.get(DcMotor.class, "motor_fr");
        backLeft = hardwareMap.get(DcMotor.class, "motor_bl");
        backRight = hardwareMap.get(DcMotor.class, "motor_br");

        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");

        intakeRoller = hardwareMap.get(CRServo.class, "intakeRoller");
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
        intakeClaw =  hardwareMap.get(Servo.class, "intakeClaw");

        slideLimitSwitch = hardwareMap.get(TouchSensor.class, "slideLimitSwitch");
        frontArmLimitSwitch = hardwareMap.get(TouchSensor.class, "frontArmLimitSwitch");
        backArmLimitSwitch = hardwareMap.get(TouchSensor.class, "backArmLimitSwitch");

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        // Set universal wheel behaviors
        for (DcMotor i : driveMotors) {
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            i.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        };
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //set deadwheel encoders
        parL = new DeadWheel(inPerTick, backRight);
        parR = new DeadWheel(inPerTick, frontRight);
        per = new DeadWheel(inPerTick, backLeft);

        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //methods
    public void setDeadwheels(DeadWheel p1, DeadWheel p2, DeadWheel pr){
        parL = p1;
        parR = p2;
        per = pr;
    }

    public DeadWheel getDeadwheel(String location){ // location can be parL, parR, or perp
        DeadWheel out;
        switch (location) {
            case "parL":
                out = parL;
                break;
            case "parR":
                out = parR;
                break;
            case "perp":
            case "per":
                out = per;
                break;
            default:
                out = null;
                break;
        }
        return out;
    }

    //Actions!
    public class followPath implements Action {
        boolean initialized = false;
        double[][] path;
        MotionProfile1D motionProfile = new MotionProfile1D(0.8, 0.4, timer);
        double velocityCoeff;
        double[] goalPoint, direction, motorPowers;
        PurePursuit pathing;
        double[] pose;
        double optimalAngle;
        boolean optimalAngleFieldReferenceFrame;

        public followPath(double[][] path, double optimalAngle, boolean optimalAngleFieldReferenceFrame) {
            this.path = path;
            pathing = new PurePursuit(path, localization);
            this.optimalAngle = optimalAngle;
            this.optimalAngleFieldReferenceFrame = optimalAngleFieldReferenceFrame;
        }

        //this is essentially what the action does. It will run until it returns false.
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                motionProfile.startSpeedUp();
                initialized = true;
                telemetryPacket.addLine("initialized!");
            }
            pose = localization.getPose();

            goalPoint = pathing.findPointOnPath();
            if(optimalAngleFieldReferenceFrame) {
                direction = MovementFunctions.createMovementVector(pose, goalPoint);
                direction[2] = MovementFunctions.proportionalAngleCorrection(optimalAngle, pose[2]);
            } else {
                direction = MovementFunctions.createMovementVector(pose, goalPoint, optimalAngle);
            }

            //uses a PID to make sure the velocity stays consistent
            //velocityCoeff = velocityControl.loop(motionProfile.getTargetSpeed(), Math.hypot(localization.getVelocity()[0], localization.getVelocity()[1]));

            velocityCoeff = motionProfile.getTargetSpeed();
            motorPowers = MecanumKinematics.getPowerFromDirection(direction, velocityCoeff);

            for (int i = 0; i < motorPowers.length; i++) {
                driveMotors[i].setPower(-motorPowers[i]);//negative is temporary until I figure out what is making the robot go backward
            }

            if ((pathing.getDistanceFromEnd() <= 10) && (motionProfile.currentPhase == MotionProfile1D.Phase.CONSTANT_SPEED || motionProfile.currentPhase == MotionProfile1D.Phase.SPEED_UP)) {
                motionProfile.startSlowDown();
            }

            telemetryPacket.put("Motion Profile Phase", motionProfile.currentPhase);
            telemetryPacket.put("Speed", velocityCoeff);

            if (motionProfile.currentPhase == MotionProfile1D.Phase.STOPPED) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action followPath(double[][] path, double optimalAngle, boolean optimalAngleFieldReferenceFrame) {
        return new followPath(path, optimalAngle, optimalAngleFieldReferenceFrame);
    }

    public class clipArmUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeElbow.setPosition(0.2);
            if(!backArmLimitSwitch.isPressed()) {
                armRotate.setPower(-0.8);
            } else {
                armRotate.setPower(0);
            }
            if(armExtend.getCurrentPosition() <= 4400) {
                armExtend.setPower(1);
            } else {
                armExtend.setPower(0);
            }
            if(backArmLimitSwitch.isPressed() && (armExtend.getCurrentPosition() >= 4400)) {
                return false;
            } else {
                return true;
            }
        }
    }

    Action clipArmUp() {
        return new clipArmUp();
    }

    public class clipArmDown implements Action {
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                armExtend.setPower(-0.5);
                initialized = true;
            }

            if(armExtend.getCurrentPosition() <= 3000) {
                armExtend.setPower(0);
                return false;
            } else {
                return true;
            }
        }
    }

    Action clipArmDown() {
        return new clipArmDown();
    }

    public class openClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeClaw.setPosition(0.1);
            return false;
        }
    }

    public Action openClaw() {
        return new openClaw();
    }



}

