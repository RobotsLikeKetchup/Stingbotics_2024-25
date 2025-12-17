package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumKinematics;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.MovementFunctions;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Robot {
    //fields
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public ColorSensor ballColor;
    public DcMotorEx shooter;
    public DcMotor intake;
    public Servo aim;
    //parallel dead wheels (measuring x-coord and heading)
    DeadWheel parL;
    DeadWheel parR;
    //perpendicular dead wheel (measuring y-coord)
    DeadWheel per;

    public DcMotor[] driveMotors;

    public RoadrunnerThreeWheelLocalizer localization;

    ElapsedTime timer;

    public AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

    public VisionPortal visionPortal;


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

        this.timer = timer;

        //TUNE THIS
        double inPerTick = 0.0004;

        //Map variables to motors
        frontLeft = hardwareMap.get(DcMotor.class,"motor_fl");
        frontRight = hardwareMap.get(DcMotor.class, "motor_fr");
        backLeft = hardwareMap.get(DcMotor.class, "motor_bl");
        backRight = hardwareMap.get(DcMotor.class, "motor_br");
        ballColor = hardwareMap.get(ColorSensor.class, "ballSensor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        aim = hardwareMap.get(Servo.class, "aim");
        intake = hardwareMap.get(DcMotor.class, "intake");
        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

                // Set universal wheel behaviors
        for (DcMotor i : driveMotors) {
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            i.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        };

        //hagrid: eres un horrocrux harry
        //harry: un que?
        //hagrid: un grrr


        //set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //set deadwheel encoders
        parL = new DeadWheel(inPerTick, backRight);
        parR = new DeadWheel(inPerTick, frontRight);
        per = new DeadWheel(inPerTick, backLeft);

        //create vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cameraOfDoom"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

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

            localization.updatePoseEstimate();

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

            final double robotWidth = 15;
            final double r = (robotWidth/2) * Math.sqrt(2);

            telemetryPacket.put("Motion Profile Phase", motionProfile.currentPhase);
            telemetryPacket.put("Speed", velocityCoeff);
            telemetryPacket.put("x", pose[0]);
            telemetryPacket.put("y", pose[1]);
            telemetryPacket.put("angle", pose[2]);

            telemetryPacket.fieldOverlay()
                    .setFill("blue")
                    .fillPolygon(new double[] {
                            pose[0] - (r * Math.sin(pose[2]+(Math.PI/4))),
                            pose[0] - (r * Math.sin(pose[2]-(Math.PI/4))),
                            pose[0] - (r * Math.sin(pose[2]-(3*Math.PI/4))),
                            pose[0] - (r * Math.sin(pose[2]+(3*Math.PI/4)))
                    }, new double[] {
                            pose[1] - (r*Math.cos(pose[2]+(Math.PI/4))),
                            pose[1] - (r*Math.cos(pose[2]-(Math.PI/4))),
                            pose[1] - (r*Math.cos(pose[2]-(3*Math.PI/4))),
                            pose[1] - (r*Math.cos(pose[2]+(3*Math.PI/4)))
                    }); //all the stuff above are the points, rotated based on the robot's angle

            for(int i=1; i < path.length; i++) {
                //draw the path
                telemetryPacket.fieldOverlay().strokeLine((int) Math.round(path[i-1][0]), (int) Math.round(path[i-1][1]), (int) Math.round(path[i][0]), (int) Math.round(path[i][1]));

            }

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

    //TODO: finish!
    public class Shoot implements Action {
        double velocity;

        public Shoot(double velocity){
            this.velocity = velocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

}

