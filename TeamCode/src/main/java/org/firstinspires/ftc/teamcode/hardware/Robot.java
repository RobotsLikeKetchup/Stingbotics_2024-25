package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumKinematics;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.MovementFunctions;
import org.firstinspires.ftc.teamcode.utilities.PIDF;
import org.firstinspires.ftc.teamcode.utilities.Vector2d;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Robot {
    //fields
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public DcMotorCombined shooter;
    public DcMotor intake;
    public Servo aim;
    public Servo ballStop;
    public DcMotorEx spin;
    //parallel dead wheels (measuring x-coord and heading)
    DeadWheel parL;
    DeadWheel parR;
    //perpendicular dead wheel (measuring y-coord)
    DeadWheel per;

    public DcMotor[] driveMotors;

    public RoadrunnerThreeWheelLocalizer localization;

    ElapsedTime timer;

    //order of constants will be P, I, D, F
    public static double[] headingConstants = {0,0,0,0};
    public static double[] driveConstants = {0,0,0,0};
    public static double[] strafeConstants = {0,0,0,0};

//    public AprilTagProcessor aprilTagProcessor;
//
//    public VisionPortal visionPortal;


    //constructor
    public Robot(){};

    public void init(HardwareMap hardwareMap, ElapsedTime timer){

        localization = new RoadrunnerThreeWheelLocalizer(hardwareMap, new Pose2d(0 ,0, 0));

        this.timer = timer;

        //TUNE THIS
        double inPerTick = 0.0004;

        //Map variables to motors
        frontLeft = hardwareMap.get(DcMotor.class,"motor_fl");
        frontRight = hardwareMap.get(DcMotor.class, "motor_fr");
        backLeft = hardwareMap.get(DcMotor.class, "motor_bl");
        backRight = hardwareMap.get(DcMotor.class, "motor_br");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter = new DcMotorCombined(shooter1, shooter2);
        aim = hardwareMap.get(Servo.class, "aim");
        ballStop = hardwareMap.get(Servo.class, "ballStop");
        spin = hardwareMap.get(DcMotorEx.class, "spin");
        intake = hardwareMap.get(DcMotor.class, "intake");
        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

                // Set universal wheel behaviors
        for (DcMotor i : driveMotors) {
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            i.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        };


        //set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        spin.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set deadwheel encoders
        parL = new DeadWheel(inPerTick, backRight);
        parR = new DeadWheel(inPerTick, frontRight);
        per = new DeadWheel(inPerTick, backLeft);

        //create vision portal
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//
//                // The following default settings are available to un-comment and edit as needed.
//                .setDrawAxes(false)
//                //.setDrawCubeProjection(false)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//
//                .build();
//        int cameraMoniterViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMoniterViewId", "id", hardwareMap.appContext.getPackageName());
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "cameraOfDoom"))
//                .addProcessor(aprilTagProcessor)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .setLiveViewContainerId(cameraMoniterViewId)
//                .setAutoStartStreamOnBuild(true)
//                .build();
//
//        visionPortal.setProcessorEnabled(aprilTagProcessor, true);

    }

    //methods

    //sets motor powers in the same order as mecanumKinematics: fl, fr, bl, br
    public void setMotorPowers(double[] motorPowers){
        frontLeft.setPower(motorPowers[0]);
        frontRight.setPower(motorPowers[1]);
        backLeft.setPower(motorPowers[2]);
        backRight.setPower(motorPowers[3]);
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

            pose = localization.getPoseDouble();

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

    public class PIDtoPt implements Action {
        double[] pt;
        PIDF headingPID;
        PIDF drivePID;
        PIDF strafePID;

        double[] position;
        Vector2d direction;
        Vector2d rotatedDirection;

        double[] robotDirection;
        double angleThreshold;
        double spaceThreshold;

        public PIDtoPt(double[] pt, double angleThreshold, double spaceThreshold) {
            this.pt = pt;
            this.angleThreshold = angleThreshold;
            this.spaceThreshold = spaceThreshold;
            headingPID = new PIDF(headingConstants[0], headingConstants[1], headingConstants[2], headingConstants[3], timer);
            drivePID = new PIDF(driveConstants[0], driveConstants[1], driveConstants[2], driveConstants[3], timer);
            strafePID = new PIDF(strafeConstants[0], strafeConstants[1], strafeConstants[2], strafeConstants[3], timer);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            localization.update();
            position = localization.getPoseDouble();
            direction = new Vector2d(pt[0] - position[0], pt[1] - position[1]);
            rotatedDirection = direction.rotateBy(position[2]);

            robotDirection = new double[]{
                    drivePID.loop(0, rotatedDirection.y),
                    strafePID.loop(0, rotatedDirection.x),
                    headingPID.loop(pt[2], position[2])
            };

            setMotorPowers(MecanumKinematics.getPowerFromDirection(robotDirection, 1));


            // debuging problem with action running bool in auton testing
            return true;
            /*if (position[2] >= angleThreshold) {
                return true;
            } else {
                return false;
            }*/
        }


        }

        public Action PIDtoPt(double[] pt, double angleThreshold, double spaceThreshold) {
            return new PIDtoPt(pt, angleThreshold, spaceThreshold);
        }
    }

