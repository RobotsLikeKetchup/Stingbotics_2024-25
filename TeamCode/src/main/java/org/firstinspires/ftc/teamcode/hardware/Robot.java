package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.MecanumKinematics;
import org.firstinspires.ftc.teamcode.pathing.MotionProfile1D;
import org.firstinspires.ftc.teamcode.pathing.PurePursuit;
import org.firstinspires.ftc.teamcode.pathing.StingLocalizer;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.pathing.roadrunner.RoadrunnerTwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.MovementFunctions;
import org.firstinspires.ftc.teamcode.utilities.PIDF;
import org.firstinspires.ftc.teamcode.utilities.Vector2Dim;

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
    public GoBildaPinpointDriver odometry;
    //parallel dead wheels (measuring x-coord and heading)
    DeadWheel parL;
    DeadWheel parR;
    //perpendicular dead wheel (measuring y-coord)
    DeadWheel per;

    public static final double TURRET_RADIUS = 6.49;

    public DcMotor[] driveMotors;

    public RoadrunnerTwoWheelLocalizer localization;

    public enum localizationType {ROADRUNNER, PINPOINT}

    public localizationType type = localizationType.PINPOINT;

    ElapsedTime timer;

    IMU imu;

    public static final double[] TURRET_LIMITS = {-180, 180};

    public final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;


    //order of constants will be P, I, D, F
    public static double[] headingConstants = {0.9,0,0,0};
    public static double[] driveConstants = {0.065,0,-0.00016,0};
    public static double[] strafeConstants = {-0.08,0,0,0};

    public static double[] shooterConstants = {0.00078, 0.00000023, 0.015, 0.00015};
    public static double[][] lookup = {
            {10, -1350, 0.9},
            {20, -1350, 0.82},
            {30, -1420, 0.82},
            {40, -1500, 0.7},
            {50, -1600, 0.67},
            {60, -1650, 0.6},
            {70, -1750, 0.55},
            {80, -1800, 0.5},
            {90, -2100, 0.3},

    };
//    public AprilTagProcessor aprilTagProcessor;
//
//    public VisionPortal visionPortal;


    //constructor
    public Robot(){};

    public void init(HardwareMap hardwareMap, ElapsedTime timer){

        if(type == localizationType.ROADRUNNER) {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    logoFacingDirection,
                                    usbFacingDirection
                            )
                    )
            );
            imu.resetYaw();

            localization = new RoadrunnerTwoWheelLocalizer(hardwareMap, imu, new Pose2d(0, 0, 0));
        }

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

        if(type == localizationType.PINPOINT) {
            odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            //these offsets are to the lens position at a zero turret bearing
            odometry.setOffsets(5.1, -8.2, DistanceUnit.INCH);
            odometry.setEncoderResolution(218.899, DistanceUnit.INCH);
            odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            odometry.recalibrateIMU();
            odometry.resetPosAndIMU();
        }
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

    //used for turning the camera lens pose to the location it would be if the turret was at zero
    //specific to 2025 bot, for use with pinpoint
    public static Pose2D cameraPoseCalc(Pose3D lensPose, double turretBearing){ //lens pose is from aprilTag
        //calculate the position of the camera from the straight-forward position
        double[] cameraPos = {
                TURRET_RADIUS*Math.sin(-Math.toRadians(turretBearing)),
                TURRET_RADIUS*(-1+Math.cos(Math.toRadians(turretBearing)))
        };

        Vector2Dim tempsubtract = new Vector2Dim(cameraPos[0], cameraPos[1]);


        //take april tag robot pose and turn it into actual pose with the camera positions
        //IMPORTANT: in this, x and y are switched because of the way the aprilTag calculates

        //a note on frame of reference: here, (0,0) is the center of the field. 0 heading is forward.
        //y is positive towards the goals and negative away
        //x is negative towards blue goal and positive towards red goal.
        //angle must be between x axis and front of robot
        double[] tempPose = new double[] {
                lensPose.getPosition().y,
                - lensPose.getPosition().x,
                lensPose.getOrientation().getYaw()
        };
        Vector2Dim subtraction = tempsubtract.rotateBy(Math.toRadians(tempPose[2]-90));

        Pose2D pose = new Pose2D(
                DistanceUnit.INCH,
                tempPose[0] - subtraction.x,
                tempPose[1] - subtraction.y,
                AngleUnit.RADIANS,
                Math.toRadians(tempPose[2] - turretBearing)
        );

        return pose;
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

            localization.update();

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


    public class Shoot implements Action {
        double velocity;
        int ballNumber;
        boolean init = false;
        int ballIndex = 0;
        double intakeStartTime;
        double intakeTime = 0.6;
        double revUpTime = 1.2;
        double ballRevUpTime = 0.4;
        PIDF shooterpid = new PIDF(shooterConstants, timer);

        public Shoot(double velocity, int ballNumber){
            this.velocity = velocity;
            this.ballNumber = ballNumber;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setPower(shooterpid.loop(velocity, shooter.getVelocity()));
            if(!init){
                intakeStartTime = timer.seconds() + revUpTime;
                init = true;
            }
            if (timer.seconds() >= intakeStartTime + intakeTime){
                ballIndex ++;
                intakeStartTime = timer.seconds() + ballRevUpTime;
            }
            if(timer.seconds() <= intakeStartTime){
                intake.setPower(0);
            } else {
                intake.setPower(1);
            }

            telemetryPacket.put("balli", ballIndex);
            telemetryPacket.put("seconds", timer.seconds());
            telemetryPacket.put("startTime", intakeStartTime);

            if(ballIndex == ballNumber) {
                shooter.setPower(0);
                intake.setPower(0);
                return false;
            } else {
                return true;
            }
        }
    }

    public Action shoot(double velocity, int ballNumber) {
        return new Shoot(velocity, ballNumber);
    }

    public class PIDtoPt implements Action {
        double[] pt;
        PIDF headingPID;
        PIDF drivePID;
        PIDF strafePID;
        double maxPower = 1;

        double[] position;
        Vector2Dim direction;
        Vector2Dim rotatedDirection;
        double rotation;

        double[] robotDirection;
        double angleThreshold;
        double spaceThreshold;

        public PIDtoPt(double[] pt, double angleThreshold, double spaceThreshold) {
            this.pt = pt;
            this.angleThreshold = angleThreshold;
            this.spaceThreshold = spaceThreshold;
            headingPID = new PIDF(headingConstants[0], headingConstants[1], headingConstants[2], headingConstants[3], timer, true);
            drivePID = new PIDF(driveConstants[0], driveConstants[1], driveConstants[2], driveConstants[3], timer);
            strafePID = new PIDF(strafeConstants[0], strafeConstants[1], strafeConstants[2], strafeConstants[3], timer);
        }
        public PIDtoPt(double[] pt, double angleThreshold, double spaceThreshold, double maxSpeed) {
            this.pt = pt;
            this.angleThreshold = angleThreshold;
            this.spaceThreshold = spaceThreshold;
            headingPID = new PIDF(headingConstants[0], headingConstants[1], headingConstants[2], headingConstants[3], timer, true);
            drivePID = new PIDF(driveConstants[0], driveConstants[1], driveConstants[2], driveConstants[3], timer);
            strafePID = new PIDF(strafeConstants[0], strafeConstants[1], strafeConstants[2], strafeConstants[3], timer);
            this.maxPower = maxSpeed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            position = localization.getPoseDouble();
            direction = new Vector2Dim(pt[0] - position[0], pt[1] - position[1]);
            rotation = (Math.PI/2) - position[2];
            rotatedDirection = direction.rotateBy(rotation);

            //flip x and y, to simplify rotation
            robotDirection = new double[]{
                    strafePID.loop(rotatedDirection.x),
                    drivePID.loop(rotatedDirection.y),
                    headingPID.loop(pt[2], position[2])
            };

            telemetryPacket.put("x-direction", robotDirection[0]);
            telemetryPacket.put("y-direction", robotDirection[1]);
            telemetryPacket.put("vector-rotation", rotation);
            telemetryPacket.put("norotate-direction-x", direction.x);
            telemetryPacket.put("norotate-direction-y", direction.y);
            telemetryPacket.put("rotate-direction-x", rotatedDirection.x);
            telemetryPacket.put("rotate-direction-y", rotatedDirection.y);
            telemetryPacket.put("angle-direction", robotDirection[2]);

            setMotorPowers(MecanumKinematics.getPowerFromDirection(robotDirection, maxPower));


            if (Math.abs(pt[2] - position[2]) >= angleThreshold | Math.abs(direction.x) >= spaceThreshold | Math.abs(direction.y) >= spaceThreshold) {
                return true;
            } else {
                return false;
            }
        }


    }

    public Action PIDtoPt(double[] pt, double angleThreshold, double spaceThreshold) {
        return new PIDtoPt(pt, angleThreshold, spaceThreshold);
    }
    public Action PIDtoPt(double[] pt, double angleThreshold, double spaceThreshold, double maxPower) {
        return new PIDtoPt(pt, angleThreshold, spaceThreshold, maxPower);
    }

    public class Stop implements Action {
        public Stop(){}
        @Override
        public boolean run(TelemetryPacket packet) {
            for(DcMotor motor : driveMotors){
                motor.setPower(0);
            }
            shooter.setPower(0);
            intake.setPower(0);

            return false;
        }
    }

    public Action stop() {return new Stop();}

    public class StartIntake implements Action {
        public StartIntake() {}

        public boolean run(@NonNull TelemetryPacket packet){
            intake.setPower(0.8);
            return false;
        }
    }

    public Action startIntake() {return new StartIntake();}

    public class BallDown implements Action {
        PIDF shooterPID = new PIDF(shooterConstants, timer);

        public BallDown() {}
        public boolean run (@NonNull TelemetryPacket packet) {
            shooter.setPower(0.6);

            return true;
        }
    }

    public Action ballDown() {return new BallDown();}

}

