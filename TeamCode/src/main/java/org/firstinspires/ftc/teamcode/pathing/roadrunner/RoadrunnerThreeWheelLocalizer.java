package org.firstinspires.ftc.teamcode.pathing.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pathing.StingLocalizer;
import org.firstinspires.ftc.teamcode.utilities.Vector2Dim;


//This is the roadrunner three deadwheel localizer, I just adapted it for our code.
//It does the math to see where the robot is any any point, based on deadwheel inputs
//It is a lot more robust to issues like bad deadwheel placement and encoder readings overflowing
//So until I am skilled enough to code my own and have enough time to test it, this is what we got.

@Config
public final class RoadrunnerThreeWheelLocalizer implements Localizer, StingLocalizer {
    public static class Params {
        public double par0YTicks = -409.9520267332982; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 619.4442806739024; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = -446.1845844487434; // x position of the perpendicular encoder (in tick units)
    }

    public static final double TURRET_RADIUS = 6.49;

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public static double inPerTick =  0.00409623;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;

    public Pose2d pose;

    public PoseVelocity2d velocity;

    public RoadrunnerThreeWheelLocalizer(HardwareMap hardwareMap, Pose2d pose) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "motor_fl")));//left
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "motor_br")));//right
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "motor_bl")));//middle - umed

        // TODO: reverse encoder directions if needed
        //   par0.setDirection(DcMotorSimple.Direction.REVERSE);
        par0.setDirection(DcMotorSimple.Direction.FORWARD);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);


        this.pose = pose;
        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }
    public void updatePoseEstimate(){
        Twist2dDual<Time> twist = update();
        pose = pose.plus(twist.value());
        velocity = twist.velocity().value();
    }
    public double[] getPoseDouble() {
        //converting the complex number format that roadrunner uses into an angle in radians
        double angle = Math.atan2(pose.heading.real, pose.heading.imag);

        return new double[] {pose.component1().x, pose.component1().y, angle};
    }
    public Pose2d getPose2D() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public double[] getVelocity() {
        return new double[]{velocity.linearVel.x, velocity.linearVel.y, velocity.angVel};
    }

    //used for turning the camera lens pose to the robot's pose
    //specific to 2025 bot
    public static Pose2d cameraToRobotPose(Pose3D lensPose, double turretBearing){ //lens pose is from aprilTag
        //calculate the position of the camera from the straight-forward position
        double[] cameraPos = {
                TURRET_RADIUS*Math.sin(-Math.toRadians(turretBearing)),
                TURRET_RADIUS*(-1+Math.cos(Math.toRadians(turretBearing)))
        };

        //calculating distance between cameraPosition and center of rotation of the robot, done with calculations on the robot
        double tempX = cameraPos[0] + 4.62 - 3.75 - (RoadrunnerThreeWheelLocalizer.PARAMS.perpXTicks * RoadrunnerThreeWheelLocalizer.inPerTick);
        double tempY = RoadrunnerThreeWheelLocalizer.PARAMS.par1YTicks * RoadrunnerThreeWheelLocalizer.inPerTick;

        Vector2Dim tempsubtract = new Vector2Dim(tempX, tempY);
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
        Vector2Dim subtraction = tempsubtract.rotateBy(Math.toRadians(tempPose[2]));

        Pose2d pose = new Pose2d(
                tempPose[0] - subtraction.x,
                tempPose[1] - subtraction.y,
                Math.toRadians(tempPose[2] - turretBearing)
        );

        return pose;
    }
}
