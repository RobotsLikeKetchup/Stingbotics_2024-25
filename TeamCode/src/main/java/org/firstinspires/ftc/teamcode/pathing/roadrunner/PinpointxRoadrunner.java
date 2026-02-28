package org.firstinspires.ftc.teamcode.pathing.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pathing.StingLocalizer;


//I'm using this localizer, with inputs as the pinpoint, because the pinpoint is just straight not working
//annoying but this will work around it.
@Config
public class PinpointxRoadrunner implements StingLocalizer {
    public static class Params {
        public double parYTicks = -827.7072066692305; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -256.4012498211663; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    private int lastParPos, lastPerpPos;
    private double lastTime;
    private Rotation2d lastHeading;

    private final double inPerTick = 0.004010101;
    private final double lateralInPerTick = 0.00406819;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;
    private Pose2d pose;
    private PoseVelocity2d poseVel;

    ElapsedTime timer;
    public GoBildaPinpointDriver odometry;

    public PinpointxRoadrunner(Pose2d initialPose, GoBildaPinpointDriver odometry, ElapsedTime timer) {

        this.odometry = odometry;
        this.timer = timer;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);

        pose = initialPose;
    }

    //for the get and setPose methods, I'm converting from the Roadrunner Pose2d to the FTC SDK Pose 2D, which is better and easier to use
    //again, ideally I just code my own localizer but rn I don't have much time.
    public void setPose(Pose2D pose) {
        this.pose = new Pose2d(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS));
    }

    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.INCH, pose.position.x, pose.position.y, AngleUnit.RADIANS, pose.heading.toDouble());
    }

    public void update() {

        odometry.update();

        //TODO: I've set these both negative, but reverse if needed...
        int parTicks = -1 * odometry.getEncoderX();
        int perpTicks = odometry.getEncoderY();

        Rotation2d heading = Rotation2d.exp(odometry.getHeading(AngleUnit.RADIANS));

        double headingVel = odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        if (!initialized) {
            initialized = true;

            lastTime = timer.seconds();
            lastParPos = parTicks;
            lastPerpPos = perpTicks;
            lastHeading = heading;

            poseVel = new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            return;
        }

        double loopTime = timer.seconds() - lastTime;

        int parPosDelta = parTicks - lastParPos;
        int perpPosDelta = perpTicks - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        //this method of calculating velocity is kind of jank, I need to see if this actually is any good...
        double parVelocity = parPosDelta / loopTime;
        double perpVelocity = perpPosDelta / loopTime;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                //subtract the heading*radius of the deadwheel(cause it rotates when it turns the deadwheels)
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parVelocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpVelocity - PARAMS.perpXTicks * headingVel,
                        }).times(lateralInPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parTicks;
        lastPerpPos = perpTicks;
        lastHeading = heading;

        pose = pose.plus(twist.value());
        poseVel = twist.velocity().value();
    }

    @Override
    public double[] getPoseDouble() {
        return new double[] {
                pose.position.x,
                pose.position.y,
                pose.heading.toDouble()
        };
    }

    public double[] getVelocity() {
        return new double[] {poseVel.linearVel.x, poseVel.linearVel.y, poseVel.angVel};
    }

}
