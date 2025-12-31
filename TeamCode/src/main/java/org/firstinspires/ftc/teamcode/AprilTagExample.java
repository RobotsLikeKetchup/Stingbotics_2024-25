package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagExample extends OpMode {
    AprilTag aprilTag = new AprilTag();
    Robot robot = new Robot();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap, timer);
        aprilTag.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        //update
        aprilTag.update();
        AprilTagDetection id20 = aprilTag.getTagByID(20);
        //display
        aprilTag.displayTelemetry(id20);
        if(id20 != null && id20.ftcPose != null){
            double bearingError = id20.ftcPose.bearing;
            if(bearingError > 5){
                robot.spin.setPower(0.3);
            } else if (bearingError < -5) {
                robot.spin.setPower(-0.3);
            }else {
                robot.spin.setPower(0);
            }
        }else{
            robot.spin.setPower(0);
        }


    }
}
