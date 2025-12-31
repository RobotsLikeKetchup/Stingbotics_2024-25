package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagExample extends OpMode {
    AprilTag aprilTag = new AprilTag();

    @Override
    public void init() {
        aprilTag.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        //update
        aprilTag.update();
        AprilTagDetection id20 = aprilTag.getTagByID(20);
        //display
        aprilTag.displayTelemetry(id20);
    }
}
