package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import java.util.List;

@TeleOp

@Config

public class AprilTagTesting extends OpMode {
    public WebcamName cameraOfDoom;
    VisionPortal visionPortal;
    AprilTagProcessor myAprilTagProcessor;
    AprilTagLibrary tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
    public void init(){
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cameraOfDoom")) // or use .setCamera(InternalCameraDirection.BACK)
                .addProcessor(myAprilTagProcessor)
                .setLiveViewContainerId(R.id.cameraMonitorViewId)
                .build();



    }

    @Override
    public void loop() {
        List<AprilTagDetection> myAprilTagDetections; // list of all detections
        int myAprilTagIdCode; // ID code of current detection, in for() loop
        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        // Cycle through through the list and process each AprilTag.
        for (AprilTagDetection myAprilTagDetection : myAprilTagDetections) {
            if (myAprilTagDetection.metadata != null) {
                myAprilTagIdCode = myAprilTagDetection.id;
                double myTagPoseX = myAprilTagDetection.ftcPose.x;
                double myTagPoseY = myAprilTagDetection.ftcPose.y;
                double myTagPoseZ = myAprilTagDetection.ftcPose.z;
                double myTagPosePitch = myAprilTagDetection.ftcPose.pitch;
                double myTagPoseRoll = myAprilTagDetection.ftcPose.roll;
                double myTagPoseYaw = myAprilTagDetection.ftcPose.yaw;
                telemetry.addData("id code", myAprilTagIdCode);
                telemetry.addData("Pose x", myTagPoseX);
                telemetry.addData("Pose Y", myTagPoseY);
                telemetry.addData("Pose z", myTagPoseZ);
                telemetry.addData("pose pitch", myTagPosePitch);
                telemetry.addData("pose roll", myTagPoseRoll);
                telemetry.addData("pose yaw", myTagPoseYaw);




            }
        }
    }
}
