package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

public class AprilTagTesting extends OpMode {
    VisionPortal visionPortal;
    AprilTagProcessor myAprilTagProcessor;
    AprilTagLibrary tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
    public void init(){
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cameraOfDoom")) // or use .setCamera(InternalCameraDirection.BACK)
                .addProcessor(myAprilTagProcessor)
                .build();
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
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
                telemetry.addData("id code", myAprilTagIdCode);
            }
        }
    }
}
