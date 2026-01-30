package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.List;

@Configurable
public class InfernoTower {

    private VisionPortal visionPortal;

    private AprilTagProcessor aprilTagProcessor;
    private List<AprilTagDetection> aprilTagDetections;
    private AprilTagDetection aprilTagDetection;


    private int cameraWidth = 640, cameraHeight = 480; // TODO: set proper aspect ratio and resolution if needed


    public InfernoTower(HardwareMap hwMap, HashMap<String, String> config) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, config.get("webcam1")))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    public List<AprilTagDetection> detectAprilTags() {
        aprilTagDetections = aprilTagProcessor.getDetections();
        return aprilTagDetections;
    }
}
