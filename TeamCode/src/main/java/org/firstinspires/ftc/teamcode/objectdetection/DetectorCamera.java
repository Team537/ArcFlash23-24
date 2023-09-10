package org.firstinspires.ftc.teamcode.objectdetection;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public class DetectorCamera {

    private OpenCvCamera webcam;
    private ColorDetectionPipeline pipeline;
    private List<ColorDetectionPipeline.ColorRegion> detectedRegions;

    public DetectorCamera(RobotHardware robot) {
        webcam = robot.webcam1;
        pipeline = new ColorDetectionPipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

    }

    public void loop() {
        detectedRegions = pipeline.getColorRegions();

        telemetry.addData("Detected Regions", detectedRegions.size());
        for (ColorDetectionPipeline.ColorRegion region : detectedRegions) {
            telemetry.addData("Region Center: %6.1f", region.center);
            telemetry.addData("Region Width: %6.1f", region.boundingRect.width);
            telemetry.addData("Region Height: %6.1f", region.boundingRect.height);
            telemetry.addData("Region Area: %6.1f", region.boundingRect.area());
            telemetry.addData("Region X: %6.1f", region.relativePosition.x);
            telemetry.addData("Region Y: %6.1f", region.relativePosition.y);
        }

    }
}
