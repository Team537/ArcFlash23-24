package org.firstinspires.ftc.teamcode.objectdetection;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.PropDetectionPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public class DetectorCamera {

    private OpenCvCamera webcam;
    private ColorDetectionPipeline colorDetectionPipeline;
    private PropDetectionPipeline propDetectionPipeline;
    private List<ColorDetectionPipeline.ColorRegion> detectedRegions;
    private CameraState currentState;

    public DetectorCamera(RobotHardware robot) {
        webcam = robot.webcam1;
        colorDetectionPipeline = new ColorDetectionPipeline();
        propDetectionPipeline = new PropDetectionPipeline();


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

    public ColorDetectionPipeline.ColorRegion getLastDetectedRegion() {
        return detectedRegions.get(detectedRegions.size() - 1);
    }

    public Point getPropCenter(){
        return propDetectionPipeline.getObjectCenter();
    }

    public void setColorDetect() {
        webcam.setPipeline(colorDetectionPipeline);
        currentState = CameraState.COLOR_DETECT;

    }
    public void setPropDetect(){
        webcam.setPipeline(propDetectionPipeline);
        currentState = CameraState.PROP_DETECT;
    }

    public void loop() {

        if(currentState == CameraState.COLOR_DETECT) {
            detectedRegions = colorDetectionPipeline.getColorRegions();

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
        else if(currentState == CameraState.PROP_DETECT){
            telemetry.addData("Prop Center X", propDetectionPipeline.getObjectCenter().x);
            telemetry.addData("Prop Center Y", propDetectionPipeline.getObjectCenter().y);
        }
    }

    public enum CameraState{
        COLOR_DETECT,
        PROP_DETECT
    }
}
