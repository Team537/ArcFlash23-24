package org.firstinspires.ftc.teamcode.objectdetection;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pipelines.BluePropDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.RedPropDetectionPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public class DetectorCamera {

    private OpenCvCamera webcam;
    private ColorDetectionPipeline colorDetectionPipeline;
    private RedPropDetectionPipeline redPropDetectionPipeline;
    private BluePropDetectionPipeline bluePropDetectionPipeline;
    private List<ColorDetectionPipeline.ColorRegion> detectedRegions;
    private CameraState currentState;

    public DetectorCamera(RobotHardware robot) {
        webcam = robot.webcam1;
        colorDetectionPipeline = new ColorDetectionPipeline();
        redPropDetectionPipeline = new RedPropDetectionPipeline();


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
        return redPropDetectionPipeline.getObjectCenter();
    }

    public void setColorDetect() {
        webcam.setPipeline(colorDetectionPipeline);
        currentState = CameraState.COLOR_DETECT;

    }
    public void setRedPropDetect(){
        webcam.setPipeline(redPropDetectionPipeline);
        currentState = CameraState.RED_PROP_DETECT;
    }

    public void setBluePropDetect(){
        webcam.setPipeline(bluePropDetectionPipeline);
        currentState = CameraState.BLUE_PROP_DETECT;
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
        else if(currentState == CameraState.RED_PROP_DETECT){
            telemetry.addData("Prop Center X", redPropDetectionPipeline.getObjectCenter().x);
            telemetry.addData("Prop Center Y", redPropDetectionPipeline.getObjectCenter().y);
        }
    }

    public enum CameraState{
        COLOR_DETECT,
        RED_PROP_DETECT,
        BLUE_PROP_DETECT
    }
}
