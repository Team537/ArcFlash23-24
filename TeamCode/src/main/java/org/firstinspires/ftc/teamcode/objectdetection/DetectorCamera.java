package org.firstinspires.ftc.teamcode.objectdetection;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pipelines.BluePropDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.RedPropDetectionPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public class DetectorCamera {

    private OpenCvCamera webcam;
    private PixelDetectionPipeline pixelDetectionPipeline;
    private RedPropDetectionPipeline redPropDetectionPipeline;
    private BluePropDetectionPipeline bluePropDetectionPipeline;
    private List<PixelDetectionPipeline.Pixel> detectedPixels;
    private CameraState currentState;

    public DetectorCamera(RobotHardware robot) {
        webcam = robot.webcam1;

        redPropDetectionPipeline = new RedPropDetectionPipeline();
        bluePropDetectionPipeline = new BluePropDetectionPipeline();
        pixelDetectionPipeline = new PixelDetectionPipeline();


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

    public PixelDetectionPipeline.Pixel getLastDetectedRegion() {
        return detectedPixels.get(detectedPixels.size() - 1);
    }

    public Point getPropCenter(){
        return redPropDetectionPipeline.getObjectCenter();
    }

    public void setPixelDetect(){
        webcam.setPipeline(pixelDetectionPipeline);
        currentState = CameraState.PIXEL_DETECT;
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

        if(currentState == CameraState.PIXEL_DETECT) {
            detectedPixels = pixelDetectionPipeline.getDetectedPixels();

            telemetry.addData("Detected Regions", detectedPixels.size());
            for (PixelDetectionPipeline.Pixel region : detectedPixels) {
                telemetry.addData("Region Center: %6.1f", region.center);
                telemetry.addData("Region Width: %6.1f", region.boundingRect.width);
                telemetry.addData("Region Height: %6.1f", region.boundingRect.height);
                telemetry.addData("Region Area: %6.1f", region.boundingRect.area());
                telemetry.addData("Region X: %6.1f", region.center.x);
                telemetry.addData("Region Y: %6.1f", region.center.y);
            }
        }
        else if(currentState == CameraState.RED_PROP_DETECT){
            telemetry.addData("Prop Center X", redPropDetectionPipeline.getObjectCenter().x);
            telemetry.addData("Prop Center Y", redPropDetectionPipeline.getObjectCenter().y);
        }
        else if(currentState == CameraState.BLUE_PROP_DETECT){
            telemetry.addData("Prop Center X", bluePropDetectionPipeline.getObjectCenter().x);
            telemetry.addData("Prop Center Y", bluePropDetectionPipeline.getObjectCenter().y);
        }
    }

    public enum CameraState{
        PIXEL_DETECT,
        RED_PROP_DETECT,
        BLUE_PROP_DETECT
    }
}
