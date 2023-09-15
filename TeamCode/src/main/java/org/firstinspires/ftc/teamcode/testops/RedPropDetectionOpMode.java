package org.firstinspires.ftc.teamcode.testops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.pipelines.RedPropDetectionPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Red Prop Detection")
public class RedPropDetectionOpMode extends CommandOpMode {
    private OpenCvCamera camera;
    private RedPropDetectionPipeline pipeline;



    @Override
    public void initialize() {
        // Initialize the OpenCV pipeline
        pipeline = new RedPropDetectionPipeline();

        // Connect to the camera using OpenCvCameraFactory
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set the pipeline on the camera
        camera.setPipeline(pipeline);
        // Start streaming the camera feed when the OpMode is initialized
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    @Override
    public void run() {
        while (opModeIsActive()) {
            // Access the detected object's center from the pipeline
            Point objectCenter = pipeline.getObjectCenter();

            // Print the object's center coordinates to the telemetry
            telemetry.addData("Object Center X", objectCenter.x);
            telemetry.addData("Object Center Y", objectCenter.y);
            telemetry.update();
        }
    }


}
