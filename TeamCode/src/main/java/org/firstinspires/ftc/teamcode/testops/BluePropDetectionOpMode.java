package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.BluePropDetectionPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Config
@TeleOp(name = "Blue Prop Detection")
public class BluePropDetectionOpMode extends CommandOpMode {
    private OpenCvCamera camera;
    private BluePropDetectionPipeline pipeline;
    private PropState currentState;


    @Override
    public void initialize() {
        // Initialize the OpenCV pipeline
        pipeline = new BluePropDetectionPipeline();

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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        while (opModeIsActive()) {
            // Access the detected object's center from the pipeline
            Point objectCenter = pipeline.getObjectCenter();

            if(objectCenter.x > 600) {
                currentState = PropState.RIGHT;
            } else if(objectCenter.x > 300 && objectCenter.x < 600){
                currentState = PropState.CENTER;
            } else if(objectCenter.x < 300) {
                currentState = PropState.LEFT;
            }

            // Print the object's center coordinates to the telemetry
            telemetry.addData("Object Center X", objectCenter.x);
            telemetry.addData("Object Center Y", objectCenter.y);
            telemetry.addData("Prop Position", currentState.toString());
            telemetry.update();
        }
    }


    public enum PropState {
        LEFT,
        CENTER,
        RIGHT,

    }

}
