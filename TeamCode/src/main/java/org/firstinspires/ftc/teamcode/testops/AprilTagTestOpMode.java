package org.firstinspires.ftc.teamcode.testops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name = "April Tag Detection")
public class AprilTagTestOpMode extends CommandOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double INCHES_PER_METER = 3.28084 * 12;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.433;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    @Override
    public void initialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void run() {
        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                } else {
                    numFramesWithoutDetection = 0;

                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f inches", detection.pose.x * INCHES_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f inches", detection.pose.y * INCHES_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f inches", detection.pose.z * INCHES_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                    }
                }

                telemetry.update();
            }

            sleep(20);
        }
    }
}
