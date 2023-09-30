package org.firstinspires.ftc.teamcode.apriltag;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.testops.AprilTagTestOpMode;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagDetectionThread implements Runnable{
    private AprilTagTestOpMode opMode;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private boolean isRunning = true;
    static  double INCHES_PER_METER = 39.3701;

    static   double fx = 578.272;
    static   double fy = 578.272;
    static  double cx = 402.145;
    static  double cy = 221.506;

    private Pose robotPose = new Pose(0, 0, 0);

    ArrayList<AprilTagDetection> detections;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public AprilTagDetectionThread(AprilTagTestOpMode opMode,Telemetry telemetry, HardwareMap hardwareMap) {
        this.opMode = opMode;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline( fx, fy, cx, cy);

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
    public void stopDetection() {
        isRunning = false;
    }

    public ArrayList<AprilTagDetection> getDetections() {

        return detections;
    }

    public Pose getRobotPose() {
        return robotPose;
    }
    public OpenCvCamera getCamera() {
        return camera;
    }
    @Override
    public void run() {




        while (isRunning && !Thread.currentThread().isInterrupted()) {
          detections = aprilTagDetectionPipeline.getDetectionsUpdate();
//            telemetry.addLine(String.format("Robot Pose: %s", robotPose));
            if (detections != null) {
//                telemetry.addData("FPS: %.2f", camera.getFps());
//                telemetry.addData("Overhead ms: %.2f", camera.getOverheadTimeMs());
//                telemetry.addData("Pipeline ms: %.2f", camera.getPipelineTimeMs());

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

//                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//                        telemetry.addLine(String.format("Translation X: %.2f inches", detection.pose.x * INCHES_PER_METER));
//                        telemetry.addLine(String.format("Translation Y: %.2f inches", detection.pose.y * INCHES_PER_METER));
//                        telemetry.addLine(String.format("Translation Z: %.2f inches", detection.pose.z * INCHES_PER_METER));
//                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
//                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
//                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));


                        robotPose = new Pose(
                                AprilTagFieldConstants.getTagPose(detection.id).x() +  ((3 * ((detection.pose.z * INCHES_PER_METER)) / 2) - 1) + 9 ,
                                AprilTagFieldConstants.getTagPose(detection.id).y() + -detection.pose.x * INCHES_PER_METER - 1,
                                AprilTagFieldConstants.getTagPose(detection.id).z().deg() + rot.firstAngle + 180);



                    }
                }


//                telemetry.update();
            }

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    }




