package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagDetectionThread;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagFieldConstants;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Config
@TeleOp(name = "April Tag Detection")
public class AprilTagTestOpMode extends CommandOpMode {
    private AprilTagDetectionThread aprilTagDetectionThread;

    private ArrayList<AprilTagDetection> detections;
    static  double INCHES_PER_METER = 39.3701;


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    aprilTagDetectionThread = new AprilTagDetectionThread(this,telemetry,hardwareMap);
        Thread thread = new Thread(aprilTagDetectionThread);
        thread.start();
    }

    @Override
    public void run() {
        FtcDashboard.getInstance().startCameraStream(aprilTagDetectionThread.getCamera(), 0);
        telemetry.setMsTransmissionInterval(50);

        detections = aprilTagDetectionThread.getDetections();

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

        telemetry.addData("Pose", aprilTagDetectionThread.getRobotPose());

        telemetry.update();


    }



}
