package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagFieldConstants;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import kotlin.Unit;

@Config
@TeleOp(name = "April Tag Detection")
public class AprilTagTestOpMode extends CommandOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;
    private static double kV = 0, kA = 0, kStatic = 0;
    static  double INCHES_PER_METER = 39.3701;

    static   double fx = 578.272;
    static   double fy = 578.272;
    static  double cx = 402.145;
    static  double cy = 221.506;

    private Pose robotPose = new Pose(0, 0, 0);

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;

    private GamepadEx gamepadEx;
    private static double MAX_X_SPEED = 5.0;
    private static double MAX_Y_SPEED = 5.0;
    private static double MAX_TURN_SPEED = Math.PI/4;
    ArrayList<AprilTagDetection> detections = new ArrayList<>(5);

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    private double xSpeed;
    private double ySpeed;
    private double omegaSpeed;
    private AprilTagDetection latestDetection;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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

        Globals.AUTO = false;

        robot.init(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        drivetrain = new SwerveDrivetrain(kV,kA,kStatic,TRACK_WIDTH,WHEEL_BASE);
        drivetrain.init(robot);

        robot.enabled = true;

        drivetrain.read();
    }

    @Override
    public void run() {
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry.setMsTransmissionInterval(50);



        while (opModeIsActive()) {



            if(latestDetection != null && gamepadEx.getButton(GamepadKeys.Button.BACK)) {

                    Orientation rot = Orientation.getOrientation(latestDetection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                    xSpeed = ((latestDetection.pose.z*INCHES_PER_METER+15)/MAX_X_SPEED) * 0.5 ;
                    ySpeed = ((-latestDetection.pose.x*INCHES_PER_METER)/MAX_Y_SPEED) * 0.50;
                    omegaSpeed = (Math.toRadians(rot.firstAngle + 180)/MAX_TURN_SPEED) * 0.1;
//                omegaSpeed = 0;




            } else {

                xSpeed = gamepadEx.getLeftY() * MAX_X_SPEED;
                ySpeed = gamepadEx.getLeftX() * MAX_Y_SPEED;
                omegaSpeed = gamepadEx.getRightX() * MAX_TURN_SPEED;
            }

            drivetrain.driveVelocity(new ChassisSpeeds(
                    xSpeed,
                    ySpeed,
                    omegaSpeed
            ),new Rotation2d(robot.getAngle()));


            drivetrain.updateModules();
            telemetry.addData("Angle", robot.getAngle());
            telemetry.addData("Swerve", drivetrain.getTelemetry());
            telemetry.addData("Swerve Module States", drivetrain.getSwerveModuleStates());

            detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            telemetry.addLine(String.format("Robot Pose: %s", robotPose));
            if (detections != null) {
                telemetry.addData("FPS: %.2f", camera.getFps());
                telemetry.addData("Overhead ms: %.2f", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms: %.2f", camera.getPipelineTimeMs());

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


                        robotPose = new Pose(
                                AprilTagFieldConstants.getTagPose(detection.id).x() +  ((3 * ((detection.pose.z * INCHES_PER_METER)) / 2) - 1) + 9 ,
                                AprilTagFieldConstants.getTagPose(detection.id).y() + -detection.pose.x * INCHES_PER_METER - 1,
                                AprilTagFieldConstants.getTagPose(detection.id).z().deg() + rot.firstAngle + 180);

                        latestDetection = detection;
                        telemetry.addData("Latest Detection", latestDetection);
                        telemetry.addData("Latest Detection Speed X", xSpeed );
                        telemetry.addData("Latest Detection Speed Y", ySpeed );
                        telemetry.addData("Latest Detection Speed Omega",omegaSpeed);
                    }
                }


                telemetry.update();
            }

            sleep(20);
        }
    }
}
