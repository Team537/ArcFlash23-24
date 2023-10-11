package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.manipulator.Deposit;
import org.firstinspires.ftc.teamcode.pipelines.BluePropDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
public class BlueMecanumAuto extends LinearOpMode {

    private Deposit deposit;
    private final RobotHardware robot = RobotHardware.getInstance();
    private OpenCvCamera camera;
    private BluePropDetectionPipeline pipeline;
    private PropState currentState;

    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

    TrajectorySequence centerPath = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
            .forward(36)
            .addDisplacementMarker(1, () -> {
                deposit.latchOpen();
            })
            .strafeLeft(75)
            .turn(Math.toRadians(90))
            .build()
            ;

    TrajectorySequence leftPath = drivetrain.trajectorySequenceBuilder(new Pose2d(38, 0, Math.toRadians(0)))
            .forward(33)
            .turn(Math.toRadians(-90))
            .forward(3)
            .addDisplacementMarker(1, () -> {
                deposit.latchOpen();
            })
            .back(72)
            .build()
            ;
    TrajectorySequence rightPath = drivetrain.trajectorySequenceBuilder(new Pose2d(38, 0, Math.toRadians(0)))
            .forward(33)
            .turn(Math.toRadians(90))
            .forward(3)
            .addDisplacementMarker(1, () -> {
                deposit.latchOpen();
            })
            .forward(78)
            .build()
            ;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
        deposit = new Deposit(robot);

        pipeline = new BluePropDetectionPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcame 1"), cameraMonitorViewId
        );

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });



        while(!isStarted() && !isStopRequested()) {

            Point objectCenter = pipeline.getObjectCenter();

            if(objectCenter.x > 500) {
                currentState = PropState.RIGHT;
            } else if(objectCenter.x > 250 && objectCenter.x < 500) {
                currentState = PropState.CENTER;
            } else if(objectCenter.x < 250) {
                currentState = PropState.RIGHT;
            }
            sleep(20);
        }

        switch (currentState) {
            case LEFT: waitForStart();
                drivetrain.followTrajectorySequence(leftPath);
                break;

            case CENTER: waitForStart();
                drivetrain.followTrajectorySequence(centerPath);
                break;

            case RIGHT: waitForStart();
                drivetrain.followTrajectorySequence(rightPath);
                break;

            default:
                break;
        }


    }




    public enum PropState{
        LEFT,
        CENTER,
        RIGHT
    }




}

