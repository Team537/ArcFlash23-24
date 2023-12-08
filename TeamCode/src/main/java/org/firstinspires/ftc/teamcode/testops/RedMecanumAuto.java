package org.firstinspires.ftc.teamcode.testops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.manipulator.Deposit;
import org.firstinspires.ftc.teamcode.pipelines.BluePropDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.RedPropDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


//@Autonomous (name = "Red Mecanum Auto")
public class RedMecanumAuto extends LinearOpMode {

    DcMotor frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
    DcMotor backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
    DcMotor frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
    DcMotor backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

@Override
    public void runOpMode() {




        while(!isStarted() && !isStopRequested()) {

            frontLeftMotor.setPower(0.1);
            backLeftMotor.setPower(0.1);
            frontRightMotor.setPower(-0.1);
            backRightMotor.setPower(-0.1);



        }

    }




    public enum PropState{
        LEFT,
        CENTER,
        RIGHT
    }




}
