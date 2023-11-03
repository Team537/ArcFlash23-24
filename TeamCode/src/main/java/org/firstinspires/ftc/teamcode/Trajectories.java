package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories {

    public SwerveDrivetrain drive;

    public TrajectorySequence blue1Spike1 = drive.trajectorySequenceBuilder(new Pose2d(-35, 60, -Math.PI/2))
            .forward(25)
            .turn(Math.toRadians(-90))
            .back(21)
            .waitSeconds(1)
            .back(59)
            .turn(Math.toRadians(180))
            .forward(2)
            .build();

    public TrajectorySequence blue1Spike2 = drive.trajectorySequenceBuilder(new Pose2d(-35, 60, -Math.PI/2))
            .forward(25)
            .turn(Math.toRadians(90))
            .forward(83)
            .build();

    public TrajectorySequence blue1Spike3 = drive.trajectorySequenceBuilder(new Pose2d(-35, 60, -Math.PI/2))
            .forward(25)
            .turn(Math.toRadians(-90))
            .forward(3)
            .back(83)
            .turn(Math.toRadians(180))
            .forward(2)
            .build();

    public TrajectorySequence blue2Spike1 = drive.trajectorySequenceBuilder(new Pose2d(13, 60, -Math.PI/2))
            .forward(28)
            .turn(Math.toRadians(-90))
            .back(20)
            .waitSeconds(1)
            .back(12)
            .turn(Math.toRadians(180))
            .forward(2)
            .build();

    public TrajectorySequence blue2Spike2 = drive.trajectorySequenceBuilder(new Pose2d(13, 60, -Math.PI/2))
            .forward(25)
            .turn(Math.toRadians(90))
            .forward(35)
            .build();

    public TrajectorySequence blue2Spike3 = drive.trajectorySequenceBuilder(new Pose2d(13, 60, -Math.PI/2))
            .forward(30)
            .turn(Math.toRadians(-90))
            .forward(3)
            .back(35)
            .turn(Math.toRadians(180))
            .forward(2)
            .build();



    public TrajectorySequence red1Spike1 = drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.PI/2))
            .forward(25)
            .turn(Math.toRadians(90))
            .forward(3)
            .back(83)
            .turn(Math.toRadians(180))
            .forward(2)
            .build();

    public TrajectorySequence red1Spike2 = drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.PI/2))
            .forward(25)
            .turn(Math.toRadians(-90))
            .forward(83)
            .build();

    public TrajectorySequence red1Spike3 = drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.PI/2))
            .forward(25)
            .turn(Math.toRadians(90))
            .back(21)
            .waitSeconds(1)
            .back(59)
            .turn(Math.toRadians(180))
            .forward(2)
            .build();

    public TrajectorySequence red2Spike1 = drive.trajectorySequenceBuilder(new Pose2d(13, -60, Math.PI/2))
            .forward(30)
            .turn(Math.toRadians(90))
            .forward(3)
            .back(35)
            .turn(Math.toRadians(180))
            .forward(2)
            .build();

    public TrajectorySequence red2Spike2 = drive.trajectorySequenceBuilder(new Pose2d(13, -60, Math.PI/2))
            .forward(25)
            .turn(Math.toRadians(-90))
            .forward(35)
            .build();

    public TrajectorySequence red2Spike3 = drive.trajectorySequenceBuilder(new Pose2d(13, -60, Math.PI/2))
            .forward(28)
            .turn(Math.toRadians(90))
            .back(20)
            .waitSeconds(1)
            .back(12)
            .turn(Math.toRadians(180))
            .forward(2)
            .build();

    public TrajectorySequence getBlue1Spike1(){
        return blue1Spike1;
    }
    public TrajectorySequence getBlue1Spike2(){
        return blue1Spike2;
    }
    public TrajectorySequence getBlue1Spike3(){
        return blue1Spike3;
    }
    public TrajectorySequence getBlue2Spike1(){
        return blue2Spike1;
    }
    public TrajectorySequence getBlue2Spike2(){
        return blue2Spike2;
    }
    public TrajectorySequence getBlue2Spike3(){
        return blue2Spike3;
    }
    public TrajectorySequence getRed1Spike1(){
        return red1Spike1;
    }
    public TrajectorySequence getRed1Spike2(){
        return red1Spike2;
    }
    public TrajectorySequence getRed1Spike3(){
        return red1Spike3;
    }
    public TrajectorySequence getRed2Spike1(){
        return red2Spike1;
    }
    public TrajectorySequence getRed2Spike2(){
        return red2Spike2;
    }
    public TrajectorySequence getRed2Spike3(){
        return red2Spike3;
    }



}
