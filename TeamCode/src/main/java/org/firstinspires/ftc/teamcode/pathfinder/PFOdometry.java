package org.firstinspires.ftc.teamcode.pathfinder;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagCamera;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagFieldConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.odometrycore.ThreeWheelOdometry;
import me.wobblyyyy.pathfinder2.robot.AbstractOdometry;
import me.wobblyyyy.pathfinder2.robot.Odometry;
import me.wobblyyyy.pathfinder2.robot.sensors.Encoder;


/**
 * An example implementation of {@link PFOdometry}, which uses
 * {@code OdometryCore} to track a robot's position.
 *
 * @since 2.4.0
 */
public class PFOdometry extends AbstractOdometry {
    // PLACEHOLDER!!!!
    private static final double CPR = 4096;
    private static final double WHEEL_DIAMETER = 0.0;
    private static final double OFFSET_LEFT = 0.0;
    private static final double OFFSET_RIGHT = 0.0;
    private static final double OFFSET_CENTER = 0.0;

    // PLACEHOLDER!!!!!
    private Motor.Encoder leftEncoder;
    private Motor.Encoder rightEncoder;
    private Motor.Encoder centerEncoder;
    private AprilTagCamera camera;

    /**
     * Create a new instance of the {@code Robot} class to demonstrate how
     * {@link PFOdometry} is instantiated.
     */

        public PFOdometry(RobotHardware robotMap, AprilTagCamera camera){
            leftEncoder = robotMap.parallelPod;
            rightEncoder = robotMap.perpindicularPod;
            centerEncoder = robotMap.centerPod;
            this.camera = camera;
        }




        // create a ThreeWheelOdometryProfile to store constants
        ThreeWheelOdometry.ThreeWheelOdometryProfile odometryProfile = new ThreeWheelOdometry.ThreeWheelOdometryProfile(
                CPR,
                WHEEL_DIAMETER,
                OFFSET_LEFT,
                OFFSET_RIGHT,
                OFFSET_CENTER
        );


        ThreeWheelOdometry.EncoderProfile encoderProfile = new ThreeWheelOdometry.EncoderProfile(
                () -> (double) leftEncoder.getPosition(),
                () -> (double) rightEncoder.getPosition(),
                () -> (double) centerEncoder.getPosition()
        );

        // initialize ThreeWheelOdometry
        Odometry odometry = new ThreeWheelOdometry(
                odometryProfile,
                encoderProfile
        );


    @Override
    public PointXYZ getRawPosition() {
        AprilTagDetection detection = camera.getLastDetection();

        //Placeholder
        if(detection.decisionMargin > 30 ){
        odometry.offsetSoPositionIs( new PointXYZ(
                AprilTagFieldConstants.getTagPose(detection.id).x() + detection.ftcPose.x,
                AprilTagFieldConstants.getTagPose(detection.id).y() + detection.ftcPose.y,
                AprilTagFieldConstants.getTagPose(detection.id).z().deg() + detection.ftcPose.yaw));
    }
        PointXYZ position = odometry.getPosition();
        return position;
    }

    public String getTelemetry(){
        return String.format(Locale.ENGLISH,"X Position: .2%f \nY Position: .2%f \nZ Angle: .2%f ", getX(),getY(),getZ().deg());

    }
}

