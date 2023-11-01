package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.SwerveDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Globals.*;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class SwerveDrivetrain extends SwerveDrive {
    public SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModule[] modules;

    private TrajectorySequenceRunner trajectorySequenceRunner;
    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;


    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    private TrajectoryFollower follower;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private final double R = hypot(TRACK_WIDTH, WHEEL_BASE);;
    public static double frontLeftOffset = 0, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = 0;

    public static boolean maintainHeading = false;

    double[] feedforward_static = new double[4];
    double[] feedforward_acceleration = new double[4];


    double maxPower = 0.0;
    ChassisSpeeds moduleSpeeds = new ChassisSpeeds();

    public final double minPower = 0.1;
    public static double imuOffset = 0.0;

    private boolean fieldOriented = true;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2),
            new Translation2d(TRACK_WIDTH / 2, -WHEEL_BASE / 2),
            new Translation2d(-TRACK_WIDTH / 2, -WHEEL_BASE / 2),
            new Translation2d(-TRACK_WIDTH / 2, WHEEL_BASE / 2)
    );

    private BNO055IMU imu;

    public SwerveDrivetrain(double kV, double kA, double kStatic, double trackWidth, double wheelBase) {
        super(kV, kA, kStatic, trackWidth, wheelBase);


    }

    /**
     * Swerve Drive Declaration
     * @param robot Robot Hardware Map
     * */

    public void init(RobotHardware robot) {



        imu = robot.imu;

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        frontLeftModule = new SwerveModule(robot.frontLeftMotor, robot.frontLeftServo, new AbsoluteAnalogEncoder(robot.frontLeftEncoder, 3.3).zero(frontLeftOffset).setInverted(true));
        backLeftModule = new SwerveModule(robot.backLeftMotor, robot.backLeftServo, new AbsoluteAnalogEncoder(robot.backLeftEncoder, 3.3).zero(backLeftOffset).setInverted(true));
        backRightModule = new SwerveModule(robot.backRightMotor, robot.backRightServo, new AbsoluteAnalogEncoder(robot.backRightEncoder, 3.3).zero(backRightOffset).setInverted(true));
        frontRightModule = new SwerveModule(robot.frontRightMotor, robot.frontRightServo, new AbsoluteAnalogEncoder(robot.frontRightEncoder, 3.3).zero(frontRightOffset).setInverted(true));

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        setLocalizer(new StandardTrackingWheelLocalizer(robot));
    }
    /**
     * Get Absolute Angles from Modules
     * */
    public void read() {
        for (SwerveModule module : modules) module.read();
    }

    /**
     * Set Swerve Pose
     * @param pose Pose in 2d Space including X,Y and Heading components
     * */
    public void set(Pose pose) {
        double x = pose.x, y = pose.y, head = pose.heading;


        //Swerve Kinematics I presume
        double a = x - head * (WHEEL_BASE / R),
                b = x + head * (WHEEL_BASE / R),
                c = y - head * (TRACK_WIDTH / R),
                d = y + head * (TRACK_WIDTH / R);


        //robot-oriented
        if (fieldOriented) {
            feedforward_static = new double[]{0, 0, 0, 0};
            feedforward_acceleration = new double[]{Math.PI / 4, -Math.PI / 4, Math.PI / 4, -Math.PI / 4};
        } else {
            //field-oriented
            feedforward_static = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if (!maintainHeading) feedforward_acceleration = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
        }

        maxPower = MathUtils.max(feedforward_static);
    }
    /**
     * Set Power to Drive Motors and Set Target Angle for Servos
     * */
    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            if (Math.abs(maxPower) > 1) feedforward_static[i] /= maxPower;
            m.setMotorPower(Math.abs(feedforward_static[i]) + ((USE_WHEEL_FEEDFORWARD) ? minPower * Math.signum(feedforward_static[i]) : 0));
            m.setTargetRotation(MathUtils.norm(feedforward_acceleration[i]));
        }
    }

    public void driveVelocity(ChassisSpeeds speeds, Rotation2d gyroAngle){

        moduleSpeeds  = fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, gyroAngle) : speeds;

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(moduleSpeeds);

       modules[0].setDesiredState(states[0]);
       modules[1].setDesiredState(states[1]);
       modules[2].setDesiredState(states[2]);
       modules[3].setDesiredState(states[3]);

    }
  /**
   * Update Swerve Modules
   * */
    public void updateModules() {
        for (SwerveModule m : modules) m.update();
    }

    /**
     * Set Field Oriented
     * @param fieldOriented Field Oriented Boolean
     * */
    public void setFieldOriented(boolean fieldOriented){
        this.fieldOriented = fieldOriented;
    }
    /**
     * Get If Field Oriented
     * @return Field Oriented Boolean
     * */
    public boolean isFieldOriented(){
        return fieldOriented;
    }


   /**
    * Get Swerve Telemetry String
    * @return Individual Module Telemtery Strings
    * */
    public String getTelemetry() {
        return frontLeftModule.getTelemetry("leftFrontModule") + "\n" +
                backLeftModule.getTelemetry("leftRearModule") + "\n" +
                frontRightModule.getTelemetry("rightFrontModule") + "\n" +
                backRightModule.getTelemetry("rightRearModule") + "\n";
    }

    public String getSwerveModuleStates(){
        return frontLeftModule.getState().toString() + "\n" +
                backLeftModule.getState().toString() + "\n" +
                frontRightModule.getState().toString()  + "\n" +
                backRightModule.getState().toString()  + "\n";
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void update(){
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    @Override
    public void setMotorPowers(double frontLeft, double rearLeft, double rearRight, double frontRight) {
        modules[0].setMotorPower(frontLeft);
        modules[1].setMotorPower(frontRight);
        modules[2].setMotorPower(rearLeft);
        modules[3].setMotorPower(rearRight);
    }

    @Override
    public void setModuleOrientations(double frontLeft, double rearLeft, double rearRight, double frontRight) {
        modules[0].setTargetRotation(frontLeft);
        modules[1].setTargetRotation(frontRight);
        modules[2].setTargetRotation(rearLeft);
        modules[3].setTargetRotation(rearRight);
    }

    @Override
    public java.util.List<java.lang.Double> getWheelPositions(){
        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(modules[0].getWheelPosition()));
        wheelPositions.add(encoderTicksToInches(modules[1].getWheelPosition()));
        wheelPositions.add(encoderTicksToInches(modules[2].getWheelPosition()));
        wheelPositions.add(encoderTicksToInches(modules[3].getWheelPosition()));
        return wheelPositions;
    }

    @Override
    public java.util.List<java.lang.Double> getWheelVelocities(){
        List<Double> wheelVelocities = new ArrayList<>();
        wheelVelocities.add(encoderTicksToInches(modules[0].getWheelVelocity()));
        wheelVelocities.add(encoderTicksToInches(modules[1].getWheelVelocity()));
        wheelVelocities.add(encoderTicksToInches(modules[2].getWheelVelocity()));
        wheelVelocities.add(encoderTicksToInches(modules[3].getWheelVelocity()));
        return wheelVelocities;
    }

    @Override
    public java.util.List<java.lang.Double> getModuleOrientations(){
        List<Double> moduleOrientations = new ArrayList<>();
        moduleOrientations.add(encoderTicksToInches(modules[0].getModuleRotation()));
        moduleOrientations.add(encoderTicksToInches(modules[1].getModuleRotation()));
        moduleOrientations.add(encoderTicksToInches(modules[2].getModuleRotation()));
        moduleOrientations.add(encoderTicksToInches(modules[3].getModuleRotation()));
        return moduleOrientations;
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

}