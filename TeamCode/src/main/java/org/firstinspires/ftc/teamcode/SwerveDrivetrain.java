package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Globals.*;

@Config
public class SwerveDrivetrain {
    public SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModule[] modules;

    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;
    private final double R;
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

    /**
     * Swerve Drive Declaration
     * @param robot Robot Hardware Map
     * */
    public SwerveDrivetrain(RobotHardware robot) {
        frontLeftModule = new SwerveModule(robot.frontLeftMotor, robot.frontLeftServo, new AbsoluteAnalogEncoder(robot.frontLeftEncoder, 3.3).zero(frontLeftOffset).setInverted(true));
        backLeftModule = new SwerveModule(robot.backLeftMotor, robot.backLeftServo, new AbsoluteAnalogEncoder(robot.backLeftEncoder, 3.3).zero(backLeftOffset).setInverted(true));
        backRightModule = new SwerveModule(robot.backRightMotor, robot.backRightServo, new AbsoluteAnalogEncoder(robot.backRightEncoder, 3.3).zero(backRightOffset).setInverted(true));
        frontRightModule = new SwerveModule(robot.frontRightMotor, robot.frontRightServo, new AbsoluteAnalogEncoder(robot.frontRightEncoder, 3.3).zero(frontRightOffset).setInverted(true));

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = hypot(TRACK_WIDTH, WHEEL_BASE);
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

    public String getSwerveServoPowers(){
        return frontLeftModule.getServoPower() + "\n" +
                backLeftModule.getServoPower() + "\n" +
                frontRightModule.getServoPower()  + "\n" +
                backRightModule.getServoPower()  + "\n";
    }
    public String getVelocities(){
        return frontLeftModule.getVelocity() + "\n" +
                backLeftModule.getVelocity() + "\n" +
                frontRightModule.getVelocity()  + "\n" +
                backRightModule.getVelocity()  + "\n";
    }
}