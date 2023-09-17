package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Test Trajectory")
public class TrajectoryTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private Trajectory trajectory;
    private GamepadEx gamepadEx;
    private static double MAX_X_SPEED = 5.0;
    private static double MAX_Y_SPEED = 5.0;
    private static double MAX_TURN_SPEED = 180;
    private RamseteController follower = new RamseteController(0,1 );
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder centerEncoder;
    private OdometrySubsystem odometry;
    private HolonomicOdometry odo;

    static final double TICKS_TO_INCHES = 15.3;
    static final double TRACKWIDTH = 13.7;
    static final double CENTER_WHEEL_OFFSET = 2.4;



    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.AUTO = false;

        leftEncoder = robot.parallelPod;
        rightEncoder = robot.perpindicularPod;
        centerEncoder = robot.centerPod;



        robot.init(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        drivetrain = new SwerveDrivetrain(robot);
        gamepadEx = new GamepadEx(gamepad1);

        leftEncoder.setDistancePerPulse(TICKS_TO_INCHES);
        rightEncoder.setDistancePerPulse(TICKS_TO_INCHES);
        centerEncoder.setDistancePerPulse(TICKS_TO_INCHES);

         odo = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                centerEncoder::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry = new OdometrySubsystem(odo);





        generateTrajectory();
        robot.enabled = true;



//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

        drivetrain.read();

    }

    @Override
    public void run() {

    while(opModeIsActive()) {
        Trajectory.State state = trajectory.sample(getRuntime());
        ChassisSpeeds speeds = follower.calculate(odometry.getPose(), state);

        drivetrain.driveVelocity(speeds, Rotation2d.fromDegrees(robot.getAngle()));
        drivetrain.updateModules();
        telemetry.update();

//        robot.clearBulkCache();
    }
    }

    public void generateTrajectory() {


        Pose2d start = new Pose2d(0, 0,
                Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d( 10 / 39.37,  0,
                Rotation2d.fromDegrees(0));

        List<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(0 ,5 / 39.37));


        TrajectoryConfig config = new TrajectoryConfig((12), (12));
        config.setReversed(false);

         trajectory = TrajectoryGenerator.generateTrajectory(
                start,
                interiorWaypoints,
                end,
                config);
    }
}
