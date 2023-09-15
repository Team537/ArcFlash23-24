package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;

import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.trajectory.Trajectory;
import me.wobblyyyy.pathfinder2.trajectory.spline.SplineBuilderFactory;

@Config
@Autonomous(name = "Test PathFinder Spline")
public class AutoSwerveSpline extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private PFinder pathfinder;
    private Trajectory trajectory;
    private static final double SPEED = 0.5;
    private static final double TOLERANCE = 1.0;
    private static final Angle ANGLE_TOLERANCE = Angle.fromDeg(15);
    private SplineBuilderFactory factory;




    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        robot.init(hardwareMap, telemetry);

        pathfinder = new PFinder(robot);

        factory = new SplineBuilderFactory()
                .setStep(0.1)
                .setSpeed(SPEED)
                .setTolerance(TOLERANCE)
                .setAngleTolerance(ANGLE_TOLERANCE);

        trajectory = factory.builder().add(new PointXYZ(0, 0, 0))
                .add(new PointXYZ(2, 2, 0)).add(new PointXYZ(3, 4, 0)).build();

        robot.enabled = true;
//
//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

        waitForStart();

        if(opModeIsActive()) pathfinder.runTrajectory(trajectory);


        while (opModeIsActive()) {
            pathfinder.loopAuto();
            telemetry.update();
//            robot.clearBulkCache();
        }


    }
}
