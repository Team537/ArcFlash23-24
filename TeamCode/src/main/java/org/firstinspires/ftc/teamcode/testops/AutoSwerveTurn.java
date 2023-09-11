package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;

import me.wobblyyyy.pathfinder2.geometry.PointXYZ;

@Config
@Autonomous(name = "Test PathFinder Turn")
public class AutoSwerveTurn extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private PFinder pathfinder;






    @Override
    public void runOpMode() {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        robot.init(hardwareMap, telemetry);

        pathfinder = new PFinder(robot);


        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        waitForStart();

        if (opModeIsActive()) pathfinder.goToPoint( new PointXYZ(0,0,90));


        while(opModeIsActive()) {
            pathfinder.loop();

            telemetry.update();

            robot.clearBulkCache();
        }

    }
}
