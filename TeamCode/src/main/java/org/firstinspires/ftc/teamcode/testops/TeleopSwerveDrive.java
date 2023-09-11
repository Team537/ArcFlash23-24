package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;

@Config
@TeleOp(name = "Test PathFinder Swerve Drive")
public class TeleopSwerveDrive extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private PFinder pathfinder;

    private GamepadEx gamepadEx;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.AUTO = false;

        robot.init(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        pathfinder = new PFinder(robot);
        gamepadEx = new GamepadEx(gamepad1);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

    }

    @Override
     public void run() {
        pathfinder.absoluteDrive(
                new Pose(
                        gamepadEx.getLeftX(),
                        gamepadEx.getLeftY(),
                        gamepadEx.getRightX()
                )
        );
        pathfinder.loop();

        telemetry.update();

        robot.clearBulkCache();

    }
}
