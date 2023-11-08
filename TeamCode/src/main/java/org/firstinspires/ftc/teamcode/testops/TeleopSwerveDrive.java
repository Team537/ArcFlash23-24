package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.manipulator.Deposit;
import org.firstinspires.ftc.teamcode.manipulator.Intake;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;

@Config
@TeleOp(name = "Test Swerve Drive")
public class TeleopSwerveDrive extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private Intake intake;
    private Deposit deposit;


    private GamepadEx gamepadEx;
    private static double MAX_X_SPEED = 5.0;
    private static double MAX_Y_SPEED = 5.0;
    private static double MAX_TURN_SPEED = Math.PI/4;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.AUTO = false;


        robot.init(hardwareMap, telemetry);
        deposit = new Deposit(robot);
        gamepadEx = new GamepadEx(gamepad1);
        drivetrain = new SwerveDrivetrain(robot);

        intake = new Intake(robot);

        robot.enabled = true;


//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

        drivetrain.read();

    }

    @Override
     public void run() {

        robot.startIMUThread(this);
        drivetrain.driveVelocity(new ChassisSpeeds(
                gamepadEx.getLeftY() * MAX_X_SPEED,
                gamepadEx.getLeftX() * MAX_Y_SPEED,
                gamepadEx.getRightX() * MAX_TURN_SPEED
        ),new Rotation2d(robot.getAngle()));


        drivetrain.updateModules();
        telemetry.addData("Angle", robot.getAngle());
        telemetry.addData("Swerve", drivetrain.getTelemetry());
        telemetry.addData("Swerve Module States", drivetrain.getSwerveModuleStates());
        telemetry.update();

        if(gamepadEx.getButton(GamepadKeys.Button.A)) {
            intake.run();
        } else {
            intake.stop();
        }

        if(gamepadEx.getButton(GamepadKeys.Button.B)){
            deposit.setHighPosition();
        }

        if(gamepadEx.getButton(GamepadKeys.Button.X)){
            deposit.setDownPosition();
        }

        intake.loop();
        deposit.periodic();
//        robot.clearBulkCache();
        telemetry.addData("Intake State", intake.getIntakeState());
        telemetry.addData("Slide State", deposit.getCurrentSlideState());
        telemetry.addData("Slide 1 Position", deposit.getSlideMotor1Position());
        telemetry.addData("Slide 2 Position", deposit.getSlideMotor2Position());


    }
}
