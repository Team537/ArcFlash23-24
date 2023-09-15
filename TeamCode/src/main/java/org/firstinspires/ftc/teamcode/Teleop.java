package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.manipulator.Deposit;
import org.firstinspires.ftc.teamcode.manipulator.DroneShooter;
import org.firstinspires.ftc.teamcode.manipulator.Intake;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;

@Config
@TeleOp(name = "😈😈😈😈😈😈")
public class Teleop extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    public static double position;

    private boolean pHeadingLock = true;
    private double targetHeading;

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private PFinder pathfinder;
    private Intake intake;
    private Deposit deposit;
    private DroneShooter droneShooter;

    private SlewRateLimiter forwardLimiter;
    private SlewRateLimiter steerLimiter;
    private final PIDFController headingController = new PIDFController(0.5, 0, 0.1, 0);

    public static double fw_r = 4;
    public static double str_r = 4;
    private boolean lock_robot_heading = false;



    GamepadEx gamepadEx, gamepadEx2;
    Localizer localizer;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = false;
        Globals.USING_IMU = true;
        Globals.USE_WHEEL_FEEDFORWARD = false;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        intake = new Intake(robot);
        deposit = new Deposit(robot);
        droneShooter = new DroneShooter(robot);

        pathfinder = new PFinder(robot);

        robot.enabled = true;
//
//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            timer = new ElapsedTime();
            robot.startIMUThread(this);
            forwardLimiter = new SlewRateLimiter(fw_r);
            steerLimiter = new SlewRateLimiter(str_r);
        }

        robot.read(drivetrain);

        if (gamepad1.right_stick_button && Globals.USING_IMU)
            SwerveDrivetrain.imuOffset = robot.getAngle() + Math.PI;

        double turn = gamepad1.right_stick_x;
        if (Math.abs(turn) > 0.002) {
            lock_robot_heading = false;
        }

        double angleError = normalizeRadians(normalizeRadians(targetHeading) - normalizeRadians(robot.getAngle()));
        double headingCorrection = -headingController.calculate(0, angleError) * 12.4 / robot.getVoltage();

        if (Math.abs(headingCorrection) < 0.01) {
            headingCorrection = 0;
        }

        SwerveDrivetrain.maintainHeading =
                (Math.abs(gamepad1.left_stick_x) < 0.002 &&
                        Math.abs(gamepad1.left_stick_y) < 0.002 &&
                        Math.abs(turn) < 0.002) &&
                        Math.abs(headingCorrection) < 0.02;


        double rotationAmount = (Globals.USING_IMU) ? robot.getAngle() - SwerveDrivetrain.imuOffset : 0;
        Pose drive = new Pose(
                new Point(joystickScalar(gamepad1.left_stick_y, 0.001),
                        joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                lock_robot_heading ? headingCorrection :
                        joystickScalar(turn, 0.01)
        );

        drive = new Pose(
                forwardLimiter.calculate(drive.x),
                steerLimiter.calculate(drive.y),
                drive.heading
        );


        robot.loop(drive, drivetrain);
        robot.write(drivetrain);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

//        robot.clearBulkCache();

        //PATHFINDER TELEOP, Don't run before commenting other drive stuff
        // Don't know if the logic for the drive pose correlates however, all that needs to be done is to create a pose with a x, y, and heading value
        pathfinder.absoluteDrive(drive);
        pathfinder.loopAuto();


        //INTAKE AND DEPOSIT PLACEHOLDER
        if(gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            intake.toggle();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            deposit.latchToggle();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.X)){
            deposit.setLowPosition();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.Y)){
            deposit.setHighPosition();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.A)){
            deposit.setMidPosition();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.B)){
            deposit.setDownPosition();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN)){
            deposit.setSwivelServoCenter();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)){
            deposit.setSwivelServoLeft();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            deposit.setSwivelServoRight();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_UP)){
            droneShooter.shoot();
        }

        intake.loop();
        deposit.loop();
        droneShooter.loop();
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }
}