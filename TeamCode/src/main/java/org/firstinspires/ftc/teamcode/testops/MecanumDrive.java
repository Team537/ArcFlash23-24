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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.manipulator.Deposit;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;

@Config
@TeleOp(name = "Mecanum Drive")
public class MecanumDrive extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private Deposit deposit;
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;



    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.AUTO = false;
        deposit = new Deposit(robot);
        robot.init(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);


        robot.enabled = true;

        frontLeft = robot.frontLeftMotor;
        frontRight = robot.frontRightMotor;
        backLeft = robot.backLeftMotor;
        backRight = robot.backRightMotor;

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

        drivetrain.read();

    }

    @Override
    public void run() {

        robot.startIMUThread(this);

        double speed = Math.hypot(gamepadEx.getLeftX(),gamepadEx.getLeftY()); //get speed
        double LeftStickAngle = Math.atan2(gamepadEx.getLeftX(), -gamepadEx.getLeftY()) - Math.PI / 4; //get angle
        double rightX = -gamepad1.right_stick_x; //rotation

        double leftFrontPower = speed * Math.cos(LeftStickAngle - robot.getAngle()) + rightX;
        double rightFrontPower = speed * Math.sin(LeftStickAngle - robot.getAngle()) - rightX;
        double leftBackPower = speed * Math.sin(LeftStickAngle - robot.getAngle()) + rightX;
        double rightBackPower = speed * Math.cos(LeftStickAngle - robot.getAngle()) - rightX;

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

        if(gamepadEx2.getButton((GamepadKeys.Button.LEFT_BUMPER))) {
            deposit.latchOpen();
        }

        if(gamepadEx2.getButton((GamepadKeys.Button.RIGHT_BUMPER))) {
            deposit.latchClose();
        }


        if(gamepadEx2.getButton((GamepadKeys.Button.B))) {
            deposit.setDownPosition();
        }

        if(gamepadEx2.getButton((GamepadKeys.Button.A))) {
            deposit.setLowPosition();
        }

        if(gamepadEx2.getButton((GamepadKeys.Button.X))) {
            deposit.setMidPosition();
        }

        if(gamepadEx2.getButton((GamepadKeys.Button.Y))) {
            deposit.setHighPosition();
        }

        if(gamepadEx.getButton(GamepadKeys.Button.A)){
            deposit.setWhiteLed();
            deposit.setLEDState(Deposit.LEDState.WHITE);

        }



        if(gamepadEx.getButton(GamepadKeys.Button.B)){
            deposit.setPurpleLed();
            deposit.setLEDState(Deposit.LEDState.PURPLE);


        }

        if(gamepadEx.getButton(GamepadKeys.Button.X)){
            deposit.setGreenLed();
            deposit.setLEDState(Deposit.LEDState.GREEN);


        }

        if(gamepadEx.getButton(GamepadKeys.Button.Y)){
            deposit.setYellowLed();
            deposit.setLEDState(Deposit.LEDState.YELLOW);


        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN)){
            deposit.setNoneLed();
            deposit.setLEDState(Deposit.LEDState.NONE);

        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_UP)){
            deposit.setLEDState(Deposit.LEDState.WHITE_GREEN);
        }


        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)){
            deposit.setLEDState(Deposit.LEDState.WHITE_PURPLE);

        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            deposit.setLEDState(Deposit.LEDState.WHITE_YELLOW);
        }
        telemetry.addData("Robot Angle: ",robot.getAngle()); //this is all telemetry stuff
        telemetry.addData("Front Left Power: ", leftFrontPower);
        telemetry.addData("Front Right Power: ", rightFrontPower);
        telemetry.addData("Rear Left Power: ", leftBackPower);
        telemetry.addData("Rear Right Power: ", rightBackPower);
        deposit.periodic();
        telemetry.update();

//        robot.clearBulkCache();

    }
}