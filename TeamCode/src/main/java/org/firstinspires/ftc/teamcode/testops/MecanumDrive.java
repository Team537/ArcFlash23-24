package org.firstinspires.ftc.teamcode.testops;

import static com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer.Lynx;

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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.manipulator.Deposit;
import org.firstinspires.ftc.teamcode.manipulator.DroneShooter;
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
    private DcMotorEx slideMotor1;
    private DcMotorEx slideMotor2;
    private Deposit.SlideState currentSlideState;
    private Deposit.SlideState targetSlideState;
    private Deposit.AngleState currentAngleState;
    private Deposit deposit;
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
    public BNO055IMU imu;
    private LynxModule controlHub;
    public DroneShooter shooter;




    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Globals.AUTO = false;
//        deposit = new Deposit(robot);
        robot.init(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        shooter = new DroneShooter(robot);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        controlHub = (LynxModule) hardwareMap.get(LynxModule.class, "Control Hub");
        robot.enabled = true;

        frontLeft = robot.frontLeftMotor;
        frontRight = robot.frontRightMotor;
        backLeft = robot.backLeftMotor;
        backRight = robot.backRightMotor;

//        slideMotor1 = robot.slideMotor1;
//        slideMotor2 = robot.slideMotor2;


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

//        drivetrain.read();

    }

    @Override
    public void run() {


//        currentSlideState = deposit.getCurrentSlideState();
//        targetSlideState = deposit.getTargetSlideState();
//        currentAngleState = deposit.getCurrentAngleState();

//       DO NOT USE robot.startIMUThread(this);

        double speed = Math.hypot(gamepadEx.getLeftX(),gamepadEx.getLeftY()); //get speed
        double LeftStickAngle = Math.atan2(gamepadEx.getLeftX(), -gamepadEx.getLeftY()) - Math.PI / 4; //get angle
        double rightX = -gamepad1.right_stick_x; //rotation

        double leftFrontPower = speed * Math.sin(LeftStickAngle - getAngle()) + rightX;
        double rightFrontPower = speed * Math.sin(LeftStickAngle - getAngle()) - rightX;
        double leftBackPower = speed * Math.cos(LeftStickAngle - getAngle()) + rightX;
        double rightBackPower = speed * Math.cos(LeftStickAngle - getAngle()) - rightX;

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

    if(gamepadEx.getButton(GamepadKeys.Button.A)){
        shooter.droneStrike();
    }

        telemetry.addData("Robot Angle: ", getAngle());
        telemetry.addData("Current", controlHub.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front Left Power: ", leftFrontPower);
        telemetry.addData("Front Right Power: ", rightFrontPower);
        telemetry.addData("Rear Left Power: ", leftBackPower);
        telemetry.addData("Rear Right Power: ", rightBackPower);

//        deposit.periodic();
        telemetry.update();

//        robot.clearBulkCache();

    }

    // Gets imu angle
    public double getAngle(){
        return imu.getAngularOrientation().firstAngle - Math.PI/2;
    }
}
