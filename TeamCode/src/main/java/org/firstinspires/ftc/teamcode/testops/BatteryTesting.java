package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Log;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;
import org.openftc.revextensions2.ExpansionHubEx;


@Config
@TeleOp(name = "Battery Testing")
public class BatteryTesting extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;
    private ExpansionHubEx hub;
    private GamepadEx gamepadEx;
    private Log log;
    private BNO055IMU imu;
    private VoltageSensor voltageSensor;
    private static double MAX_X_SPEED = 5.0;
    private static double MAX_Y_SPEED = 5.0;
    private static double MAX_TURN_SPEED = 180;
    LynxModule control_hub;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.AUTO = false;

        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        robot.init(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);

        motor1 = robot.frontLeftMotor;
        motor2 = robot.frontRightMotor;
        motor3 = robot.backLeftMotor;
        motor4 = robot.backRightMotor;
        imu = robot.imu;
        control_hub = (LynxModule) hardwareMap.get(LynxModule.class, "Control Hub");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        log = new Log("Battery Testing",true);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

    }

    @Override
    public void run() {

        motor1.setPower(gamepadEx.getLeftY());
        motor2.setPower(gamepadEx.getLeftY());
        motor3.setPower(gamepadEx.getLeftY());
        motor4.setPower(gamepadEx.getLeftY());

    log.addData(voltageSensor.getVoltage());
    log.addData(control_hub.getCurrent(CurrentUnit.AMPS));
    log.addData(control_hub.getGpioBusCurrent(CurrentUnit.AMPS));
    log.addData(control_hub.getI2cBusCurrent(CurrentUnit.AMPS));
    log.addData(imu.getPosition().x);
    log.addData(imu.getPosition().y);
    log.addData(imu.getPosition().z);
    log.addData(motor1.getCurrent(CurrentUnit.AMPS));
    log.addData(motor2.getCurrent(CurrentUnit.AMPS));
    log.addData(motor3.getCurrent(CurrentUnit.AMPS));
    log.addData(motor4.getCurrent(CurrentUnit.AMPS));

        telemetry.update();

        robot.clearBulkCache();

        log.update();

    }
}
