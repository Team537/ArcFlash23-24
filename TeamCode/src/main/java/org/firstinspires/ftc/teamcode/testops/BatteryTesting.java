package org.firstinspires.ftc.teamcode.testops;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Log;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;

import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Date;


@Config
@TeleOp(name = "Battery Testing")
public class BatteryTesting extends CommandOpMode {


    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;
    private VoltageSensor voltageSensor;
    private GamepadEx gamepadEx;
    private Log log;
    private BNO055IMU imu;

    private static double MAX_X_SPEED = 5.0;
    private static double MAX_Y_SPEED = 5.0;
    private static double MAX_TURN_SPEED = 180;
    LynxModule control_hub;
    Double volts;
    Double current;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.AUTO = false;

        SimpleDateFormat formatter = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
        Date date = new Date();

        gamepadEx = new GamepadEx(gamepad1);

        motor1 =  hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        motor2 = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        motor3 = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        motor4 = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        control_hub = (LynxModule) hardwareMap.get(LynxModule.class, "Control Hub");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        String logName = "Battery Testing ";

        log = new Log(logName,true);


        


//        log.addData(control_hub.getGpioBusCurrent(CurrentUnit.AMPS));
//        log.addData(control_hub.getI2cBusCurrent(CurrentUnit.AMPS));
//        log.addData(imu.getPosition().x);
//        log.addData(imu.getPosition().y);
//        log.addData(imu.getPosition().z);
//        log.addData(motor1.getCurrent(CurrentUnit.AMPS));
//        log.addData(motor2.getCurrent(CurrentUnit.AMPS));
//        log.addData(motor3.getCurrent(CurrentUnit.AMPS));
//        log.addData(motor4.getCurrent(CurrentUnit.AMPS));





    }

    @Override
    public void run() {
        volts = voltageSensor.getVoltage();
        current = control_hub.getCurrent(CurrentUnit.AMPS);
        motor1.setPower(gamepadEx.getLeftY());
        motor2.setPower(gamepadEx.getLeftY());
        motor3.setPower(gamepadEx.getLeftY());
        motor4.setPower(gamepadEx.getLeftY());

        log.addData(volts);
        log.addData(current);
        telemetry.addData("Log", log.getLine());
        telemetry.update();



        log.update();

    }
}
