package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Log;

import java.text.SimpleDateFormat;
import java.util.Date;


@Config
@TeleOp(name = "Battery Testing")
public class BatteryTesting extends CommandOpMode {


    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;
    private CRServo servo1;
    private VoltageSensor voltageSensor;
    private GamepadEx gamepadEx;
    private Log log;
    private BNO055IMU imu;

    private static double MAX_X_SPEED = 5.0;
    private static double MAX_Y_SPEED = 5.0;
    private static double MAX_TURN_SPEED = 180;
    LynxModule control_hub;
    Double total_volts;
    Double total_current;

    LynxGetADCCommand.Channel servoChannel;
    LynxGetADCCommand servoCommand;
    LynxGetADCResponse servoResponse;
    double servoBusCurrent;

    double motor1BusCurrent;
    double motor2BusCurrent;
    double motor3BusCurrent;
    double motor4BusCurrent;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.AUTO = false;

        

        gamepadEx = new GamepadEx(gamepad1);

        motor1 =  hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        motor2 = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        motor3 = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        motor4 = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        servo1 = hardwareMap.get(CRServo.class, "frontLeftServo");
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
        total_volts = voltageSensor.getVoltage();
        total_current = control_hub.getCurrent(CurrentUnit.AMPS);
        motor1.setPower(gamepadEx.getLeftY());
        motor2.setPower(gamepadEx.getLeftY());
        motor3.setPower(gamepadEx.getLeftY());
        motor4.setPower(gamepadEx.getLeftY());
        servo1.setPower(gamepadEx.getRightY());
        servoBusCurrent = getServoBusCurrent();

        motor1BusCurrent = motor1.getCurrent(CurrentUnit.AMPS);
        motor2BusCurrent = motor2.getCurrent(CurrentUnit.AMPS);
        motor3BusCurrent = motor3.getCurrent(CurrentUnit.AMPS);
        motor4BusCurrent = motor4.getCurrent(CurrentUnit.AMPS);

        log.addData(motor1BusCurrent);
        log.addData(motor2BusCurrent);
        log.addData(motor3BusCurrent);
        log.addData(motor4BusCurrent);

        log.addData(total_volts);
        log.addData(total_current);
        telemetry.addData("Log", log.getLine());
        telemetry.update();



        log.update();

    }

    double getServoBusCurrent()
    {
        servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
        servoCommand = new LynxGetADCCommand(control_hub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            servoResponse = servoCommand.sendReceive();
            return servoResponse.getValue() / 1000.0;    // return value in Amps
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
        }
        return 999;
    }


}
