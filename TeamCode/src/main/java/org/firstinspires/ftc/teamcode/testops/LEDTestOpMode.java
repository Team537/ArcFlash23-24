package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.manipulator.Deposit;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;

@Config
@TeleOp(name = "Test LED OpMode")
public class LEDTestOpMode extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private Deposit deposit;
    private boolean isWhiteGreen = false;
    private boolean isWhiteGreenDone = false;
    private boolean isWhitePurple;
    private boolean isWhiteYellow;
    int i = 0;

    private GamepadEx gamepadEx;
    private static double MAX_X_SPEED = 5.0;
    private static double MAX_Y_SPEED = 5.0;
    private static double MAX_TURN_SPEED = 180;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.AUTO = false;

        robot.init(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        deposit = new Deposit(robot);
        gamepadEx = new GamepadEx(gamepad1);


        robot.enabled = true;

//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

    }

    @Override
    public void run() {
        deposit.loop();
        if(gamepadEx.getButton(GamepadKeys.Button.A)){
            deposit.setWhiteLed();
            isWhiteGreen = false;
            isWhiteYellow = false;
            isWhitePurple = false;

        }

        if(gamepadEx.getButton(GamepadKeys.Button.B)){
            deposit.setPurpleLed();
            isWhiteGreen = false;
            isWhiteYellow = false;
            isWhitePurple = false;

        }

        if(gamepadEx.getButton(GamepadKeys.Button.X)){
            deposit.setGreenLed();
            isWhiteGreen = false;
            isWhiteYellow = false;
            isWhitePurple = false;

        }

        if(gamepadEx.getButton(GamepadKeys.Button.Y)){
            deposit.setYellowLed();
            isWhiteGreen = false;
            isWhiteYellow = false;
            isWhitePurple = false;

        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN)){
            deposit.setNoneLed();
            isWhiteGreen = false;
            isWhiteYellow = false;
            isWhitePurple = false;
        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_UP)){
            isWhiteGreen = true;
            isWhiteYellow = false;
            isWhitePurple = false;
        }


        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)){
            isWhiteGreen = false;
            isWhiteYellow = true;
            isWhitePurple = false;

        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            isWhiteGreen = false;
            isWhiteYellow = false;
            isWhitePurple = true;
        }

        if(isWhiteGreen){
//            sleep(5);
            deposit.setWhiteLed();
//            sleep(5);
            deposit.setGreenLed();

        }

        if(isWhiteYellow){
//            sleep(5);
            deposit.setWhiteLed();
//            sleep(5);
            deposit.setYellowLed();

        }

        if(isWhitePurple){
//            sleep(5);
            deposit.setWhiteLed();
//            sleep(5);
            deposit.setPurpleLed();
        }









        telemetry.addData("Mode", deposit.getState().toString());
        telemetry.addData("Touch Sensor", deposit.getTouchBool());
        telemetry.addData("LED State", deposit.getCurrentLEDState().toString());
        telemetry.update();

//        robot.clearBulkCache();

    }

}