package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.manipulator.Deposit;

@Config
@TeleOp(name = "Test LED OpMode")
public class TimeLEDTestOpMode extends CommandOpMode {

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


        }



        if(gamepadEx.getButton(GamepadKeys.Button.B)){
            deposit.setPurpleLed();


        }

        if(gamepadEx.getButton(GamepadKeys.Button.X)){
            deposit.setGreenLed();


        }

        if(gamepadEx.getButton(GamepadKeys.Button.Y)){
            deposit.setYellowLed();


        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN)){
            deposit.setNoneLed();

        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_UP)){
           deposit.setWhiteGreenLed(getRuntime());
        }


        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)){
            deposit.setWhitePurpleLed(getRuntime());

        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)){
           deposit.setWhiteYellowLed(getRuntime());
        }











        telemetry.addData("Mode", deposit.getState().toString());
        telemetry.addData("Touch Sensor", deposit.getTouchBool());
        telemetry.addData("LED State", deposit.getCurrentLEDState().toString());
        telemetry.update();

//        robot.clearBulkCache();

    }

}