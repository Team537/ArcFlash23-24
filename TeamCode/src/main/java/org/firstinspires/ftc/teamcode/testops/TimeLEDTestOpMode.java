package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.manipulator.Deposit;

@Config
@TeleOp(name = "Test Time LED OpMode")
public class TimeLEDTestOpMode extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private Deposit deposit;
    private boolean isWhiteGreen = false;
    private boolean isWhiteGreenDone = false;
    private boolean isWhitePurple;
    private boolean isWhiteYellow;
    int i = 0;

    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
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
        gamepadEx2 = new GamepadEx(gamepad2);


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

        if(gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.9) {
            deposit.setAngleServoScore();
        }

        if(gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            deposit.latchToggle();
        }

        if(deposit.getCurrentLEDState() == Deposit.LEDState.WHITE_GREEN){
            deposit.setWhiteGreenLed(getRuntime());
        }

        if(deposit.getCurrentLEDState() == Deposit.LEDState.WHITE_PURPLE){
            deposit.setWhitePurpleLed(getRuntime());
        }

        if(deposit.getCurrentLEDState() == Deposit.LEDState.WHITE_YELLOW){
            deposit.setWhiteYellowLed(getRuntime());
        }







        telemetry.addData("Angle Servo", deposit.getCurrentAngleState().toString());
        telemetry.addData("Latch Servo", deposit.getCurrentLatchState().toString());
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Mode", deposit.getState().toString());
        telemetry.addData("Slide State", deposit.getCurrentSlideState().toString());
        telemetry.addData("Touch Sensor", deposit.getTouchBool());
        telemetry.addData("LED State", deposit.getCurrentLEDState().toString());
        telemetry.update();

//        robot.clearBulkCache();

    }

}