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
    private IntakeToggle intakeToggle = IntakeToggle.STOP;
    private boolean intakeRunOut = false;
    private double intakeToggleNumber = 0;


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
        telemetry.addData("Swerve Module Servo", drivetrain.getSwerveServoPowers());
        telemetry.update();

        if(gamepadEx.getButton(GamepadKeys.Button.A)) {
            if(intakeToggleNumber <= 0) {
                if(intakeToggle == IntakeToggle.STOP) {
                    intakeToggle = IntakeToggle.RUN;
                } else if(intakeToggle == IntakeToggle.RUN) {
                    intakeToggle = IntakeToggle.STOP;
                } else if(intakeToggle == IntakeToggle.RUNOUT) {
                    intakeToggle = IntakeToggle.STOP;

                }
            }
            intakeToggleNumber++;
            intakeToggleNumber++;

        }
        if(gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            intake.resetIntakeCount();
        }

        if(intakeToggleNumber > 4) {
            intakeToggleNumber = 4;
        } else if(intakeToggleNumber > 0) {
            intakeToggleNumber--;
        }

//        if (intake.getLEDFlashBool()){
//            deposit.setWhiteLed();
//            deposit.setLEDState(Deposit.LEDState.NONE);
//        }


        if(intakeToggle == IntakeToggle.RUN){
            intake.run();
        }else if(intakeToggle == IntakeToggle.RUNOUT){
            intake.runOut();
        } else if (intakeToggle == IntakeToggle.STOP){
            intake.stop();
        }

        if(intake.isDoublePixel()){
            intakeToggle = IntakeToggle.RUNOUT;
        }

        if(intakeToggle == IntakeToggle.RUNOUT && !intake.isDoublePixel()) {
            intakeToggle = IntakeToggle.STOP;
        }

        intake.runColorSensor();

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_UP)){
            deposit.setHighPosition();
        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            deposit.setMidPosition();
        }

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)){
            deposit.setLowPosition();
        }
        
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN)){
            deposit.setDownPosition();
        }




        intake.loop();
        deposit.periodic();
//        robot.clearBulkCache();
        telemetry.addData("Intake State", intake.getIntakeState());
        telemetry.addData("Slide State", deposit.getCurrentSlideState());
        telemetry.addData("Slide 1 Position", deposit.getSlideMotor1Position());
        telemetry.addData("Slide 2 Position", deposit.getSlideMotor2Position());
        telemetry.addData("Current Pixel State", intake.getCurrentPixelState());
        telemetry.addData("Current Pixel Count", intake.getCurrentPixelCount());


    }


    public enum IntakeToggle {
        RUN,
        RUNOUT,
        STOP
    }
}

