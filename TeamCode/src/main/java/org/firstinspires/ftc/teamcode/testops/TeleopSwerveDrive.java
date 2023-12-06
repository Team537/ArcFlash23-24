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


    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    private static double MAX_X_SPEED = 5.0;
    private static double MAX_Y_SPEED = 5.0;
    private static double MAX_TURN_SPEED = Math.PI/4;
    private boolean intakeToggle = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.AUTO = false;


        robot.init(hardwareMap, telemetry);
//        arm = new ArmSystem(robot);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
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
                gamepadEx1.getLeftY() * MAX_X_SPEED,
                -gamepadEx1.getLeftX() * MAX_Y_SPEED, // if not - it will go left when joystick is right...
                gamepadEx1.getRightX() * MAX_TURN_SPEED
        ),new Rotation2d(robot.getAngle()));

        robot.read(drivetrain);



//        pixel1Detected = arm.getPixel1Detcted();
//        pixel2Detected = arm.getPixel2Detcted();
//
//
//
//        if(gamepadEx2.getButton(GamepadKeys.Button.DPAD_UP)){
//            arm.setHighPosition();
//        }
//
//        if(gamepadEx2.getButton(GamepadKeys.Button.DPAD_RIGHT)){
//            arm.setMidPosition();
//        }
//
//        if(gamepadEx2.getButton(GamepadKeys.Button.DPAD_LEFT)){
//            arm.setLowPosition();
//        }
//
//        if(gamepadEx2.getButton(GamepadKeys.Button.DPAD_DOWN)){
//            arm.setDownPosition();
//        }
//
//        if(gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER)){
//            claw1Boolean = !claw1Boolean;
//            toggleClaw1();
//
//
//        }
//
//        if(gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
//            claw2Boolean = !claw2Boolean;
//            toggleClaw2();
//
//        }
//
//        if(gamepadEx2.getButton(GamepadKeys.Button.Y)){
//            clawBoolean = !clawBoolean;
//
//            claw1Boolean = clawBoolean;
//            claw2Boolean = clawBoolean;
//
//            toggleClaw1();
//            toggleClaw2();
//
//        }
//
//        if(pixel1Detected){
//            if(claw1OpenFirst) {
//                arm.setClaw1Closed();
//                claw1Boolean = false;
//                claw1OpenFirst = false;
//            } else if (claw1Boolean){
//                claw1OpenFirst = true;
//            }
//        }
//
//        if(pixel2Detected){
//            if(claw2OpenFirst) {
//                arm.setClaw2Closed();
//                claw2Boolean = false;
//                claw2OpenFirst = false;
//            } else if (claw2Boolean){
//                claw2OpenFirst = true;
//            }
//        }
//




        drivetrain.updateModules();
        telemetry.addData("Angle", robot.getAngle());
        telemetry.addData("Swerve", drivetrain.getTelemetry());
        telemetry.addData("Swerve Module States", drivetrain.getSwerveModuleStates());
        telemetry.addData("Swerve Module Servo", drivetrain.getSwerveServoPowers());
        telemetry.addData("Module Velocities", drivetrain.getVelocities());
//        telemetry.addData("Pivot Position", arm.getPivotPosition());
//        telemetry.addData("Pivot Speed", arm.getPivotSpeed());
//        telemetry.addData("Extend Position", arm.getExtendPosition());
//        telemetry.addData("Extend Speed", arm.getExtendSpeed());
//        telemetry.addData("Wrist Position", arm.getWristPosition());
//        telemetry.addData("Claw 1 Position", arm.getClaw1Position());
//        telemetry.addData("Claw 2 Position", arm.getClaw2Position());
        //arm.loop();
        telemetry.update();

//        if(gamepadEx.getButton(GamepadKeys.Button.A)) {
//            intakeToggle = !intakeToggle;
//
//        }
//
//        if(intakeToggle){
//            intake.run();
//        }else{
//            intake.stop();
//        }
//
//        if(gamepadEx.getButton(GamepadKeys.Button.B)){
//            deposit.setHighPosition();
//        }
//
//        if(gamepadEx.getButton(GamepadKeys.Button.X)){
//            deposit.setDownPosition();
//        }
//
//        intake.loop();
//        deposit.periodic();
//        robot.clearBulkCache();

        //this shouldnt be here. its from another teleop mode and will break everything
//        telemetry.addData("Intake State", intake.getIntakeState());
//        telemetry.addData("Slide State", deposit.getCurrentSlideState());
//        telemetry.addData("Slide 1 Position", deposit.getSlideMotor1Position());
//        telemetry.addData("Slide 2 Position", deposit.getSlideMotor2Position());


    }
}
