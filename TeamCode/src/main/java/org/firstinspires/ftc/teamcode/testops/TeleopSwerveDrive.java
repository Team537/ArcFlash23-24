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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.manipulator.ArmSystem;


@Config
@TeleOp(name = "Test Swerve Drive")
public class TeleopSwerveDrive extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private ArmSystem arm;



    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    private static double MAX_X_SPEED = 5.0;
    private static double MAX_Y_SPEED = 5.0;
    private static double MAX_TURN_SPEED = Math.PI/4;

    private boolean claw1Boolean = false;
    private boolean claw2Boolean = false;
    private boolean clawBoolean = false;




    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.AUTO = false;


        robot.init(hardwareMap, telemetry);
        arm = new ArmSystem(robot);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        drivetrain = new SwerveDrivetrain(robot);



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
                gamepadEx1.getLeftX() * MAX_Y_SPEED,
                gamepadEx1.getRightX() * MAX_TURN_SPEED
        ),new Rotation2d(robot.getAngle()));


        drivetrain.updateModules();
        telemetry.addData("Angle", robot.getAngle());
        telemetry.addData("Swerve", drivetrain.getTelemetry());
        telemetry.addData("Swerve Module States", drivetrain.getSwerveModuleStates());
        telemetry.addData("Swerve Module Servo", drivetrain.getSwerveServoPowers());
        telemetry.addData("Module Velocities", drivetrain.getVelocities());
        telemetry.addData("Pivot Position", arm.getPivotPosition());
        telemetry.addData("Pivot Speed", arm.getPivotSpeed());
        telemetry.addData("Extend Position", arm.getExtendPosition());
        telemetry.addData("Extend Speed", arm.getExtendSpeed());
        telemetry.addData("Wrist Position", arm.getWristPosition());
        telemetry.addData("Claw 1 Position", arm.getClaw1Position());
        telemetry.addData("Claw 2 Position", arm.getClaw2Position());
        telemetry.update();

        arm.loop();

        if(gamepadEx2.getButton(GamepadKeys.Button.DPAD_UP)){
            arm.setHighPosition();
        }

        if(gamepadEx2.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            arm.setMidPosition();
        }

        if(gamepadEx2.getButton(GamepadKeys.Button.DPAD_LEFT)){
            arm.setLowPosition();
        }
        
        if(gamepadEx2.getButton(GamepadKeys.Button.DPAD_DOWN)){
            arm.setDownPosition();
        }

        if(gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            claw1Boolean = !claw1Boolean;

            if(claw1Boolean){
                arm.setClaw1Open();
            } else {
                arm.setClaw1Closed();
            }
        }

        if(gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            claw2Boolean = !claw2Boolean;

            if(claw2Boolean){
                arm.setClaw2Open();
            } else {
                arm.setClaw2Closed();
            }
        }

        if(gamepadEx2.getButton(GamepadKeys.Button.Y)){
            clawBoolean = !clawBoolean;

            claw1Boolean = clawBoolean;
            claw2Boolean = clawBoolean;

            if(clawBoolean){
                arm.setClawOpen();

            } else {
                arm.setClawClosed();
            }
        }








    }



}

