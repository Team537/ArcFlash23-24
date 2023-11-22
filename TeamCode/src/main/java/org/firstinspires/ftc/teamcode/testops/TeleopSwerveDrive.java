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

    private boolean claw1OpenFirst = true;
    private boolean claw2OpenFirst = true;

    private boolean pixel1Detected = false;
    private boolean pixel2Detected = false;



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




        pixel1Detected = arm.getPixel1Detcted();
        pixel2Detected = arm.getPixel2Detcted();



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
            toggleClaw1();


        }

        if(gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            claw2Boolean = !claw2Boolean;
            toggleClaw2();

        }

        if(gamepadEx2.getButton(GamepadKeys.Button.Y)){
            clawBoolean = !clawBoolean;

            claw1Boolean = clawBoolean;
            claw2Boolean = clawBoolean;

            toggleClaw1();
            toggleClaw2();

        }

        if(pixel1Detected){
            if(claw1OpenFirst) {
                arm.setClaw1Closed();
                claw1Boolean = false;
                claw1OpenFirst = false;
            } else if (claw1Boolean){
                claw1OpenFirst = true;
            }
        }

        if(pixel2Detected){
            if(claw2OpenFirst) {
                arm.setClaw2Closed();
                claw2Boolean = false;
                claw2OpenFirst = false;
            } else if (claw2Boolean){
                claw2OpenFirst = true;
            }
        }





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
        arm.loop();
        telemetry.update();






    }

    public void toggleClaw1(){
        if(claw1Boolean){
            arm.setClaw1Open();
        } else {
            arm.setClaw1Closed();
        }
    }

    public void toggleClaw2(){
        if(claw2Boolean){
            arm.setClaw2Open();
        } else {
            arm.setClaw2Closed();
        }
    }


}

