package org.firstinspires.ftc.teamcode.testops;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.manipulator.ArmSystem;

@TeleOp(name = "Teleop")
public class MecanumDrive extends LinearOpMode {
    public boolean claw1Boolean = false;
    public boolean claw2Boolean = false;
    public boolean clawBoolean = false;
    ArmSystem arm;
    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = RobotHardware.getInstance();
        arm = new ArmSystem(robot);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);


        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        double slow = 1;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.x) {
                imu.resetYaw();
            }

            if(gamepad1.right_trigger >= 0.75) {
                slow = 0.5;
            } else {
                slow = 1;
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower * slow);
            backLeftMotor.setPower(backLeftPower * slow);
            frontRightMotor.setPower(frontRightPower * slow);
            backRightMotor.setPower(backRightPower * slow);

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



    }
    public void toggleClaw1() {
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