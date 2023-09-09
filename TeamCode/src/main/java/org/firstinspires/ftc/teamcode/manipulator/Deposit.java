package org.firstinspires.ftc.teamcode.manipulator;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class Deposit {
    private DcMotorEx slideMotor1;
    private DcMotorEx slideMotor2;
    private Servo latchServo;
    private NormalizedColorSensor colorSensor;
    private View relativeLayout;

    private double targetPosition1 = 0;
    private double targetPosition2 = 0;
    private double servoPosition = 0;

    private static double latchServoOpen = 90;
    private static double latchServoClosed = 0;


    // PLACEHOLDER VALUES
    private static double downPosition = 0;

    private static double lowPosition1 = 50;
    private static double lowPosition2 = 50;

    private static double midPosition1 = 75;
    private static double midPosition2 = 75;

    private static double highPosition1 = 100;
    private static double highPosition2 = 100;

    private boolean isServoToggled = false;

    private double slideSpeed;

    private SlideState currentSlideState = SlideState.DOWN;
    private LatchState currentLatchState = LatchState.CLOSED;
    private DepositState currentDepositState = DepositState.NO_PIXEL;

    private PIDController pidController = new PIDController(0.01, 0, 0);
    private double slideMotor1Position;
    private static double ticksPerDegree = 700/180;

    private int colorSensorGain = 2;
    private float[] hsvValues = new float[3];




    public Deposit(RobotHardware robot){
        slideMotor1 = robot.slideMotor1;
        slideMotor2 = robot.slideMotor2;
        latchServo = robot.latchServo;
        colorSensor = robot.colorSensor;
        relativeLayout = robot.relativeLayout;

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        colorSensor.setGain(colorSensorGain);
    }

    public void loop(){
        slideMotor1Position = slideMotor1.getCurrentPosition();
        slideSpeed = pidController.calculate(targetPosition1-slideMotor1Position);


        slideMotor1.setTargetPosition((int)targetPosition1);
        slideMotor2.setTargetPosition((int)targetPosition2);
        slideMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor1.setPower(slideSpeed);
        slideMotor2.setPower(slideSpeed);
        latchServo.setPosition(servoPosition);

        telemetry.addData("Slide State", currentSlideState);
        telemetry.addData("Latch State", currentLatchState);
        telemetry.addData("Deposit State", currentDepositState);
        telemetry.addData("Slide Motor 1 Current Position", slideMotor1.getCurrentPosition());
        telemetry.addData("Slide Motor 2 Current Position", slideMotor2.getCurrentPosition());
        telemetry.addData("Slide Motor 1 Target Position", targetPosition1);
        telemetry.addData("Slide Motor 2 Target Position", targetPosition2);
        telemetry.addData("Latch Servo Current Position", latchServo.getPosition());
        telemetry.addData("Latch Servo Target Position", servoPosition);

        runColorSensor();

    }

    public void latchToggle(){
        isServoToggled = !isServoToggled;
        servoPosition = isServoToggled ? latchServoOpen : latchServoClosed;
        currentLatchState = isServoToggled ? LatchState.OPEN : LatchState.CLOSED;

    }


    public void setDownPosition(){
            targetPosition1 = downPosition;
            targetPosition2 = downPosition;
            currentSlideState = SlideState.DOWN;
    }

    public void setLowPosition(){
        if(currentDepositState == DepositState.HAS_PIXEL) {
            targetPosition1 = lowPosition1;
            targetPosition2 = lowPosition2;
            currentSlideState = SlideState.LOW;
        }

    }

    public void setMidPosition(){
        if(currentDepositState == DepositState.HAS_PIXEL) {
            targetPosition1 = midPosition1;
            targetPosition2 = midPosition2;
            currentSlideState = SlideState.MID;
        }
    }

    public void setHighPosition(){
        if(currentDepositState == DepositState.HAS_PIXEL) {
            targetPosition1 = highPosition1;
            targetPosition2 = highPosition2;
            currentSlideState = SlideState.HIGH;
        }
    }

    public void runColorSensor(){

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(),  hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));

            if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > 1){
                currentDepositState = DepositState.HAS_PIXEL;
            } else {
                currentDepositState = DepositState.NO_PIXEL;
            }
        }


        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });

    }

    public enum SlideState{
        DOWN,
        LOW,
        MID,
        HIGH
    }

    public enum LatchState{
        OPEN,
        CLOSED
    }

    public enum DepositState{
        HAS_PIXEL,
        NO_PIXEL
    }



}

