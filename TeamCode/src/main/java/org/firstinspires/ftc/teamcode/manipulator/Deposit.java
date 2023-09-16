package org.firstinspires.ftc.teamcode.manipulator;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;
import android.view.View;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class Deposit {
    private Servo slideServo1;
    private Servo slideServo2;
    private Servo angleServo;
    private Servo swivelServo;
    private NormalizedColorSensor colorSensor;
    private RevBlinkinLedDriver blinkin;
    private View relativeLayout;

    private double targetPosition1 = 0;
    private double targetPosition2 = 0;
    private double anglePosition = 0;
    private double swivelPosition = 0;

    private static double angleServoOpen = 0.3;
    private static double angleServoClosed = 0;


    // PLACEHOLDER VALUES
    private static double downPosition = 0;

    private static double lowPosition1 = 0.2;
    private static double lowPosition2 = 0.2;

    private static double midPosition1 = 0.6;
    private static double midPosition2 = 0.6;

    private static double highPosition1 = 0.8;
    private static double highPosition2 = 0.8;

    private static double swivelServoLeft = 0;
    private static double swivelServoRight = 0.3;
    private static double swivelServoCenter = 0.6;

    private boolean isServoToggled = false;

    private double slideSpeed;

    private SlideState currentSlideState = SlideState.DOWN;
    private SlideState targetSlideState = SlideState.DOWN;
    private LatchState currentLatchState = LatchState.CLOSED;
    private SwivelState currentSwivelState = SwivelState.CENTER;
    private SwivelState targetSwivelState = SwivelState.CENTER;
    private DepositState currentDepositState = DepositState.NO_PIXEL;


    private Timing.Timer lowScoreTimer = new Timing.Timer(3);
    private Timing.Timer midScoreTimer = new Timing.Timer(4);
    private Timing.Timer highScoreTimer = new Timing.Timer(5);


    private int colorSensorGain = 2;
    private float[] hsvValues = new float[3];




    public Deposit(RobotHardware robot){
        slideServo1 = robot.slideServo1;
        slideServo2 = robot.slideServo2;
        angleServo = robot.angleServo;
        swivelServo = robot.swivelServo;
        colorSensor = robot.colorSensor;
        relativeLayout = robot.relativeLayout;
        blinkin = robot.blinkin;

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        colorSensor.setGain(colorSensorGain);
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
    }

    public void loop(){



        slideServo1.setPosition((int)targetPosition1);
        slideServo2.setPosition((int)targetPosition2);
        angleServo.setPosition(anglePosition);
        swivelServo.setPosition(swivelPosition);
        telemetry.addData("Current Slide State", currentSlideState);
        telemetry.addData("Target Slide State", targetSlideState);
        telemetry.addData("Current Swivel State", currentSwivelState);
        telemetry.addData("Target Swivel State", targetSwivelState);
        telemetry.addData("Latch State", currentLatchState);
        telemetry.addData("Deposit State", currentDepositState);
        telemetry.addData("Slide Servo 1 Current Position",  slideServo1.getPosition());
        telemetry.addData("Slide Servo 2 Current Position",  slideServo2.getPosition());
        telemetry.addData("Slide Servo 1 Target Position", targetPosition1);
        telemetry.addData("Slide Servo 2 Target Position", targetPosition2);
        telemetry.addData("Angle Servo Current Position", angleServo.getPosition());
        telemetry.addData("Angle Servo Target Position", anglePosition);
        telemetry.addData("Swivel Servo Current Position", swivelServo.getPosition());
        telemetry.addData("Swivel Servo Target Position", swivelPosition);

        runColorSensor();

        if(Math.abs(targetPosition1-slideServo1.getPosition()) < 10 || Math.abs(targetPosition2-slideServo2.getPosition()) < 10){
            currentSlideState = SlideState.TRANSITION;
        } else {
            currentSlideState = targetSlideState;
        }

        if(Math.abs(swivelPosition-swivelServo.getPosition()) < 0.01){
            currentSwivelState = SwivelState.TRANSITION;
        } else {
            currentSwivelState = targetSwivelState;
        }

        if(currentDepositState == DepositState.HAS_PIXEL)   blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

    }

    public void latchToggle(){
        if(currentSlideState != SlideState.TRANSITION && currentSwivelState != SwivelState.TRANSITION) {
            isServoToggled = !isServoToggled;
            anglePosition = isServoToggled ? angleServoOpen : angleServoClosed;
            currentLatchState = isServoToggled ? LatchState.OPEN : LatchState.CLOSED;
        }
    }

    public void setWhiteLed(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void setYellowLed(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void setPurpleLed(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }

    public void setGreenLed(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }



    public void scoreLowPosition(){
        lowScoreTimer.start();
        setLowPosition();
        if (lowScoreTimer.elapsedTime() == 0.5) latchToggle();
        if (lowScoreTimer.elapsedTime() == 1.5) latchToggle();
        if (lowScoreTimer.elapsedTime() == 2.5) setDownPosition();

    }

    public void scoreMidPosition(){
        midScoreTimer.start();
        setMidPosition();
        if (lowScoreTimer.elapsedTime() == 1.5) latchToggle();
        if (lowScoreTimer.elapsedTime() == 2.5) latchToggle();
        if (lowScoreTimer.elapsedTime() == 3.5) setDownPosition();
    }

    public void scoreHighPosition(){
        highScoreTimer.start();
        setHighPosition();
        if (lowScoreTimer.elapsedTime() == 2.5) latchToggle();
        if (lowScoreTimer.elapsedTime() == 3.5) latchToggle();
        if (lowScoreTimer.elapsedTime() == 4.5) setDownPosition();
    }


    public void setDownPosition(){
         if(currentLatchState == LatchState.OPEN) latchToggle();

            targetPosition1 = downPosition;
            targetPosition2 = downPosition;
            targetSlideState = SlideState.DOWN;
    }

    public void setLowPosition(){
        if(currentDepositState == DepositState.HAS_PIXEL) {
            if(currentLatchState == LatchState.OPEN) latchToggle();
            targetPosition1 = lowPosition1;
            targetPosition2 = lowPosition2;
            targetSlideState = SlideState.LOW;
        }

    }

    public void setMidPosition(){
        if(currentDepositState == DepositState.HAS_PIXEL) {
            if(currentLatchState == LatchState.OPEN) latchToggle();
            targetPosition1 = midPosition1;
            targetPosition2 = midPosition2;
            targetSlideState = SlideState.MID;
        }
    }

    public void setHighPosition(){
        if(currentDepositState == DepositState.HAS_PIXEL) {
            if(currentLatchState == LatchState.OPEN) latchToggle();
            targetPosition1 = highPosition1;
            targetPosition2 = highPosition2;
            targetSlideState = SlideState.HIGH;
        }
    }

    public void setSwivelServoLeft(){
        swivelPosition = swivelServoLeft;
        targetSwivelState = SwivelState.LEFT;
    }

    public void setSwivelServoRight(){
        swivelPosition = swivelServoRight;
        targetSwivelState = SwivelState.RIGHT;
    }

    public void setSwivelServoCenter(){
        swivelPosition = swivelServoCenter;
        targetSwivelState = SwivelState.CENTER;
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

            if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 1){
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
        HIGH,
        TRANSITION
    }

    public enum SwivelState{
        LEFT,
        RIGHT,
        CENTER,
        TRANSITION
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

