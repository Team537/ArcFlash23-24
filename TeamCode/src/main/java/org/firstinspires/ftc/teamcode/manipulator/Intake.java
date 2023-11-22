package org.firstinspires.ftc.teamcode.manipulator;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;


public class Intake {

    private DcMotorEx intakeMotor;
    private double power = 0;
    private boolean isToggled = false;
    private IntakeState intakeState = IntakeState.STOPPED;
    private RevColorSensorV3 colorSensor;
    private View relativeLayout;
    private Intake.PixelState currentPixelState = Intake.PixelState.NONE;
    private Deposit deposit;

    private int colorSensorGain = 2;
    private float[] hsvValues = new float[3];
    private double pixelCount = 0;

    private boolean isDoublePixel = false;
    private double pixelSenseDelay = 0;
//    public boolean flashLED = false;



    public Intake(RobotHardware robot){
        intakeMotor = robot.intakeMotor;
        colorSensor = robot.colorSensor;
        relativeLayout = robot.relativeLayout;

    }

    public void loop(){
      intakeMotor.setPower(power);
//        telemetry.addData("Intake State", intakeState);
//        telemetry.addData("Intake Power", power);
        runColorSensor();

    }

    public IntakeState getIntakeState(){
        return intakeState;
    }
    public PixelState getCurrentPixelState(){ return currentPixelState; }
    public double getCurrentPixelCount(){ return pixelCount; }
//    public boolean getLEDFlashBool(){ return flashLED; }

    public void run(){
        power = -1;
        intakeState = IntakeState.RUNNING ;
    }
    public void runOut(){
        power = 0.25;
        intakeState = IntakeState.SPITTING ;
    }
    public void stop(){
        power = 0;
        intakeState = IntakeState.STOPPED ;
    }

    public void runColorSensor(){

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(),  hsvValues);

//        if(colors != null) {
//            telemetry.addLine()
//                    .addData("Red", "%.3f", colors.red)
//                    .addData("Green", "%.3f", colors.green)
//                    .addData("Blue", "%.3f", colors.blue);
//            telemetry.addLine()
//                    .addData("Hue", "%.3f", hsvValues[0])
//                    .addData("Saturation", "%.3f", hsvValues[1])
//                    .addData("Value", "%.3f", hsvValues[2]);
//            telemetry.addData("Alpha", "%.3f", colors.alpha);
//        }
        if (colorSensor instanceof DistanceSensor) {
//            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));

            if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 1){
                if(pixelSenseDelay <= 0) {
                    if (pixelCount == 0) {
                        currentPixelState = Intake.PixelState.DETECTED1;
                    } else if (pixelCount == 1) {
                        currentPixelState = Intake.PixelState.DETECTED2;
                        isDoublePixel = true;
                        runOut();
//                        flashLED = true;
                    }
                    pixelCount++;
                }
                pixelSenseDelay = pixelSenseDelay + 2;
            } else {

            }
        }

        if(pixelSenseDelay > 3) {
            pixelSenseDelay = 3;
        } else if (pixelSenseDelay > 0) {
            pixelSenseDelay--;
        }

//        currentPixelState = Intake.PixelState.NONE;
//        pixelCount = 0;

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });



    }
    public void resetIntakeCount() {
        currentPixelState = Intake.PixelState.NONE;
        pixelCount = 0;
        isDoublePixel = false;
    }

    public boolean isDoublePixel(){

        return isDoublePixel;
    }

    public enum IntakeState{
        RUNNING,
        SPITTING,
        STOPPED
    }

    public enum PixelState{
        DETECTED1,
        DETECTED2,
        DETECTED2MANY,
        NONE
    }


}
