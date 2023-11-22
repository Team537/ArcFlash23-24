package org.firstinspires.ftc.teamcode.manipulator;

import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class ArmSystem {
    private DcMotorEx armPivot;
    private DcMotorEx armExtend;
    private Servo clawServo1;
    private Servo clawServo2;
    private Servo wristServo;
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    private View relativeLayout;

    private PIDController armPivotController = new PIDController(0.01, 0, 0.01);
    private double pivotF = 0;

    private PIDController armExtendController = new PIDController(0.01, 0, 0.01);
    private double extendF = 0;

    double pivotSpeed = 0;
    double extendSpeed = 0;

    private double pivotTargetPosition = 0;
    private double extendTargetPosition = 0;
    private double claw1TargetPosition = 0;
    private double claw2TargetPosition = 0;
    private double wristTargetPosition = 0;

    private double pivotDownPosition = 0;
    private double pivotLowPosition = 10;
    private double pivotMidPosition = 20;
    private double pivotHighPosition = 30;

    private double extendDownPosition = 0;
    private double extendLowPosition = 10;
    private double extendMidPosition = 20;
    private double extendHighPosition = 30;

    private double clawOpenPosition1 = 0;
    private double clawClosedPosition1 = 0.75;

    private double clawOpenPosition2 = 0;
    private double clawClosedPosition2 = 0.75;

    private double wristDownPosition = 0;
    private double wristLowPosition = 0.1;
    private double wristMidPosition = 0.2;
    private double wristHighPosition = 0.3;

    private double ticks_per_degree = 700/180;

    public ArmSystem(RobotHardware robot){
        armPivot = robot.slideMotor1;
        armExtend = robot.intakeMotor;
        clawServo1 = robot.angleServo;
        clawServo2 = robot.angleServo2;
        wristServo = robot.latchServo;
        colorSensor1 = robot.colorSensor;
//        colorSensor2 = robot.colorSensor2;
        relativeLayout = robot.relativeLayout;

        if (colorSensor1 instanceof SwitchableLight) {
           ((SwitchableLight)colorSensor1).enableLight(true);
        }
    }

    public void loop(){

        double armPosition = armPivot.getCurrentPosition();
        double pivotPid = armPivotController.calculate(pivotTargetPosition-armPosition);
        double pivotFf = Math.cos(Math.toRadians(pivotTargetPosition / ticks_per_degree)) * pivotF;
        pivotSpeed = pivotPid + pivotFf;

        if(Math.abs(pivotTargetPosition-armPosition) > 5) {
            armPivot.setPower(pivotSpeed);
        } else {
            armPivot.setPower(0);
        }

        double extendPosition = armExtend.getCurrentPosition();
        double extendPid = armExtendController.calculate(extendTargetPosition-extendPosition);
        double extendFf = Math.cos(Math.toRadians(extendTargetPosition / ticks_per_degree)) * extendF;
        extendSpeed = extendPid + extendFf;

        if(Math.abs(extendTargetPosition-extendPosition) > 5) {
            armExtend.setPower(extendSpeed);
        } else {
            armExtend.setPower(0);
        }

        clawServo1.setPosition(claw1TargetPosition);
        clawServo2.setPosition(claw2TargetPosition);
        wristServo.setPosition(wristTargetPosition);


    }

    public void setDownPosition(){
        pivotTargetPosition = pivotDownPosition;
        extendTargetPosition = extendDownPosition;
        wristTargetPosition = wristDownPosition;
    }

    public void setLowPosition(){
        pivotTargetPosition = pivotLowPosition;
        extendTargetPosition = extendLowPosition;
        wristTargetPosition = wristLowPosition;
    }

    public void setMidPosition(){
        pivotTargetPosition = pivotMidPosition;
        extendTargetPosition = extendMidPosition;
        wristTargetPosition = wristMidPosition;

    }

    public void setHighPosition(){
        pivotTargetPosition = pivotHighPosition;
        extendTargetPosition = extendHighPosition;
        wristTargetPosition = wristHighPosition;

    }

    public void setClaw1Open(){
        claw1TargetPosition = clawOpenPosition1;
    }

    public void setClaw1Closed(){
        claw1TargetPosition = clawClosedPosition1;
    }

    public void setClaw2Open(){
        claw2TargetPosition = clawOpenPosition2;
    }

    public void setClaw2Closed(){
        claw2TargetPosition = clawClosedPosition2;
    }

    public void setClawOpen(){
        setClaw1Open();
        setClaw2Open();
    }

    public void setClawClosed(){
        setClaw1Closed();
        setClaw2Closed();
    }

    public double getPivotPosition(){
        return armPivot.getCurrentPosition();
    }

    public double getExtendPosition(){
        return armExtend.getCurrentPosition();
    }

    public double getPivotSpeed(){
        return pivotSpeed;
    }

    public double getExtendSpeed(){
        return extendSpeed;
    }

    public double getWristPosition(){
        return wristServo.getPosition();
    }

    public double getClaw1Position(){
        return clawServo1.getPosition();
    }

    public double getClaw2Position(){
        return clawServo2.getPosition();
    }

    public boolean getPixel1Detcted(){
        return colorSensor1.getDistance(DistanceUnit.CM) < 5;
    }

    public boolean getPixel2Detcted(){
        return colorSensor2.getDistance(DistanceUnit.CM) < 5;
    }





}
