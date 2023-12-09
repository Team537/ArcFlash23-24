package org.firstinspires.ftc.teamcode.manipulator;

import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class ArmSystem {
    public  double pivotTargetPosition = 0;
    public  double extendTargetPosition = 0;
    public  double claw1TargetPosition = 0.75;
    public  double claw2TargetPosition = 0;
    public  double wristTargetPosition = 0.5;

    public static double pivotDownPosition = 0;
    public static double pivotLowPosition = 10;
    public static double pivotMidPosition = 20;
    public static double pivotHighPosition = 30;

    public static double extendDownPosition = 0;
    public static double extendLowPosition = 10;
    public static double extendMidPosition = 20;
    public static double extendHighPosition = 30;

    public static double clawOpenPosition1 = 0.2;
    public static double clawClosedPosition1 = 0;

    public static double clawOpenPosition2 = 0.3;
    public static double clawClosedPosition2 = 0.45;

    public static double wristDownPosition = 0.6;
    public static double wristLowPosition = 0.4;
    public static double wristMidPosition = 0.5;
    public static double wristHighPosition = 0.6;

    private DcMotorEx armPivot;
    private DcMotorEx armExtend;
    private Servo clawServo1;
    private Servo clawServo2;
    private Servo wristServo;
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    private View relativeLayout;

    private PIDController armPivotController = new PIDController(0.01, 0, 0);
    private static double pivotF = 0.1;

    private PIDController armExtendController = new PIDController(0.01, 0, 0);
    private static double extendF = 0.1;

    private  double pivotSpeed = 0;
    private  double extendSpeed = 0;

    public static double ticks_per_degree = 700/180;

    public ArmSystem(RobotHardware robot){
        armPivot = robot.armPivot;
        armExtend = robot.armExtend;
        clawServo1 = robot.clawServo1;
        clawServo2 = robot.clawServo2;
        wristServo = robot.wristServo;
        colorSensor1 = robot.colorSensor1;
        colorSensor2 = robot.colorSensor2;
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
//
//        double extendPosition = armExtend.getCurrentPosition();
//        double extendPid = armExtendController.calculate(extendTargetPosition-extendPosition);
//        double extendFf = Math.cos(Math.toRadians(extendTargetPosition / ticks_per_degree)) * extendF;
//        extendSpeed = extendPid + extendFf;
//
//        if(Math.abs(extendTargetPosition-extendPosition) > 5) {
//            armExtend.setPower(extendSpeed);
//        } else {
//            armExtend.setPower(0);
//        }

        clawServo1.setPosition(claw1TargetPosition);
        clawServo2.setPosition(claw2TargetPosition);
        wristServo.setPosition(wristTargetPosition);


    }

    public void setDownPosition(){
        pivotTargetPosition = pivotDownPosition;
        extendTargetPosition = extendDownPosition;
       // wristTargetPosition = wristDownPosition;
    }

    public void setLowPosition(){
        pivotTargetPosition = pivotLowPosition;
        extendTargetPosition = extendLowPosition;
        //wristTargetPosition = wristLowPosition;
    }

    public void setMidPosition(){
        pivotTargetPosition = pivotMidPosition;
        extendTargetPosition = extendMidPosition;
        //wristTargetPosition = wristMidPosition;

    }

    public void setHighPosition(){
        pivotTargetPosition = pivotHighPosition;
        extendTargetPosition = extendHighPosition;
        //wristTargetPosition = wristHighPosition;

    }

    public void wristStop(){
        wristTargetPosition = wristMidPosition;
    }

    public void wristUp(){
        wristTargetPosition = wristLowPosition;
    }

    public void wristDown(){
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
