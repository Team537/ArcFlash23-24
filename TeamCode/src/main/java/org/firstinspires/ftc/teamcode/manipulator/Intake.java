package org.firstinspires.ftc.teamcode.manipulator;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Intake {

    private DcMotorEx intakeMotor;
    private double power = 0;
    private boolean isToggled = false;
    private IntakeState intakeState = IntakeState.STOPPED;

    public Intake(RobotHardware robot){
//        intakeMotor = robot.intakeMotor;

    }

    public void loop(){
      intakeMotor.setPower(power);
//        telemetry.addData("Intake State", intakeState);
//        telemetry.addData("Intake Power", power);

    }

    public IntakeState getIntakeState(){
        return intakeState;
    }

    public void run(){
        power = 1;
        intakeState = IntakeState.RUNNING ;
    }

    public void stop(){
        power = 0;
        intakeState = IntakeState.STOPPED ;
    }

    public enum IntakeState{
        RUNNING,
        STOPPED
    }


}
