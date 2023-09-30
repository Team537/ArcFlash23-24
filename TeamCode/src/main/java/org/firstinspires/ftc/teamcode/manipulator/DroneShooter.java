package org.firstinspires.ftc.teamcode.manipulator;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.Timer;

public class DroneShooter {
    private Servo shooterServo;
    private double shootPosition = 0.3;

    private double currentPosition = 0;
    public ShooterState shooterState = ShooterState.ARMED;

//    private Timing.Timer timer = new Timing.Timer(5);

    public DroneShooter(RobotHardware robot){
        shooterServo = robot.shooterServo;

    }

    public void loop(){
//        shooterServo.setPower(power);
//        telemetry.addData("Shooter State", shooterServo);
//        telemetry.addData("Shooter Power", power);


    }

    public void droneStrike(){
//        timer.start();
//        if(timer.done()) {
//            isToggled = !isToggled;
//            power = isToggled ? .5 : 0;
//            shooterState = isToggled ? ShooterState.FIRED : ShooterState.ARMED;
//        } else {
//            power = 0;
//            shooterState = ShooterState.ARMED;
//        }
        currentPosition = shootPosition;
        shooterServo.setPosition(currentPosition);
        shooterState = ShooterState.FIRED;

    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public enum ShooterState{
        FIRED,
        ARMED
    }
}

