package org.firstinspires.ftc.teamcode.manipulator;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.Timer;

public class DroneShooter {
    private CRServo shooterServo;
    private double power = 0;
    private boolean isToggled = false;
    private ShooterState shooterState = ShooterState.STOPPED;
    private Timing.Timer timer = new Timing.Timer(5);

    public DroneShooter(RobotHardware robot){
        shooterServo = robot.shooterServo;

    }

    public void loop(){
        shooterServo.setPower(power);
//        telemetry.addData("Shooter State", shooterServo);
//        telemetry.addData("Shooter Power", power);


    }

    public void shoot(){
        timer.start();
        if(timer.done()) {
            isToggled = !isToggled;
            power = isToggled ? .5 : 0;
            shooterState = isToggled ? ShooterState.RUNNING : ShooterState.STOPPED;
        } else {
            power = 0;
            shooterState = ShooterState.STOPPED;
        }

    }

    public enum ShooterState{
        RUNNING,
        STOPPED
    }
}

