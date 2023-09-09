package org.firstinspires.ftc.teamcode.manipulator;


import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Intake {

    private DcMotorEx intakeMotor;
    private double power = 0;
    private boolean isToggled = false;

    public Intake(RobotHardware robot){
        intakeMotor = robot.intakeMotor;

    }

    public void loop(){
        intakeMotor.setPower(power);
    }

    public void toggle(){
        isToggled = !isToggled;
        power =  isToggled ? .5 : 0;

    }


}
