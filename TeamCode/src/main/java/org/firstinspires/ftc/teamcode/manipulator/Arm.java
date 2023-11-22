package org.firstinspires.ftc.teamcode.manipulator;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private DcMotorEx armPivot1;
    private DcMotorEx armPivot2;
    private DcMotorEx armExtend;
    private Servo clawServo1;
    private Servo clawServo2;
    private Servo wristServo;
    private RevColorSensorV3 colorSensor;

}
