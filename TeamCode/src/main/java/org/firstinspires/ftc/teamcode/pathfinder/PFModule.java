package org.firstinspires.ftc.teamcode.pathfinder;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.AbsoluteAnalogEncoder;

import java.util.Locale;
import java.util.function.Supplier;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.kinematics.RelativeSwerveModuleState;
import me.wobblyyyy.pathfinder2.robot.components.Motor;


@Config
public class PFModule {



    private final DcMotorEx driveMotor;
    private final CRServo servo;
    private final AbsoluteAnalogEncoder absoluteAnalogEncoder;
    private RelativeSwerveModuleState targetState;

    public static double  MAX_SERVO = .95, MAX_MOTOR = 0.2;




    public PFModule(DcMotorEx drive, CRServo turn, AbsoluteAnalogEncoder encoder) {
        servo = turn;
        driveMotor= drive;
        absoluteAnalogEncoder = encoder;

        MotorConfigurationType motorConfigurationType = this.driveMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        this.driveMotor.setMotorType(motorConfigurationType);
        this.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ((CRServoImplEx) this.servo).setPwmRange(new PwmControl.PwmRange(505, 2495, 5000));


    }

    /**
     * Get the module's turn servo.
     *
     * @return the module's turn servo.
     */
    public CRServo turn() {
        return this.servo;
    }

    /**
     * Get the module's drive motor.
     *
     * @return the module's drive motor.
     */
    public DcMotorEx drive() {
        return this.driveMotor;
    }

    /**
     * Set a swerve module state to the swerve module. This will set power
     * to both the turn servo and drive motor.
     *
     * @param state the state to set to the turn module.
     */
    public void set(RelativeSwerveModuleState state) {
        state = targetState;
        servo.setPower(Range.clip(state.getTurn(),-MAX_SERVO,MAX_SERVO));
        driveMotor.setPower(state.getDrive());
    }

    /**
     * Get the angle at which the swerve module is currently facing.
     *
     * @return the angle the swerve module is currently facing.
     */
    public Angle getAngle() {
        return new Angle( 360 * absoluteAnalogEncoder.getCurrentPosition() / 4096);
    }

    public String getTelemetry(String moduleName) {
        return String.format(Locale.ENGLISH,
                "%s: Motor Direction: %s " +
                        "\nServo Direction: %s" +
                        "\nCurrent Angle =  %.2f " +
                        "\nMotor Power = %.2f " +
                        "\nServo Power = %.2f " +
                        "\nMotor Current  = %.2f" +
                        "\nTarget Drive Power = %.2f" +
                        "\nTarget Turn Power = %.2f"  ,
                moduleName,
                driveMotor.getDirection().toString(),
                servo.getDirection().toString(),
                getAngle(),
                driveMotor.getPower(),
                servo.getPower(),
                driveMotor.getCurrent( CurrentUnit.AMPS),
                targetState.getDrive(),
                targetState.getTurn());
    }
}