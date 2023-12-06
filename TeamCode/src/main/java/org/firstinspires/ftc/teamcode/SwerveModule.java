package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.Locale;

import javax.xml.parsers.SAXParser;


@Config
public class SwerveModule {
    public static double P = 0.1, I = 0, D = 0, F = 0.2;
    //  f is basically the strength used to turn the servos or motors (power maybe?)
    public static double K_STATIC = 0.03;

    public static double MAX_SERVO = .95, MAX_MOTOR = 0.2; //max speed of either, motor at 20% now for testing

    public static boolean MOTOR_FLIPPING = true;

    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1 / (1.5 * 2 * 2); // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 28;

    private DcMotorEx driveMotor;
    private CRServo servo;
    private AbsoluteAnalogEncoder absoluteAnalogEncoder;
    private PIDFController rotationController;

    public boolean wheelFlipped = false;
    private double targetAngle = 0.0;
    private double moduleAngle = 0.0;
    public double lastMotorPower = 0;
    double rotationTarget = 0;
    double currentAngle = 0;
    double angleError = 0;
    double steerPower = 0;

    double velocity = 0;

    double wheelInverse = 1;

    double velocityPowerAmount = 0.05; //basically a percentage ex. 0.05 = 5% acceleration speed
    private SwerveModuleState state = new SwerveModuleState();


    /**
    * Swerve Module Declaration
    * @param driveMotor DCMotor for Drive
    * @param servo CRServo for Steer
    * @param absoluteAnalogEncoder AnalogEncoder for Absolute Position
    * */
    public SwerveModule(DcMotorEx driveMotor, CRServo servo, AbsoluteAnalogEncoder absoluteAnalogEncoder) {
        this.driveMotor = driveMotor;
        MotorConfigurationType motorConfigurationType = this.driveMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        this.driveMotor.setMotorType(motorConfigurationType);
        this.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        this.servo = servo;
        ((CRServoImplEx) this.servo).setPwmRange(new PwmControl.PwmRange(505, 2495, 5000));

        this.absoluteAnalogEncoder = absoluteAnalogEncoder;
        rotationController = new PIDFController(P, I, D, F);
        this.driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    /**
     * Alternate Swerve Module Declaration
     * @param hardwareMap Robot Hardware Map
     * @param motorName Drive Motor Name
     * @param servoName Servo Name
     * @param absoluteAnalogEncoderName Absolute Encoder Name
     * */

    public SwerveModule(HardwareMap hardwareMap, String motorName, String servoName, String absoluteAnalogEncoderName) {
        this(hardwareMap.get(DcMotorEx.class, motorName),
                hardwareMap.get(CRServo.class, servoName),
                new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, absoluteAnalogEncoderName)));
    }

    /**
     * Read Swerve Module Absolute Angle
     * @return Module Absolute Angle in Radians
     */
    public void read() {
        moduleAngle = absoluteAnalogEncoder.getCurrentPosition();
//        setTargetRotation(0); // dont even try to use this. it will ALWAYS set target position to 0
    }

    /**
     * Update Swerve Module by Adjusting Angle Error in Radians and Drive Power Percentage
     * */
    public void update() {
       read();
        rotationController.setPIDF(P, I, D, F);
        rotationTarget = getTargetRotation();
        currentAngle = getModuleRotation();

         angleError = normalizeRadians(rotationTarget - currentAngle);
        if (MOTOR_FLIPPING && Math.abs(angleError) > Math.PI / 2) {

                //try pi/4 for accuracy?

            rotationTarget = normalizeRadians(rotationTarget - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        angleError = normalizeRadians(rotationTarget - currentAngle);

        steerPower = Range.clip(rotationController.calculate(0, angleError), -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(steerPower)) steerPower = 0;
        servo.setPower(steerPower + (Math.abs(angleError) > 0.02 ? K_STATIC : 0) * Math.signum(steerPower));
    }
    /** Get  Target Module Rotation
     * @return Target Module Rotation in Radians */

    public double getTargetRotation() {
        return normalizeRadians(targetAngle);
    }
    /** Get Actual Module Rotation
     * @return Actual Module Rotation in Radians */
    public double getModuleRotation() {
        return normalizeRadians(moduleAngle);
    }
  /**
   * Set Drive Motor Power
   * @param drivePower Drive Power Percentage
*/
    public void setMotorPower(double drivePower) {
        if (wheelFlipped) drivePower *= -1;
        lastMotorPower = drivePower;
        driveMotor.setPower(drivePower);
    }
    /**
     * Set Target Module Rotation
     * @param targetAngle Target Module Rotation in Radians
     * */
    public void setTargetRotation(double targetAngle) {
        this.targetAngle = normalizeRadians(targetAngle);
    }
    /**
     * Get Module Telemetry
     * @return Telemetry String
     * */
    public String getTelemetry(String moduleName) {
        return String.format(Locale.ENGLISH, "%s: Motor Flipped: %b \ncurrent position %.2f target position %.2f flip modifer = %d motor power = %.2f", moduleName, wheelFlipped, getModuleRotation(), getTargetRotation(), flipModifier(), lastMotorPower);
    }

    public SwerveModuleState getState() {
        return state;
    }
    /**
     * Flip Modifier for Wheel Inversion
     * @return Wheel Inversion Boolean
     * */
    public int flipModifier() {
        return wheelFlipped ? -1 : 1;
    }

    /**
     * Set Drive Motor Run Mode
     * @param runMode DcMotor RunMode
     * */
    public void setDriveMotorMode(DcMotor.RunMode runMode) {
        driveMotor.setMode(runMode);
    }
    /**
     * Set Drive Motor Zero Power Behavior
     * @param zeroPowerBehavior DcMotor ZeroPowerBehavior
     * */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        driveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    /**
     * Set PID Coefficients for Drive Motor
     * @param runMode DcMotor RunMode
     * @param coefficients PID Coefficients for Drive Motor
     * */
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        driveMotor.setPIDFCoefficients(runMode, coefficients);

    }

   /**
   * Get Servo Power
   * @return Servo Power Percentage
   * */
    public double getServoPower() {
        return servo.getPower();
    }
    /**
     * Get Wheel Position
     * @return Drive Motor Position in Inches
     * */
    public double getWheelPosition() {
        return encoderTicksToInches(driveMotor.getCurrentPosition());
    }
    /**
     * Get Wheel Velocity
     * @return Drive Motor Velocity in Inches per Second
     * */
    public double getWheelVelocity() {
        return encoderTicksToInches(driveMotor.getVelocity());
    }


    public void setDesiredState(SwerveModuleState state){

        this.state = state;

                //optimize(state, Rotation2d.fromDegrees(getModuleRotation() *180 / Math.PI));


        if(wheelFlipped){
            wheelInverse = 1;
        } else {
            wheelInverse = -1;
        }


        // DANGEROUS WHEN TESTING --- USE LOWER VALUES FOR TEST IF NOT ON THE GROUND
        //USE VELOCITYpOWERaMOUNT TO SLOW IT DOWN
        velocity = wheelInverse * velocityPowerAmount * ((this.state.speedMetersPerSecond * 39.3701) * TICKS_PER_REV)/ (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO );
        driveMotor.setVelocity(velocity);

        //driveMotor.setVelocity(-10);

        setTargetRotation(state.angle.getRadians());


    }

    public SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {

        double targetAngle = desiredState.angle.getDegrees();
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();

        if(Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180 ) : (targetAngle += 180);
        }

        return new SwerveModuleState(targetSpeed,Rotation2d.fromDegrees(targetAngle));

    }

    private double placeInAppropriateScope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if(lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while ( newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if(newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public double getSteerPower(){
        return steerPower;
    }

    //idk what this is
//    public SwerveModuleState asState() {
//        return new SwerveModuleState(this);
//    }
//
//
//    // Class for Storing The State of a Swerve Module in terms of its Wheel Position and Module Rotation
//    public static class SwerveModuleState {
//        public SwerveModule module;
//        public double wheelPosition, moduleRotation;
//
//
//        /** Class for Storing The State of a Swerve Module in terms of its Wheel Position and Module Rotation
//         * @param swerveModule Swerve Module Object
//         * */
//        public SwerveModuleState(SwerveModule swerveModule) {
//            module = swerveModule;
//            wheelPosition = 0;
//            moduleRotation = 0;
//        }
//        /**
//         * Update SwerveModule State
//         * */
//        public SwerveModuleState update() {
//            return setState(-module.getWheelPosition(), module.getModuleRotation());
//        }
//        /**
//         * Set Swerve Module State
//         * @param moduleRotation Module Rotation in Radians
//         * @param wheelPosition Wheel Position in Ticks
//         * */
//        public SwerveModuleState setState(double wheelPosition, double moduleRotation) {
//            this.wheelPosition = wheelPosition;
//            this.moduleRotation = moduleRotation;
//            return this;
//        }
//
//        //TODO add averaging for podrots based off of past values
//        public Vector2d calculateDelta() {
//            double oldWheelPosition = wheelPosition;
//            update();
//            return Vector2d.polar(wheelPosition - oldWheelPosition, moduleRotation);
//        }
//    }
     /**
      * Convert from Encoder Ticks to Inches
      * @param ticks Encoder Ticks
      * */
    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double getVelocity(){
        return ((wheelInverse * velocityPowerAmount * (state.speedMetersPerSecond * 39.3701) * TICKS_PER_REV)/ (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO ));
        // (wheelInverse * velocityPowerAmount * (state.speedMetersPerSecond * 39.3701) * TICKS_PER_REV)/ (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO ))
    }
}