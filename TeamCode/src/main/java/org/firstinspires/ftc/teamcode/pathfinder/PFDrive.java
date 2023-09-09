package org.firstinspires.ftc.teamcode.pathfinder;

import org.firstinspires.ftc.teamcode.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SwerveModule;

import java.util.function.Function;
import me.wobblyyyy.pathfinder2.control.Controller;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.kinematics.RelativeSwerveDriveKinematics;
import me.wobblyyyy.pathfinder2.kinematics.RelativeSwerveModuleKinematics;
import me.wobblyyyy.pathfinder2.kinematics.RelativeSwerveModuleState;
import me.wobblyyyy.pathfinder2.kinematics.RelativeSwerveState;
import me.wobblyyyy.pathfinder2.robot.Drive;


public class PFDrive implements Drive {

    private final PFModule frontRightModule;

    private final  PFModule frontLeftModule;

    private final  PFModule backRightModule;

    private final  PFModule backLeftModule;

    private final RelativeSwerveDriveKinematics kinematics;

    public static double frontLeftOffset = 0, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = 0;

    private Translation translation = Translation.zero();

    private Function<Translation, Translation> modifier = s -> s;

    /**
     * Create a new swerve drive.
     *
     * @param moduleController the turn controller used to control the swerve
     *                         module's turn angle. This controller accepts
     *                         degrees as input and as the target.
     * @param turnCoefficient  the coefficient used in calculating how fast
     *                         the chassis should turn. Higher values make
     *                         the robot turn faster, and lower values make
     *                         the robot turn slower. This value should usually
     *                         be around 0.1, but it'll take some testing to
     *                         figure out what number works best for you.
     */
    public PFDrive(
            RobotHardware robot,
            Controller moduleController,
            double turnCoefficient
    ) {
        this.frontRightModule = new PFModule(robot.frontRightMotor, robot.frontRightServo, new AbsoluteAnalogEncoder(robot.frontRightEncoder, 3.3).zero(frontRightOffset).setInverted(true));
        this.frontLeftModule = new PFModule(robot.frontLeftMotor, robot.frontLeftServo, new AbsoluteAnalogEncoder(robot.frontLeftEncoder, 3.3).zero(frontLeftOffset).setInverted(true));
        this.backRightModule = new PFModule(robot.backRightMotor, robot.backRightServo, new AbsoluteAnalogEncoder(robot.backRightEncoder, 3.3).zero(backRightOffset).setInverted(true));
        this.backLeftModule = new PFModule(robot.backLeftMotor, robot.backLeftServo, new AbsoluteAnalogEncoder(robot.backLeftEncoder, 3.3).zero(backLeftOffset).setInverted(true));

        this.kinematics =
                new RelativeSwerveDriveKinematics(
                        new RelativeSwerveModuleKinematics(moduleController),
                        frontRightModule::getAngle,
                        frontLeftModule::getAngle,
                        backRightModule::getAngle,
                        backLeftModule::getAngle,
                        turnCoefficient
                );
    }

    /**
     * Get the last translation that was set to the robot.
     *
     * @return the last translation that was set to the robot.
     */
    @Override
    public Translation getTranslation() {
        return translation;
    }

    /**
     * Set a translation to the robot. In the case of this swerve chassis,
     * this does a couple of things. Firstly, it applies the modifier to
     * the inputted translation. Secondly, it calculates a swerve chassis
     * state based on the translation. Thirdly, it determines the swerve
     * module states of each individual module. And finally, it sets the
     * state to each of these modules, making the robot move.
     *
     * @param translation a translation the robot should act upon. This
     *                    translation should always be <em>relative</em>,
     *                    meaning whatever the translation says should make
     *                    the robot act accordingly according to the robot's
     *                    position and the robot's current heading. I'm
     *                    currently exhausted and just about entirely unable
     *                    to type, so this isn't coherent, but guess what -
     */
    @Override
    public void setTranslation(Translation translation) {
        translation = modifier.apply(translation);

        this.translation = translation;

        RelativeSwerveState state = kinematics.calculate(translation);

        RelativeSwerveModuleState frState = state.fr();
        RelativeSwerveModuleState flState = state.fl();
        RelativeSwerveModuleState brState = state.br();
        RelativeSwerveModuleState blState = state.bl();

        this.frontRightModule.set(frState);
        this.frontLeftModule.set(flState);
        this.backRightModule.set(brState);
        this.backLeftModule.set(blState);
    }

    /**
     * Get the modifier.
     *
     * @return the modifier.
     */
    @Override
    public Function<Translation, Translation> getDriveModifier() {
        return this.modifier;
    }

    /**
     * Set the modifier.
     *
     * @param modifier the modifier. This modifier... well, it literally just
     *                 modifies any translations that the robot is given.
     *                 For example, if you want to invert the X and Y values
     *                 of any given translation, you could do so by using
     *                 a modifier.
     */
    @Override
    public void setDriveModifier(Function<Translation, Translation> modifier) {
        this.modifier = modifier;
    }

    public String getTelemetry() {

        return frontLeftModule.getTelemetry("leftFrontModule") + "\n" +
                backLeftModule.getTelemetry("leftRearModule") + "\n" +
                frontRightModule.getTelemetry("rightFrontModule") + "\n" +
                backRightModule.getTelemetry("rightRearModule") + "\n";
    }
}
