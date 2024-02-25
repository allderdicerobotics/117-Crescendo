package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.misc.SquaredSlewRate;
import frc.robot.subsystems.Drive;

public class TeleopSwerve extends Command {
    private Drive swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotSup;
    private BooleanSupplier FOCSup;
    private SquaredSlewRate slewRate;

    public TeleopSwerve(Drive swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotSup,
            BooleanSupplier FOCSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotSup = rotSup;
        this.FOCSup = FOCSup;
        slewRate = new SquaredSlewRate(0.3, Constants.Swerve.stickDeadband);
    }

    @Override
    public void execute() {
        /* Apply Deadband to controller inputs
         * -> feed joystick inputs into field-relative drive
         */
        double translationVal = slewRate.calculate(translationSup.getAsDouble());
        double strafeVal = slewRate.calculate(strafeSup.getAsDouble());
        double rotationVal = slewRate.calculate(rotSup.getAsDouble()); 
        // double translationVal = -MathUtil.applyDeadband(
        //     Math.copySign(Math.pow(translationSup.getAsDouble(),2), translationSup.getAsDouble())
        //     , Constants.Swerve.stickDeadband);
        // double strafeVal = -MathUtil.applyDeadband(
        //     Math.copySign(Math.pow(strafeSup.getAsDouble(),2), strafeSup.getAsDouble())
        //     , Constants.Swerve.stickDeadband);
        // double rotationVal = -MathUtil.applyDeadband(
        //     Math.copySign(Math.pow(rotSup.getAsDouble(),2), rotSup.getAsDouble())
        //     , Constants.Swerve.stickDeadband);;
       
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                FOCSup.getAsBoolean(),
                false);
    }
}
