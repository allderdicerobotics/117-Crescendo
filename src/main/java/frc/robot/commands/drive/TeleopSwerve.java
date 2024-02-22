package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Drive;

public class TeleopSwerve extends Command {
    private Drive swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotSup;
    private BooleanSupplier FOCSup;

    public TeleopSwerve(Drive swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotSup,
            BooleanSupplier FOCSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotSup = rotSup;
        this.FOCSup = FOCSup;
    }

    @Override
    public void execute() {
        /* Apply Deadband to controller inputs
         * -> feed joystick inputs into field-relative drive
         */
        double translationVal = -MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
        double strafeVal = -MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
        double rotationVal = -MathUtil.applyDeadband(rotSup.getAsDouble(), Constants.Swerve.stickDeadband);
       
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                FOCSup.getAsBoolean(),
                false);
    }
}