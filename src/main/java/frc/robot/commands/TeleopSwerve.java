package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopSwerve extends Command{
    private DriveSubsystem swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotSup;
    private BooleanSupplier FOCSup;

    public TeleopSwerve (DriveSubsystem swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotSup, BooleanSupplier FOCSup){
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotSup = rotSup;
        this.FOCSup = FOCSup;
    }

    @Override
    public void execute(){
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotSup.getAsDouble(), Constants.Swerve.stickDeadband);
            /* Drive */
        swerve.drive(
            new Translation2d(translationVal * Constants.Swerve.maxSpeed, strafeVal * Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity,
            FOCSup.getAsBoolean(),
            true
        );
    }
}
