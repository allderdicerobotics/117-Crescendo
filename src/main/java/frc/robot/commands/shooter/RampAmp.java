package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Shooter;

public class RampAmp extends Command{
    private Shooter shooter;
    public RampAmp(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        shooter.runAmp(Constants.Shooter.apmRPM);
    }

    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }
}
