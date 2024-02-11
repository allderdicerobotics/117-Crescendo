package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Shooter;

public class ShootManual extends Command {
    private Shooter shooter;
    public ShootManual(Shooter shooter){
        this.shooter = shooter;
    }

    @Override
    public void execute(){
        shooter.shoot(Constants.Shooter.maxRPM);
    }
    
    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }
    
}
