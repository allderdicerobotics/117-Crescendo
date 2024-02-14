package frc.robot.commands.tower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tower;

public class MoveTowerUp extends Command{
    private Tower tower;
    public MoveTowerUp(Tower tower){
        this.tower = tower;
    }
    
    @Override
    public void execute(){
        tower.moveTowerUp();
    }

    @Override
    public void end(boolean interrupted){
        tower.stop();
    }
}
