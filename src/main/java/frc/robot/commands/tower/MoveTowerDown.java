package frc.robot.commands.tower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tower;

public class MoveTowerDown extends Command{
    private Tower tower;
    public MoveTowerDown(Tower tower){
        this.tower = tower;
        addRequirements(tower);
    }
    
    @Override
    public void execute(){
        tower.moveTowerDown();
    }

    @Override
    public void end(boolean interrupted){
        tower.stop();
    }
}
