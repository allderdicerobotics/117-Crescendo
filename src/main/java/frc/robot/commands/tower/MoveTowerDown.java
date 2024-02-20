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
    public void initialize(){
        tower.moveTower(-0.1);
    }

    @Override
    public void end(boolean interrupted){
        tower.stop();
        tower.setPivotAngle(tower.getPivotAngle());
    }
}
