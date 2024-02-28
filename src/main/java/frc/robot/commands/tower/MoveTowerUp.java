package frc.robot.commands.tower;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tower;

public class MoveTowerUp extends Command{
    private Tower tower;
    public MoveTowerUp(Tower tower){
        this.tower = tower;
        addRequirements(tower);
    }
    
    @Override
    public void initialize(){
        tower.moveTower(0.1);
    }

    @Override
    public void end(boolean interrupted){
        tower.stop();
        Timer.delay(0.5);
        tower.setPivotAngle(tower.getPivotAngle());
    }
}
