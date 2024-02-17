package frc.robot.commands.tower;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tower;

public class ZeroTower extends Command{
    private Tower tower;
    private DigitalInput hallEffect;
    
    public ZeroTower(Tower tower, DigitalInput hallEffect){
        this.tower = tower;
        this.hallEffect = hallEffect;
        addRequirements(tower);
    }

    @Override
    public void execute(){
        tower.moveTower(-0.1);
    }

    @Override
    public boolean isFinished(){
        return hallEffect.get();
    }

    @Override
    public void end(boolean interrupted){
        tower.stop();
        if (!interrupted){
            tower.zero();    
        }
    }
}
