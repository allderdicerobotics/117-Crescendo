package frc.robot.commands.tower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tower;

public class ZeroTower extends Command {
    private Tower tower;

    public ZeroTower(Tower tower) {
        this.tower = tower;
        addRequirements(tower);
    }

    @Override
    public void initialize() {
        tower.moveTower(-0.1);
    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished() {
        return tower.getOutputCurrent() > 15;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("here");
        tower.stop();
        tower.zero();
    }

}
