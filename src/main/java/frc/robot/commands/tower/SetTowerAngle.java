package frc.robot.commands.tower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tower;

public class SetTowerAngle extends Command {
    private Tower tower;
    private double angle;
    public SetTowerAngle(Tower tower, double angle) {
        this.tower = tower;
        this.angle = angle;
    }

    @Override
    public void execute(){
        tower.setPivotAngle(angle);
    }
    @Override
    public boolean isFinished(){
        return tower.nearAngle(angle);
    }
    @Override
    public void end(boolean interrupted){
        tower.stop();
    }
}
