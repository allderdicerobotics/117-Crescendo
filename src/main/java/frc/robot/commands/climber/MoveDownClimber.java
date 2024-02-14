package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveDownClimber extends Command{
    private Climber climber;
    
    public MoveDownClimber(Climber climber){
        this.climber = climber;
        
        // addRequirements(climber); We want to be able to run both climbers at same time
    }

    @Override
    public void execute(){
        climber.moveDown();
    }

    @Override
    public void end(boolean interrupted){
        climber.stop();
    }
}
