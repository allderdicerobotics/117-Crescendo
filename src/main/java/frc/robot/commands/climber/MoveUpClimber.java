package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveUpClimber extends Command{
    private Climber climber;
    
    public MoveUpClimber(Climber climber){
        this.climber = climber;
        
        // addRequirements(climber); We want to be able to run both climbers at same time
    }

    @Override
    public void initialize(){
        climber.moveUp();
    }

    @Override
    public void end(boolean interrupted){
        climber.stop();
    }
}
