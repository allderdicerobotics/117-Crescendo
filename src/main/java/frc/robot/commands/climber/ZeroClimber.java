package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends Command{
    private Climber climber;
    public ZeroClimber(Climber climber){
        this.climber = climber;
    }
    
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        climber.runZero();
    }

    @Override
    public void end(boolean interrupted){
        climber.stop();
        climber.zero();
    }
    @Override
    public boolean isFinished(){
        return climber.isClimberZeroed();
    }
}
