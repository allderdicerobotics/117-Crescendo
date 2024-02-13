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
        climber.zeroed = false;
    }

    @Override
    public void execute(){
        climber.motionToZero();
    }

    @Override
    public void end(boolean interrupted){
        climber.stopClimber();
        climber.zeroClimber();
    }
    @Override
    public boolean isFinished(){
        climber.zeroed = climber.isClimberZeroed();
        return climber.zeroed;
    }
}
