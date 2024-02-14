package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ReverseIntake extends Command{
    private Intake intake;
    public ReverseIntake(Intake intake){
        this.intake = intake;
    }
    
    @Override
    public void execute(){
        intake.reverse();
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }
}
