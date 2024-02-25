package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class ReverseIntake extends Command{
    private Intake intake;
    private Indexer indexer;
    public ReverseIntake(Intake intake, Indexer indexer){
        this.intake = intake;
        this.indexer = indexer;
    }
    
    @Override
    public void initialize(){
        intake.reverse();
        indexer.run(-0.5);
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
        indexer.stop();
    }
}
