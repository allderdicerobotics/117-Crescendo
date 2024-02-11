package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IntakePiece extends Command{
    private Intake intake;
    private Indexer indexer;

    public IntakePiece(Intake intake, Indexer indexer){
        this.intake = intake;
        this.indexer = indexer;
    }

    @Override
    public void execute(){
        while (!indexer.indexerFilled()){
            intake.run();
        }
    }
    @Override
    public void end(boolean interrupted){
        
    }
}
