package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IndexerIntake extends Command{
    private Indexer indexer;

    public IndexerIntake(Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize(){
        indexer.run(0.35); // run at 35%
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
    }
}
