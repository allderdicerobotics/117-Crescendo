package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class SourceIntake extends Command {
    private Indexer indexer;
    private Shooter shooter;
    private boolean pieceInIndexer = false;

    public SourceIntake(Shooter shooter, Indexer indexer){
        this.shooter = shooter;
        this.indexer = indexer;
        
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize(){
        shooter.runSource();
        indexer.run(-0.1);
    }
    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        // System.out.println(indexer.indexerFilled());
        if (indexer.indexerFilled()){
            pieceInIndexer = true;
        }
        if (!indexer.indexerFilled() && pieceInIndexer){
            pieceInIndexer = false;
            return true;
        }
        return false;        
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
        shooter.stop();
    }
}
