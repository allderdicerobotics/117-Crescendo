package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class SpeakerShot extends Command{
    private Indexer indexer;
    public SpeakerShot(Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize(){
        indexer.run(0.5); // run at 50%
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
    }
}
