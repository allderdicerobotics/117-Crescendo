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
        System.out.println("Shoot Piece from Indexer Auto");
        indexer.run(0.5);
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
    }
}
