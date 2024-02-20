package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IntakePiece extends Command {
    private Intake intake;
    private Indexer indexer;

    public IntakePiece(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {
        /* Run the Intake and Indexer until the piece is in the indexer */
        // System.out.println(indexer.sensorVal());
        
        intake.run();
        indexer.run(0.40);
        
    }
    
    @Override
    public boolean isFinished(){
        // return false;
        return indexer.indexerFilled();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("here");
        indexer.stop();
        intake.stop();
    }
}
