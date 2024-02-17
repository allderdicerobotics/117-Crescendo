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
    public void execute() {
        /* Run the Intake and Indexer until the piece is in the indexer */
        System.out.println(indexer.sensorVal());
        // while (!indexer.indexerFilled()) {
        intake.run();
        indexer.run(1);
        // }
    }
    
    @Override
    public boolean isFinished(){
        return indexer.indexerFilled();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        intake.stop();
    }
}
