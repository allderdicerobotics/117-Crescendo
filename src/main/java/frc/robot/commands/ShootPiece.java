package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootPiece extends Command {
    private Shooter shooter;
    private Indexer indexer;

    public ShootPiece(Shooter shooter, Indexer indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
    }

    @Override
    public void execute() {
        /* Run the Shooter Motor and achieve full speed
         * -> spit the piece out of the indexer at full speed into shooter wheels
         */
        shooter.run(Constants.Shooter.maxRPM);
        if (shooter.nearSpeed(Constants.Shooter.maxRPM)) {
            indexer.run();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stop();
    }

}
