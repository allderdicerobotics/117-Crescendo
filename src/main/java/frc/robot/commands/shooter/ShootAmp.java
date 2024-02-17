package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {
    private Shooter shooter;
    private Indexer indexer;

    public ShootAmp(Shooter shooter, Indexer indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        /* Run the Shooter Motor and achieve full speed
         * -> spit the piece out of the indexer at full speed into shooter wheels
         */
        // System.out.println(indexer.indexerFilled());
        // System.out.println(C)
        shooter.run(Constants.Shooter.apmRPM);
        if (shooter.atSpeed(Constants.Shooter.apmRPM)) {
            indexer.run(0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stop();
    }

}
