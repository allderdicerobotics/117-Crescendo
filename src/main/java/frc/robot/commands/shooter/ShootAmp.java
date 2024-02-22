package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {
    private Shooter shooter;
    private Indexer indexer;
    private boolean setIndexer = false;
    public ShootAmp(Shooter shooter, Indexer indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize(){
        setIndexer = false;
        shooter.runAmp(Constants.Shooter.apmRPM);
        
    }
    @Override
    public void execute() {
        /* Run the Shooter Motor and achieve full speed
         * -> spit the piece out of the indexer at full speed into shooter wheels
         */
        if ((shooter.atSpeed(Constants.Shooter.apmRPM) || shooter.aboveSpeed(Constants.Shooter.apmRPM)) && !setIndexer) {
            Timer.delay(0.5);
            indexer.run(0.35);
            setIndexer = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stop();
    }

}
