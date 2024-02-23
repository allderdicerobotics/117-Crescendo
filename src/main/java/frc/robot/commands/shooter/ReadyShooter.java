package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Shooter;

public class ReadyShooter extends Command {
    private Shooter shooter;

    public ReadyShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        shooter.run(Constants.Shooter.speakerRPM);
    }
    @Override
    public void execute() {
        /* Run the Shooter Motor and achieve full speed
            -> prepares for shot
         */
         
    }

    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

}
