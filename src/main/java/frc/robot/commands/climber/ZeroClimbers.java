package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;

public class ZeroClimbers extends ParallelCommandGroup {

    public ZeroClimbers(Climber leftClimber, Climber rightClimber) {

        addCommands(
                new ZeroClimber(leftClimber),
                new ZeroClimber(rightClimber));

    }
}
