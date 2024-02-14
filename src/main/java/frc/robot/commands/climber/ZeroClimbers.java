package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;

public class ZeroClimbers extends ParallelCommandGroup {
    private Climber leftClimber, rightClimber;

    public ZeroClimbers(Climber leftClimber, Climber rightClimber) {
        this.leftClimber = leftClimber;
        this.rightClimber = rightClimber;

        addCommands(
                new ZeroClimber(leftClimber),
                new ZeroClimber(rightClimber));

    }
}
