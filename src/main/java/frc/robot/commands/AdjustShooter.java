package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Vision;

public class AdjustShooter extends Command {
    private Vision limelight;
    private PoseEstimator poseEstimator;
    private Tower tower;

    public AdjustShooter(Vision limelight, Tower tower, PoseEstimator poseEstimator) {
        this.limelight = limelight;
        this.tower = tower;
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void execute() {
        /* Get Distance from Robot to Speaker 
         * -> interpolate an angle from existing data using this distance
         * -> if the angle isn't to extreme, make it the setpoint
         */

        double dist = limelight.getDistSpeaker(poseEstimator.getPose());
        var angle = tower.interpolateAngle(dist);
        if (Math.abs(tower.getPivotAngle() - angle) < Constants.Tower.threshAngle * 10) {
            tower.setPivotAngle(angle);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
