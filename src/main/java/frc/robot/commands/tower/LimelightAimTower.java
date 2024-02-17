package frc.robot.commands.tower;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Vision;

public class LimelightAimTower extends Command {
    private Vision limelight;
    private PoseEstimator poseEstimator;
    private Tower tower;

    public LimelightAimTower(Tower tower, PoseEstimator poseEstimator) {
        this.tower = tower;
        this.poseEstimator = poseEstimator;
        addRequirements(tower);
    }

    @Override
    public void execute() {
        /* Get Distance from Robot to Speaker 
         * -> interpolate an angle from existing data using this distance
         * -> if the angle isn't to extreme, make it the setpoint
         */

        double dist = poseEstimator.getDistSpeaker(poseEstimator.getPose());
        var angle = tower.interpolateAngle(dist);
        if (Math.abs(tower.getPivotAngle() - angle) < Constants.Tower.threshAngle) {
            tower.setPivotAngle(angle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        tower.stop();
    }
}
