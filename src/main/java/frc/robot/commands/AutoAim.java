// package frc.robot.commands;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.misc.Constants;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Vision;

// public class AutoAim extends Command{
//     private Vision limelight;
//     private Drive swerve;
//     private static boolean finished;
//     public AutoAim(Vision limelight, Drive swerve){
//         this.limelight = limelight;
//         this.swerve = swerve;
//     }
//     @Override
//     public void execute(){
//         finished = false;
//         while ((Math.abs(limelight.getX()) > Constants.Vision.autoAimThresh)){    
//             double adjustSpeed = limelight.getX() * Constants.Vision.autoAimAdjustP;
//             swerve.driveRobotRelative(
//                 new ChassisSpeeds(
//                     0,
//                     0,
//                     adjustSpeed
//                 )
//             );
//         }
//         finished = true;
//     }
//     @Override
//     public boolean isFinished(){
//         return finished;
//     }
//     @Override
//     public void end(boolean interrupted){
//         swerve.driveRobotRelative(new ChassisSpeeds());
//     }
// }
