// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Operator;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drive swerve = new Drive();
  private final Operator OI = new Operator();
  private final Vision limelight = new Vision();
  // private final PoseEstimationSubsystem poseEstimationSubsystem = new PoseEstimationSubsystem(swerve, limelight, navX);
  
  XboxController driverController = OI.driverController;
  Joystick operatorController = OI.operatorController;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    swerve.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new TeleopSwerve(
            swerve,
            () -> (driverController.getLeftY()),
            () -> (driverController.getLeftX()),
            () -> (driverController.getRightX()),
            () -> false
        )
    );
    
  }

  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */ 
  public Command getAutonomousCommand() {
    return new WaitCommand(1);
  }

}
