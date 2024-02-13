// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.intake.IntakePiece;
import frc.robot.commands.shooter.ShootPiece;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drive swerve = new Drive();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Indexer indexer = new Indexer();
    private final Climber leftClimber = new Climber(Constants.Climber.leftMotorID);
    private final Climber rightClimber = new Climber(Constants.Climber.rightMotorID);
    private final SendableChooser<Command> autoChooser;

    PS4Controller driverController = Constants.Operator.driverController;
    Joystick operatorController = Constants.Operator.operatorController;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Make the AutoChooser (default to Driving Backwards with no commands)
        autoChooser = AutoBuilder.buildAutoChooser("DriveBackAuto");


        // Configure Button Bindings 
        configureButtonBindings();
        
        // Default Command for SwerveDrive is TeleOpSwerve
        swerve.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new TeleopSwerve(
                swerve,
                () -> (driverController.getLeftY()),
                () -> (driverController.getLeftX()),
                () -> (driverController.getRightX()),
                () -> true
            )
        );
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {

        /*TODO: Add commands for Automatically Adjusting Shooter
         * Add commands for climber
         */
        
        Constants.Operator.intakeTrigger
                .whileTrue(new IntakePiece(intake, indexer));

        Constants.Operator.resetGyroTrigger
                .whileTrue(new RunCommand(() -> swerve.resetOrientation()));

        Constants.Operator.shooterTrigger
                .whileTrue(new ShootPiece(shooter, indexer));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
