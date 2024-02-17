// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.climber.ZeroClimbers;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.indexer.SpeakerShot;
import frc.robot.commands.intake.IntakePiece;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.shooter.ReadyShooter;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.tower.MoveTowerDown;
import frc.robot.commands.tower.MoveTowerUp;
import frc.robot.commands.tower.SetTowerAngle;
import frc.robot.commands.tower.ZeroTower;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drive swerve = new Drive();
    // private final Shooter shooter = new Shooter();
    // private final Intake intake = new Intake();
    // private final Indexer indexer = new Indexer();
    // private final Tower tower = new Tower();
    // private final Climber leftClimber = new Climber(Constants.Climber.leftMotorID,false);
    // private final Climber rightClimber = new Climber(Constants.Climber.rightMotorID,true);

    // private final SendableChooser<Command> autoChooser;
    

    PS4Controller driverController = Constants.Operator.driverController;
    Joystick operatorController = Constants.Operator.operatorController;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Make the AutoChooser (default to Driving Backwards with no commands)
        // autoChooser = AutoBuilder.buildAutoChooser("DriveBackAuto");
        // NamedCommands.registerCommand("aimTowerInitial", new SetTowerAngle(tower, Constants.Auto.towerAngleInitial));
        // NamedCommands.registerCommand("rampWheels", new ReadyShooter(shooter));
        // NamedCommands.registerCommand("speakerShoot", new SpeakerShot(indexer));
        // NamedCommands.registerCommand("intakePiece", new IntakePiece(intake, indexer));


        // Command zeroClimbers = new ZeroClimbers(leftClimber, rightClimber);
        // zeroClimbers.schedule();
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
        // Constants.Logging.commandTab.add(autoChooser);
        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {

        /*TODO: Add commands for Automatically Adjusting Shooter
         * Add commands for climber
         */
        
        // Constants.Operator.intakeTrigger
            // .whileTrue(new IntakePiece(intake, indexer));

        // Constants.Operator.resetGyroTrigger
        //         .whileTrue(new RunCommand(() -> swerve.resetOrientation()));

        // Constants.Operator.towerUpTrigger
        //     .whileTrue(new MoveTowerUp(tower));
        
        // Constants.Operator.towerDownTrigger
        //     .whileTrue(new MoveTowerDown(tower));
        
        // Constants.Operator.towerZeroTrigger
        //     .whileTrue(new RunCommand(() -> tower.zero()));//new ZeroTower(tower, null));

        // Constants.Operator.reverseIntakeTrigger
        //     .whileTrue(new ReverseIntake(intake));

        // Constants.Operator.shooterRampTrigger
            // .toggleOnTrue(new ReadyShooter(shooter));

        // Constants.Operator.shootSpeakerTrigger
            // .toggleOnTrue(new SpeakerShot(indexer));

        // Constants.Operator.shootAmpTrigger
            // .whileTrue(new ShootAmp(shooter,indexer));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null; //autoChooser.getSelected();
    }

}
