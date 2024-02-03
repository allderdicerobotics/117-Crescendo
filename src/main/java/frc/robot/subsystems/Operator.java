package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Operator extends SubsystemBase{
    /*TODO: Integrate Buttons and Operator Board within this file */
    public XboxController driverController = new XboxController(JoystickConstants.driverControllerPort);
    public Joystick operatorController = new Joystick(JoystickConstants.buttonBoardPort);
    
    public static final class JoystickConstants{
        public static final int driverControllerPort = 0;
        public static final int buttonBoardPort = 2;
        
        public static final int translationAxis = 1;
        public static final int strafeAxis = 0;
        public static final int rotAxis = 2;

        public static final int intakeToggleButton = 1;
        public static final int shooterCmdButton = 2;
        public static final int towerCmdButton = 3;
        public static final int climberToggleButton = 4;
        public static final int resetGyroButton = 5;
    }
}
