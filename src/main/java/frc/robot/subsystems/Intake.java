package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);

        configIntakeMotor();
    }

    public void run() {
        intakeMotor.set(1);
    }
    public void reverse(){
        intakeMotor.set(-1.0);
    }

    public void stop() {
        intakeMotor.set(0);
        // intakeMotor.stopMotor();
    }

    private void configIntakeMotor() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(Constants.Intake.currentLimit);
        intakeMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setInverted(true);

        intakeMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus0,
            0
        );

        intakeMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus1,
            0
        );

        intakeMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus2,
            0
        );

        intakeMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus3,
            0
        );

        intakeMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus4,
            0   
        );

        intakeMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus5,
            0
        );

        intakeMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus6,
            0
        );
        
        // intakeMotor.setOpenLoopRampRate(0.5);
        intakeMotor.burnFlash();

    }

}
