package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);

        configIntakeMotor();
    }

    public void run() {
        intakeMotor.set(0.4);
    }
    public void reverse(){
        intakeMotor.set(-0.4);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    private void configIntakeMotor() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(Constants.Intake.currentLimit);
        intakeMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setInverted(true);

        // intakeMotor.setOpenLoopRampRate(0.5);
        intakeMotor.burnFlash();

    }

}
