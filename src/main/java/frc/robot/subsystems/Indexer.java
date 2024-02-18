package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.playingwithfusion.TimeOfFlight.Status;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Indexer extends SubsystemBase {
    private CANSparkMax indexMotor;//, invIndexMotor;
    private TimeOfFlight beamBreakSensor;

    public Indexer() {
        indexMotor = new CANSparkMax(Constants.Indexer.motorID, MotorType.kBrushless);
        // invIndexMotor = new CANSparkMax(Constants.Indexer.invMotorID, MotorType.kBrushless);

        beamBreakSensor = new TimeOfFlight(Constants.Indexer.sensorID);
        indexMotor.restoreFactoryDefaults();
        // invIndexMotor.restoreFactoryDefaults();

        indexMotor.setSmartCurrentLimit(20);
        // invIndexMotor.setSmartCurrentLimit(20);

        beamBreakSensor.setRangingMode(RangingMode.Short, 24); // sample time min is 24 ms
        indexMotor.setIdleMode(IdleMode.kBrake); // instantly stopping intake is important to ensure Note won't slide
                                                 // any further
        indexMotor.setInverted(true);
        // invIndexMotor.follow(indexMotor,true);
    }

    public void run(double speed) {
        indexMotor.set(speed);
    }

    public void stop() {
        indexMotor.set(0);
        // indexMotor.stopMotor();
    }

    public boolean indexerFilled() {
        if (beamBreakSensor.getStatus() == Status.Valid) {
            return beamBreakSensor.getRange() <= Constants.Indexer.threshDist;
        }
        return false;
    }
    public double sensorVal(){
        return beamBreakSensor.getRange();
    }
    // @Override
    // public void periodic(){
    //     System.out.println(sensorVal());
    // }
}
