package frc.robot.sensors;

import java.util.function.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class ThriftyEncoder implements Supplier<Rotation2d> {
    private final AnalogInput encoder;
    private Rotation2d offset = new Rotation2d(0.0);

    public ThriftyEncoder(AnalogInput encoder) {
        this.encoder = encoder;
    }

    public void configure(boolean inverted) {
        // none are inverted, absolute encoders don't need configuration
    }

    private double getRawPositionHelper() {
        /* Read Angle from Absolute Encoder with respect to True Voltage of 5V Rail */
        return ((encoder.getAverageVoltage() / RobotController.getVoltage5V()) * (Math.PI * 2));
    }

    public Rotation2d getRawPosition() {
        return new Rotation2d(getRawPositionHelper());
    }

    // Does include offset
    public Rotation2d get() {
        return getRawPosition().plus(offset);
    }

    public double getDegrees(){
        return getRawPosition().getDegrees();
    }
    // Update the offset (radians)
    public ThriftyEncoder shiftRads(double radians) {
        offset = offset.plus(new Rotation2d(radians));
        return this;
    }

    // Update the offset (degrees)
    public ThriftyEncoder shiftDegs(double degrees) {
        offset = offset.plus(Rotation2d.fromDegrees(degrees));
        return this;
    }
}
