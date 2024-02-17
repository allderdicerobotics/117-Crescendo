package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.SerialPort;

public final class NavX {
    private static AHRS ahrs = new AHRS(SerialPort.Port.kUSB);

    public NavX() {

    }

    public static double getRate() {
        return ahrs.getRate();
    }

    public Rotation2d getAngle() {
        return ahrs.getRotation2d();
    }

    public double getDegrees(){
        return getAngle().getDegrees();
    }

    public void zeroYaw() {
        ahrs.zeroYaw();
    }

    public void reset() {
        ahrs.reset();
    }

    
}
