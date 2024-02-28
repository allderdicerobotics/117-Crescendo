package frc.robot.misc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class SquaredSlewRate {
    private double slewRate;
    private double deadband;
    private SlewRateLimiter slewRateLimiter;
    public SquaredSlewRate(double slewRate, double deadband){
        this.slewRate = slewRate;
        this.deadband = deadband;
        slewRateLimiter = new SlewRateLimiter(this.slewRate);

    }

    public double calculate(double joystickInput){
         double joystickVal = MathUtil.applyDeadband(
            Math.copySign(Math.pow(joystickInput,2), joystickInput)
            , this.deadband);
        return -this.slewRateLimiter.calculate(joystickVal);
    }
}
