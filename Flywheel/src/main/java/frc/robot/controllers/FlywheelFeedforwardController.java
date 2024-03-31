package frc.robot.controllers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Flywheel;

public class FlywheelFeedforwardController extends FlywheelController{
    SimpleMotorFeedforward m_feedforward;
    boolean m_useSlewRateLimiter;
    SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter(FeedforwardConstants.slewRateLimit);

    static class FeedforwardConstants {
        // These constants determined by running SydId on this simulated flywheel
        static double kS = 0.011;   // Volts
        static double kV = 0.00637; // Volts/RPM
        static double kA = 0.0017;  // (Volts/RPM)/second

        // What if we get them a bit wrong -- with these values the steady-state speed is 940RPM
        // when using the slew rate limiter and only 500RPM without it (because the code 
        // below actually puts feedback into the feedforward controller)
        // static double kS = 0.0055;   // Volts
        // static double kV = 0.00315; // Volts/RPM
        // static double kA = 0.0009;  // (Volts/RPM)/second

        static double slewRateLimit = 3000; // RPM/second - get to 1000 in 1/3 second
    }

    public FlywheelFeedforwardController(Flywheel flywheel) {
        this(flywheel, false);
    }

    public FlywheelFeedforwardController(Flywheel flywheel, boolean useSlewRateLimiter) {
        super(flywheel);
        m_feedforward = new SimpleMotorFeedforward(FeedforwardConstants.kS, FeedforwardConstants.kV, FeedforwardConstants.kA);
        m_useSlewRateLimiter = useSlewRateLimiter;
    }

    public double calculate() {
        if (!m_useSlewRateLimiter) {
            return m_feedforward.calculate(m_setpoint); // just uses kV and kS
        } else {
            var lastSetpoint = m_slewRateLimiter.lastValue();
            var newSetpoint = m_slewRateLimiter.calculate(m_setpoint);
            SmartDashboard.putNumber("Flywheel/slew-rate limited setpoint", newSetpoint);
            return m_feedforward.calculate(lastSetpoint, newSetpoint, 0.02); // pure feedforward
            // return m_feedforward.calculate(m_flywheel.getRPM(), newSetpoint, 0.02); // feedback actual RPM
        }
    }
    
}
