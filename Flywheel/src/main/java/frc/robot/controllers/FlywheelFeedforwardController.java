package frc.robot.controllers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Flywheel.Constants;

public class FlywheelFeedforwardController extends FlywheelController{
    SimpleMotorFeedforward m_feedforward;
    boolean m_useSlewRateLimiter;
    SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter(Constants.kSlewRateLimit);

    public FlywheelFeedforwardController(Flywheel flywheel) {
        this(flywheel, false);
    }

    public FlywheelFeedforwardController(Flywheel flywheel, boolean useSlewRateLimiter) {
        super(flywheel);
        m_feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
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
