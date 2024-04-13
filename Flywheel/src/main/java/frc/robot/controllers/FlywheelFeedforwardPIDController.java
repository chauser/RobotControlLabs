package frc.robot.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Flywheel.Constants;

public class FlywheelFeedforwardPIDController extends FlywheelController{
    SimpleMotorFeedforward m_feedforward;
    PIDController m_pid;
    double m_lastFeedback = 0.0;
    boolean m_useSlewRateLimiter;
    SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter(Constants.kSlewRateLimit);

    static class PIDConstants {
        // These constants determined by running SydId on this simulated flywheel
        // and dividing by 60 to convert SysID's RPS calculation to RPM
        // This achieves steady state of 540 RPM on a setting of 1000RPM 
        // static double kP = 0.00744;   // Volts/RPM
        // static double kD = 0.0;       // Volts/(RPM/second)
        // static double kI = 0.0;       // Volts/(RPM*second)
        
        // Experimentally derived values 
        static double kP = 0.003;    // Volts/RPM
        static double kD = 0.0;   // Volts/(RPM/second)
        static double kI = 0.0;      // Volts/(RPM*second)
    }

    public FlywheelFeedforwardPIDController(Flywheel flywheel) {
        this(flywheel, false);
    }

    public FlywheelFeedforwardPIDController(Flywheel flywheel, boolean useSlewRateLimiter) {
        super(flywheel);
        m_pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);
        SmartDashboard.putData("FFPID", m_pid);
        m_feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
        m_useSlewRateLimiter = useSlewRateLimiter;
    }

    @Override
    public void setSetpoint(double rpm) {
        if (rpm != m_setpoint) {
            super.setSetpoint(rpm);
            // discard controller's accumulated state due to previous setpoint
            m_pid.reset();
            m_pid.setSetpoint(rpm);
        }
    }

    public double calculate() {
        m_pid.setSetpoint(m_setpoint);
        if (!m_useSlewRateLimiter) {
            m_lastFeedback += m_pid.calculate(m_flywheel.getRPM());
            SmartDashboard.putNumber("Feedback", m_lastFeedback);
            return m_feedforward.calculate(m_setpoint)+m_lastFeedback; // just uses kV and kS
        } else {
            var lastSetpoint = m_slewRateLimiter.lastValue();
            var newSetpoint = m_slewRateLimiter.calculate(m_setpoint);
            m_pid.setSetpoint(lastSetpoint);
            SmartDashboard.putNumber("Flywheel/slew-rate limited setpoint", newSetpoint);
            // return m_feedforward.calculate(lastSetpoint, newSetpoint, 0.02); // pure feedforward
            m_lastFeedback += m_pid.calculate(m_flywheel.getRPM());
            SmartDashboard.putNumber("Feedback", m_lastFeedback);
            return m_feedforward.calculate(m_flywheel.getRPM(), newSetpoint, 0.02)+m_lastFeedback; // feedback actual RPM
            // return m_feedforward.calculate(lastSetpoint, newSetpoint, 0.02)+m_lastFeedback;
        }
    }
    
}
