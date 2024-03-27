package frc.robot.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Flywheel;

// tl;dr - PID-only control of the flywheel doesn't work well regardless of constants
public class FlywheelPIDController extends FlywheelController{
    PIDController m_pid;
    double m_prevOutput;
    static class PIDConstants {
        // These constants determined by running SydId on this simulated flywheel
        // and dividing by 60 to convert SysID's RPS calculation to RPM
        // This achieves steady state of 540 RPM on a setting of 1000RPM 
        // static double kP = 0.00744;   // Volts/RPM
        // static double kD = 0.0;       // Volts/(RPM/second)
        // static double kI = 0.0;       // Volts/(RPM*second)
        
        // Experimentally derived values 
        static double kP = 0.002;    // Volts/RPM
        static double kD = 0.0003;   // Volts/(RPM/second)
        static double kI = 0.0;      // Volts/(RPM*second)
    }

    public FlywheelPIDController(Flywheel flywheel) {
        super(flywheel);
        m_pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);
        // The integrator range is very important; default is -1..1
        m_pid.setIntegratorRange(-50, 50);
        m_pid.setIZone(50.0);
        SmartDashboard.putData("FlywheelPIDController", m_pid);
    }

    @Override
    public void setSetpoint(double rpm) {
        if (rpm != m_setpoint) {
            super.setSetpoint(rpm);
            // discard controller's accumulated state due to previous setpoint
            m_pid.reset();
            m_pid.setSetpoint(rpm);
            m_prevOutput = 0;
        }
    }

    // The trick to getting PID to work for velocity is to have the
    // PID compute the difference between the previous output and the new output
    // NOT the entire new output.
    @Override
    public double calculate() {
        m_prevOutput += m_pid.calculate(m_flywheel.getRPM());
        return m_prevOutput;
    }
    
}
