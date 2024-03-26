package frc.robot.controllers;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Flywheel;

// tl;dr - PID-only control of the flywheel doesn't work well regardless of constants
public class FlywheelPIDController extends FlywheelController{
    PIDController m_pid;
    static class PIDConstants {
        // These constants determined by running SydId on this simulated flywheel
        // and dividing by 60 to convert SysID's RPS calculation to RPM
        // This achieves steady state of 540 RPM on a setting of 1000RPM 
        // static double kP = 0.00744;   // Volts/RPM
        // static double kD = 0.0;       // Volts/(RPM/second)
        // static double kI = 0.0;       // Volts/(RPM*second)
        // Experimentally derived values 
        // kP = 0.12 achieves only 650 RPM
        // kP = 0.1 achieves 940 RPM with small overshoot of steady state value
        // kP = 0.2 achieves 975 RPM with oscillations at both the 0 and 1000 RPM setpoints
        static double kP = 0.1;     // Volts/RPM
        static double kD = 0.0;       // Volts/(RPM/second)
        static double kI = 0.0;       // Volts/(RPM*second)

    }

    public FlywheelPIDController(Flywheel flywheel) {
        super(flywheel);
        m_pid = new PIDController(PIDConstants.kP, PIDConstants.kD, PIDConstants.kI);
    }

    public double calculate() {
        return m_pid.calculate(m_flywheel.getRPM()- m_setpoint);
    }
    
}
