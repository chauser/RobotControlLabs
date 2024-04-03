package frc.robot.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SimplePositionMechanism;

public class SimplePositionPIDController extends SimplePositionController{
    PIDController m_pid;
    static class PIDConstants {
        
        // Experimentally derived values 
        static double kP = 10;    // Volts/M
        static double kD = 0;     // Volts/(M/s)
        static double kI = 0.0;   // Volts/(M*s)
    }

    public SimplePositionPIDController(SimplePositionMechanism mechanism) {
        super(mechanism);
        m_pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);
        // The integrator range is very important; default is -1..1
        m_pid.setIntegratorRange(-50, 50);
        m_pid.setIZone(50.0);
        SmartDashboard.putData("SimplePositionPIDController", m_pid);
    }

    @Override
    public void setSetpoint(double position, double tolerance) {
        m_setpoint = position;
        m_tolerance = tolerance;
        m_pid.setSetpoint(m_setpoint);
        m_pid.setTolerance(tolerance);
    }
    
    // The trick to getting PID to work for velocity is to have the
    // PID compute the difference between the previous output and the new output
    // NOT the entire new output.
    @Override
    public double calculate() {

        var output = m_pid.calculate(m_mechanism.getLeftDistance());
        output = m_pid.atSetpoint() ? 0.0 : output;
        SmartDashboard.putNumber("Feedback", output);
        return output;
    }
    
}
