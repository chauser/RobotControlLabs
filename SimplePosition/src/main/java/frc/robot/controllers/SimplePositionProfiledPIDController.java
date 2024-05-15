package frc.robot.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SimplePositionMechanism;

public class SimplePositionProfiledPIDController extends SimplePositionController{
    SimpleMotorFeedforward m_feedforward;
    ProfiledPIDController m_ppid;

    

    static class FeedforwardConstants {
        // These constants determined by running SydId on this mechanism
        // static double kS = 0.012;   // Volts
        // static double kV = 4.23; // Volts/(M/s)
        // static double kA = 0.30;  // Volts/(M/s^2)

        // What if we get them a bit wrong -- these are 10% more than the correct values
        static double kS = 0.013; // Volts
        static double kV = 4.65;  // Volts/(M/s)
        static double kA = 0.33;  // Volts/(M/s^2)

        // ... or a lot wrong -- these are 50% of the correct values
        // static double kS = 0.006; // Volts
        // static double kV = 2.12;  // Volts/(M/s)
        // static double kA = 0.15;  // Volts/(M/s^2)

        static double maxVel = 2.5; // Meters/second
        static double maxAccel = 8; // Meters/second^2
    }

    static class PIDConstants {
        
        // Experimentally derived values
        // kP doesn't have much physical meaning here

        static double kP = 10.0;    // Volts/M
        static double kD = 0.0;     // Volts/(M/s)
        static double kI = 0.0;     // Volts/(M*s)
    }

    public SimplePositionProfiledPIDController(SimplePositionMechanism flywheel) {
        super(flywheel);
        var constraints = new TrapezoidProfile.Constraints(FeedforwardConstants.maxVel, FeedforwardConstants.maxAccel);
        m_ppid = new ProfiledPIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD, constraints);
        SmartDashboard.putData("PPID", m_ppid);
        m_feedforward = new SimpleMotorFeedforward(FeedforwardConstants.kS, FeedforwardConstants.kV, FeedforwardConstants.kA);
    }

    public double calculate() {
        var output = m_ppid.calculate(m_mechanism.getDistance(), m_setpoint);
        var ff = m_feedforward.calculate(m_mechanism.getLeftVelocity(), m_ppid.getSetpoint().velocity, 0.02);
        SmartDashboard.putNumber("SimplePosition/Profile Distance", m_ppid.getSetpoint().position);
        SmartDashboard.putNumber("SimplePosition/Profile Velocity", m_ppid.getSetpoint().velocity);
        SmartDashboard.putNumber("Feedback", output);
        return output+ff; 
    }
    
}
