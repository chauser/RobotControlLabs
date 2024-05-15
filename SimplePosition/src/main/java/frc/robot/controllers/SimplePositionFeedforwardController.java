package frc.robot.controllers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SimplePositionMechanism;

// The problem with this controller is that if the FF does not accurately
// track the trapezoidal plan, it will end up at the wrong place.
// Note: position error is the integral of the velocity error
public class SimplePositionFeedforwardController extends SimplePositionController{
    SimpleMotorFeedforward m_feedforward;
    TrapezoidProfile m_trapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(FeedforwardConstants.maxVel, FeedforwardConstants.maxAccel));
    TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0.0, 0.0);
    TrapezoidProfile.State m_nextState = new TrapezoidProfile.State(0.0, 0.0);

    static class FeedforwardConstants {
        // These constants determined by running SydId on this mechanism
        // static double kS = 0.012;   // Volts
        // static double kV = 4.23; // Volts/(M/s)
        // static double kA = 0.30;  // Volts/(M/s^2)

        // What if we get them a bit wrong -- these are 10% more than the correct values
        // static double kS = 0.013; // Volts
        // static double kV = 4.65;  // Volts/(M/s)
        // static double kA = 0.33;  // Volts/(M/s^2)

        // ... or a lot wrong -- these are 50% of the correct values
        static double kS = 0.006; // Volts
        static double kV = 2.12;  // Volts/(M/s)
        static double kA = 0.15;  // Volts/(M/s^2)

        static double maxVel = 2.5; // Meters/second
        static double maxAccel = 8; // Meters/second^2
    }

    @Override
    public void setSetpoint(double position, double tolerance) {
        m_setpoint = position;
        m_goal = new TrapezoidProfile.State(position, 0.0);
        m_nextState = new TrapezoidProfile.State(m_mechanism.getDistance(), m_mechanism.getLeftVelocity());
        m_tolerance = tolerance;
    }

    public SimplePositionFeedforwardController(SimplePositionMechanism mechanism) {
        super(mechanism);
        m_feedforward = new SimpleMotorFeedforward(FeedforwardConstants.kS, FeedforwardConstants.kV, FeedforwardConstants.kA);
    }

    public double calculate() {
        var currentPosition = m_mechanism.getDistance();
        if (Math.abs(currentPosition - m_goal.position)<m_tolerance) {
            SmartDashboard.putNumber("SimplePosition/Profile Distance", m_nextState.position);
            SmartDashboard.putNumber("SimplePosition/Profile Velocity", 0.0);
            return 0.0; }
        var currentState = new TrapezoidProfile.State(currentPosition, m_mechanism.getLeftVelocity());
        // no replanning
        m_nextState = m_trapezoidProfile.calculate(0.02, m_nextState, m_goal);
        // replanning - problem with replanning is that it will get "stuck" when 
        // it falls behind and the acceleration limit does not allow a too-small
        // feedforward to move beyond that point
        // m_nextState = m_trapezoidProfile.calculate(0.02, currentState, m_goal);
        SmartDashboard.putNumber("SimplePosition/Profile Distance", m_nextState.position);
        SmartDashboard.putNumber("SimplePosition/Profile Velocity", m_nextState.velocity);
        return m_feedforward.calculate(currentState.velocity, m_nextState.velocity, 0.02);
    }
    
}
