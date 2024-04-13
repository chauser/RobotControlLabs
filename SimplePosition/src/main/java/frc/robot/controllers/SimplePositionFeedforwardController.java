package frc.robot.controllers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SimplePositionMechanism;
import frc.robot.subsystems.SimplePositionMechanism.Constants;

// The problem with this controller is that if the FF does not accurately
// track the trapezoidal plan, it will end up at the wrong place.
// Note: position error is the integral of the velocity error
public class SimplePositionFeedforwardController extends SimplePositionController{
    SimpleMotorFeedforward m_feedforward;
    TrapezoidProfile m_trapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(Constants.maxVel, Constants.maxAccel));
    TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0.0, 0.0);
    TrapezoidProfile.State m_nextState = new TrapezoidProfile.State(0.0, 0.0);

    @Override
    public void setSetpoint(double position, double tolerance) {
        m_setpoint = position;
        m_goal = new TrapezoidProfile.State(position, 0.0);
        m_nextState = new TrapezoidProfile.State(m_mechanism.getDistance(), m_mechanism.getVelocity());
        m_tolerance = tolerance;
    }

    public SimplePositionFeedforwardController(SimplePositionMechanism mechanism) {
        super(mechanism);
        m_feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
    }

    public double calculate() {
        var currentPosition = m_mechanism.getDistance();
        if (Math.abs(currentPosition - m_goal.position)<m_tolerance) {
            SmartDashboard.putNumber("SimplePosition/Profile Distance", m_nextState.position);
            SmartDashboard.putNumber("SimplePosition/Profile Velocity", 0.0);
            return 0.0; }
        var currentState = new TrapezoidProfile.State(currentPosition, m_mechanism.getVelocity());
        
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
