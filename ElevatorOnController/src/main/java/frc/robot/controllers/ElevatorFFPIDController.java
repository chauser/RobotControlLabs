package frc.robot.controllers;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Constants;

public class ElevatorFFPIDController extends ElevatorController {

    static class PIDConstants {
        // Empirical
        // public static final double kElevatorKp = 1.0;
        // public static final double kElevatorKi = 0.0;
        // public static final double kElevatorKd = 0.0;

        // From WPILib example
        public static final double kElevatorKp = 5.0;
        public static final double kElevatorKi = 0.0;
        public static final double kElevatorKd = 0.0;      
    }

    // Standard classes for controlling our elevator
  private final ProfiledPIDController m_ppid =
      new ProfiledPIDController(
          PIDConstants.kElevatorKp,
          PIDConstants.kElevatorKi,
          PIDConstants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  
    ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);

    public ElevatorFFPIDController(Elevator Elevator) {
        super(Elevator);
        SmartDashboard.putData("FFPPID", m_ppid);
    }
      
    @Override
    public void setSetpoint(double height) {
        if (height != m_setpoint) {
            super.setSetpoint(height);
            // discard controller's accumulated state due to previous setpoint
            m_ppid.reset(m_elevator.getPosition());
            m_ppid.setGoal(height);
        }
    }

    @Override
    public double calculate() {
        // With the setpoint value we run PID control like normal
        double pidOutput = m_ppid.calculate(m_elevator.getPosition());
        double feedforwardOutput = m_feedforward.calculate(m_ppid.getSetpoint().velocity);
        SmartDashboard.putNumber("Elevator/Feedback", pidOutput);
        return (pidOutput + feedforwardOutput);
    }  
}
