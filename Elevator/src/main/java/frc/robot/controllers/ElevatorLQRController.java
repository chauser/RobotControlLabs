package frc.robot.controllers;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

public class ElevatorLQRController extends ElevatorController {

    static class Constants {
        // Empirical
        // public static final double kElevatorKp = 1.0;
        // public static final double kElevatorKi = 0.0;
        // public static final double kElevatorKd = 0.0;


        // Constants from sysId
        // We need these because they are not modelled in the LinearSystem
        public static final double kElevatorkS = 0.024; // volts (V)
        public static final double kElevatorkG = 0.458; // volts (V)


        // public static final double kElevatorkV = 1.19; // volt per velocity (V/(m/s))
        // public static final double kElevatorkA = 0.086; // volt per acceleration (V/(m/s²))
        
        // Constants from WPILib example
        // public static final double kElevatorkS = 0.0; // volts (V)
        // public static final double kElevatorkG = 0.762; // volts (V)
        // public static final double kElevatorkV = 1.19; // volt per velocity (V/(m/s))
        // public static final double kElevatorkA = 0.086; // volt per acceleration (V/(m/s²))
    }

    // Standard classes for controlling our elevator

    private final TrapezoidProfile m_profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.feetToMeters(3.0),
                Units.feetToMeters(6.0))); // Max elevator speed and acceleration.
    private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

    /* The plant holds a state-space model of our elevator. This system has the following properties:

    States: [position, velocity], in meters and meters per second.
    Inputs (what we can "put in"): [voltage], in volts.
    Outputs (what we can measure): [position], in meters.

    This elevator is driven by two NEO motors.
    */
    private final LinearSystem<N2, N1, N2> m_elevatorPlant =
        LinearSystemId.createElevatorSystem(
            DCMotor.getKrakenX60(1), 
            Elevator.Constants.kCarriageMass, 
            Elevator.Constants.kElevatorDrumRadius, 
            Elevator.Constants.kElevatorGearing);

    // The observer fuses our encoder data and voltage inputs to reject noise.
    @SuppressWarnings("unchecked")
    private final KalmanFilter<N2, N1, N1> m_observer =
        new KalmanFilter<N2, N1, N1>(
            Nat.N2(),
            Nat.N1(),
            (LinearSystem<N2, N1, N1>) m_elevatorPlant.slice(0),
            VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(40)), // How accurate we
            // think our model is, in meters and meters/second.
            VecBuilder.fill(0.001), // How accurate we think our encoder position and velocity
            // data is. In this case we very highly trust our encoder position reading.
            0.020);

    // A LQR uses feedback to create voltage commands.
    @SuppressWarnings("unchecked")
    private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
        new LinearQuadraticRegulator<N2, N1, N1>(
            (LinearSystem<N2, N1, N1>) m_elevatorPlant.slice(0),
            VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)), // qelms. Position
            // and velocity error tolerances, in meters and meters per second. Decrease this to more
            // heavily penalize state excursion, or make the controller behave more aggressively. In
            // this example we weight position much more highly than velocity, but this can be
            // tuned to balance the two.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    @SuppressWarnings("unchecked")
    private final LinearSystemLoop<N2, N1, N1> m_loop =
        new LinearSystemLoop<N2, N1, N1>(
                               (LinearSystem<N2, N1, N1>) m_elevatorPlant.slice(0), 
                               m_controller, 
                               m_observer, 
                               12.0, 
                               0.020);


    public ElevatorLQRController(Elevator elevator) {
        super(elevator);
    }
      
    @Override
    public void setSetpoint(double height) {
        if (height != m_setpoint || m_goal == null) {
            super.setSetpoint(height);
            m_goal = new TrapezoidProfile.State(height, 0.0);
        }
    }

    TrapezoidProfile.State m_goal;

    public double calculate() {
        // Sets the target position of our arm. This is similar to setting the setpoint of a
        // PID controller.

        // Step our TrapezoidalProfile forward 20ms and set it as our next reference
        m_lastProfiledReference = m_profile.calculate(0.020, m_lastProfiledReference, m_goal);
        SmartDashboard.putNumber("Elevator/Profiled Position", m_lastProfiledReference.position);
        SmartDashboard.putNumber("Elevator/Profiled Velocity", m_lastProfiledReference.velocity);

        m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);
    
        // Correct our Kalman filter's state vector estimate with encoder data.
        m_loop.correct(VecBuilder.fill(m_elevator.getPosition()));
    
        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state with out Kalman filter.
        m_loop.predict(0.020);
        SmartDashboard.putNumber("Elevator/SE Position", m_loop.getXHat(0));
        SmartDashboard.putNumber("Elevator/SE Velocity", m_loop.getXHat(1));
    
        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = m_loop.getU(0); // + Constants.kElevatorkG;
        // add static offset and gravity offset
        return nextVoltage + Math.copySign(Constants.kElevatorkS, nextVoltage) + Constants.kElevatorkG;
      }  
}
