package frc.robot.controllers;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Flywheel.Constants;

public class FlywheelLQRController extends FlywheelController {

    // Standard classes for controlling our elevator

    boolean m_useSlewRateLimiter;
    SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter(Constants.kSlewRateLimit);

    /* The plant holds a state-space model of our elevator. This system has the following properties:

    States: [position, velocity], in meters and meters per second.
    Inputs (what we can "put in"): [voltage], in volts.
    Outputs (what we can measure): [position], in meters.

    This elevator is driven by two NEO motors.
    */
    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.createFlywheelSystem(
            DCMotor.getVex775Pro(1), 
            Constants.kMomentOfInertia,
            Constants.kGearing);

    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_flywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is, in RPM?
            VecBuilder.fill(0.01), // How accurate we think our encoder velocity is, in RPM?
            0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
        new LinearQuadraticRegulator<>(
            m_flywheelPlant,
            VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease this to more
            // heavily penalize state excursion, or make the controller behave more aggressively. In
            // this example we weight position much more highly than velocity, but this can be
            // tuned to balance the two.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be lower if using notifiers.

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N1, N1, N1> m_loop =
        new LinearSystemLoop<>(m_flywheelPlant, 
                               m_controller, 
                               m_observer, 
                               12.0, 
                               0.020);

    public FlywheelLQRController(Flywheel flywheel) {
        this(flywheel, false);
    }

    public FlywheelLQRController(Flywheel flywheel, boolean useSlewRateLimiter) {
        super(flywheel);
        m_useSlewRateLimiter = useSlewRateLimiter;
    }
      
    @Override
    public void setSetpoint(double rpm) {
        var rps = 2*Math.PI*rpm/60.0;
        if (rps != m_setpoint) {
            super.setSetpoint(rps);
        }
    }

    public double calculate() {
        // Achieve the setpoint velocity

        if (m_useSlewRateLimiter) {
            var thisVelocity = m_slewRateLimiter.calculate(m_setpoint*60/(Math.PI*2));
            SmartDashboard.putNumber("Flywheel/slew-rate limited setpoint", thisVelocity);
            m_loop.setNextR(thisVelocity*Math.PI*2/60.0);
        } else {
            SmartDashboard.putNumber("Flywheel/slew-rate limited setpoint", m_setpoint*60/(Math.PI*2));
            m_loop.setNextR(m_setpoint);
        }
    
        // Correct our Kalman filter's state vector estimate with encoder data.
        m_loop.correct(VecBuilder.fill(m_flywheel.getRPM()*Math.PI*2.0/60.0));
    
        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state with out Kalman filter.
        m_loop.predict(0.020);
        SmartDashboard.putNumber("Flywheel/SE Velocity", m_loop.getXHat(0));
    
        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = m_loop.getU(0); // + Constants.kElevatorkG;
        // add static offset and gravity offset
        return nextVoltage;
      }  
}
