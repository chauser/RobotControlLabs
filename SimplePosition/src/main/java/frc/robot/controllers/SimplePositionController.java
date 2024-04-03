package frc.robot.controllers;

import frc.robot.subsystems.SimplePositionMechanism;

public abstract class SimplePositionController {
    double m_setpoint = 0.0;
    double m_tolerance = 1.0;
    SimplePositionMechanism m_mechanism;

    public SimplePositionController(SimplePositionMechanism mechanism) {
        m_mechanism = mechanism;
    }

    public void setSetpoint(double distance, double tolerance) {
        m_setpoint = distance;
        m_tolerance = tolerance;
    }

    public abstract double calculate();

}
