package frc.robot.controllers;

import frc.robot.subsystems.Elevator;

public abstract class ElevatorController {
    double m_setpoint = 0.0;
    Elevator m_elevator;

    public ElevatorController(Elevator elevator) {
        m_elevator = elevator;
    }

    public void setSetpoint(double rpm) {
        m_setpoint = rpm;
    }

    public abstract double calculate();

}

