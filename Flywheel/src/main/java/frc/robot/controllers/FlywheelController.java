package frc.robot.controllers;

import frc.robot.subsystems.Flywheel;

public abstract class FlywheelController {
    double m_setpoint = 0.0;
    Flywheel m_flywheel;

    public FlywheelController(Flywheel flywheel) {
        m_flywheel = flywheel;
    }

    public void setSetpoint(double rpm) {
        m_setpoint = rpm;
    }

    public abstract double calculate();

}
