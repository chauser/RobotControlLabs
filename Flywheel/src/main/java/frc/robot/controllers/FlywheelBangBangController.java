package frc.robot.controllers;

import frc.robot.subsystems.Flywheel;

public class FlywheelBangBangController extends FlywheelController{
    static class BangBangConstants {
        static double onVoltage = 10.0;
    }

    public FlywheelBangBangController(Flywheel flywheel) {
        super(flywheel);
    }

    public double calculate() {
        return m_flywheel.getRPM() < m_setpoint ? BangBangConstants.onVoltage : 0.0;
    }
    
}
