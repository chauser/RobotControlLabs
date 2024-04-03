package frc.robot.controllers;

import frc.robot.subsystems.SimplePositionMechanism;

public class SimplePositionBangBangController extends SimplePositionController{
    static class BangBangConstants {
        static double onVoltage = 10.0;
    }

    double m_tolerance;

    public SimplePositionBangBangController(SimplePositionMechanism mechanism) {
        super(mechanism);
    }

    public double calculate() {
        return m_mechanism.getLeftDistance() < (m_setpoint - m_tolerance) 
                ? BangBangConstants.onVoltage 
                : (m_mechanism.getLeftDistance() > (m_setpoint + m_tolerance)
                    ? -BangBangConstants.onVoltage
                    : 0.0
                );
    }
    
}
