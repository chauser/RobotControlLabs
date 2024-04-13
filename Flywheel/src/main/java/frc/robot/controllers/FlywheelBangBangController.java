package frc.robot.controllers;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.Flywheel;

public class FlywheelBangBangController extends FlywheelController{
    private final BangBangController m_bangBangController = new BangBangController();
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Flywheel.Constants.kS, Flywheel.Constants.kV, Flywheel.Constants.kA);

    public FlywheelBangBangController(Flywheel flywheel) {
        super(flywheel);
    }

    public double calculate() {
        double bangbangOutput = m_bangBangController.calculate(m_flywheel.getRPM(), m_setpoint)*12;
        return bangbangOutput + 0.9 * m_feedforward.calculate(m_setpoint);
    }
    
}
