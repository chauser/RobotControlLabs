package frc.robot.controllers;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Flywheel.Constants;

public class FlywheelBangBangController extends FlywheelController{
    private final BangBangController m_bangBangController = new BangBangController();
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

    public FlywheelBangBangController(Flywheel flywheel) {
        super(flywheel);
    }

    public double calculate() {
        double bangbangOutput = m_bangBangController.calculate(m_flywheel.getRPM(), m_setpoint)*12;
        return bangbangOutput + 0.9 * m_feedforward.calculate(m_setpoint);
    }  
}
