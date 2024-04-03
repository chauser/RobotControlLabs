// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.controllers.SimplePositionController;
import frc.robot.controllers.SimplePositionProfiledPIDController;
import frc.robot.controllers.SimplePositionFeedforwardController;
import frc.robot.controllers.SimplePositionPIDController;
import frc.robot.subsystems.SimplePositionMechanism;

/** This is a program to introduce various controllers for flywheels and 
 *  allow exploration of their behaviors
 */
public class Robot extends TimedRobot {
  private final CommandXboxController m_joystick = new CommandXboxController(Constants.kControllerPort);
  private final SimplePositionMechanism m_mechanism = new SimplePositionMechanism();

  private final SendableChooser<SimplePositionController> m_controllerChooser = new SendableChooser<>();
  private final SimplePositionController m_feedforwardController = new SimplePositionFeedforwardController(m_mechanism);
  private final SimplePositionController m_PIDController = new SimplePositionPIDController(m_mechanism);
  private final SimplePositionController m_PPID = new SimplePositionProfiledPIDController(m_mechanism);
  private SimplePositionController m_currentController = m_feedforwardController;
 
  @Override
  public void robotInit() {
    SmartDashboard.putData("Controller type", m_controllerChooser);
    m_controllerChooser.setDefaultOption("Feedforward", m_feedforwardController);
    m_controllerChooser.addOption("PID", m_PIDController);
    // m_controllerChooser.addOption("FeedForward with SlewRateLimiter", m_FFwithSlewRateController);
    m_controllerChooser.addOption("Profiled PID", m_PPID);
    m_mechanism.setDefaultCommand(m_mechanism.run(m_mechanism::stop));
    m_joystick.a()
      .whileTrue(
        m_mechanism.runOnce(() -> {
          // Find out what controller should be used and update it
          m_currentController = m_controllerChooser.getSelected();
          m_currentController.setSetpoint(Constants.kSetpointDistance, Constants.kSetpointTolerance);
        })  
        .andThen(m_mechanism.run(() -> {
          var volts = m_currentController.calculate();
          m_mechanism.setVoltage(volts);
        })));

    m_joystick.povDown()
      .whileTrue(
        m_mechanism.runOnce(() -> {
          // Find out what controller should be used and update it
          m_currentController = m_controllerChooser.getSelected();
          m_currentController.setSetpoint(0.0, Constants.kSetpointTolerance);
        })  
        .andThen(m_mechanism.run(() -> {
          var volts = m_currentController.calculate();
          m_mechanism.setVoltage(volts);
        })));
    m_joystick.b().whileTrue(m_mechanism.sysIdQuasistaticCommand(Direction.kForward));
    m_joystick.y().whileTrue(m_mechanism.sysIdDynamicCommand(Direction.kForward));
    m_joystick.povRight().whileTrue(m_mechanism.sysIdQuasistaticCommand(Direction.kReverse));
    m_joystick.povUp().whileTrue(m_mechanism.sysIdDynamicCommand(Direction.kReverse));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_mechanism.stop();
  }

  @Override
  public void close() {
    m_mechanism.close();
    super.close();
  }
}
