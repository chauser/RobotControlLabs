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
import frc.robot.controllers.FlywheelBangBangController;
import frc.robot.controllers.FlywheelController;
import frc.robot.controllers.FlywheelFeedforwardPIDController;
import frc.robot.controllers.FlywheelFeedforwardController;
import frc.robot.controllers.FlywheelPIDController;
import frc.robot.subsystems.Flywheel;

/** This is a program to introduce various controllers for flywheels and 
 *  allow exploration of their behaviors
 */
public class Robot extends TimedRobot {
  private final CommandXboxController m_joystick = new CommandXboxController(Constants.kControllerPort);
  private final Flywheel m_flywheel = new Flywheel();

  private final SendableChooser<FlywheelController> m_controllerChooser = new SendableChooser<>();
  private final FlywheelController m_bangbangController = new FlywheelBangBangController(m_flywheel);
  private final FlywheelController m_feedforwardController = new FlywheelFeedforwardController(m_flywheel);
  private final FlywheelController m_PIDController = new FlywheelPIDController(m_flywheel);
  private final FlywheelController m_FFwithSlewRateController = new FlywheelFeedforwardController(m_flywheel, true);
  private final FlywheelController m_FFPID = new FlywheelFeedforwardPIDController(m_flywheel, false);
  private final FlywheelController m_FFPIDSlew = new FlywheelFeedforwardPIDController(m_flywheel, true);
  private FlywheelController m_currentController = m_bangbangController;
 
  @Override
  public void robotInit() {
    SmartDashboard.putData("Controller type", m_controllerChooser);
    m_controllerChooser.setDefaultOption("BangBang", m_bangbangController);
    m_controllerChooser.addOption("Feedforward", m_feedforwardController);
    m_controllerChooser.addOption("PID", m_PIDController);
    m_controllerChooser.addOption("FeedForward with SlewRateLimiter", m_FFwithSlewRateController);
    m_controllerChooser.addOption("FF+PID", m_FFPID);
    m_controllerChooser.addOption("FF+PID+Slew", m_FFPIDSlew);
    m_flywheel.setDefaultCommand(m_flywheel.run(() -> {
      // Find out what controller should be used and update it
      m_currentController = m_controllerChooser.getSelected();
      if (m_joystick.getHID().getAButton()) {
        // Here, we set the constant setpoint
        // of the currentController
        m_currentController.setSetpoint(Constants.kSetpointRPM);
      } else {
        // Otherwise, we update the setpoint to 0.
        m_currentController.setSetpoint(0.0);
      }
      var volts = m_currentController.calculate();
      m_flywheel.setVoltage(volts);
      }
    ));
    m_joystick.b().whileTrue(m_flywheel.sysIdQuasistaticCommand(Direction.kForward));
    m_joystick.y().whileTrue(m_flywheel.sysIdDynamicCommand(Direction.kForward));
    m_joystick.povRight().whileTrue(m_flywheel.sysIdQuasistaticCommand(Direction.kReverse));
    m_joystick.povUp().whileTrue(m_flywheel.sysIdDynamicCommand(Direction.kReverse));
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
    m_flywheel.stop();
  }

  @Override
  public void close() {
    m_flywheel.close();
    super.close();
  }
}
