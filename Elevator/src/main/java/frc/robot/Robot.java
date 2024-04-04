// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.controllers.ElevatorController;
import frc.robot.controllers.ElevatorFFPIDController;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {
  static class Constants {
    static final double kSetpointMeters = 0.75;
    static final int kJoystickPort = 0;
  }

  private final CommandXboxController m_joystick = new CommandXboxController(Constants.kJoystickPort);
  private final Elevator m_elevator = new Elevator();
  private final SendableChooser<ElevatorController> m_controllerChooser = new SendableChooser<>();
  private final ElevatorController m_ppidController = new ElevatorFFPIDController(m_elevator);

  private void runElevator(double setPoint) {
    var controller = m_controllerChooser.getSelected();
    controller.setSetpoint(setPoint);
    m_elevator.setVoltage(controller.calculate());
  }
  
  @Override
  public void robotInit() {
    m_elevator.setDefaultCommand(m_elevator.run(m_elevator::stop));
    m_controllerChooser.setDefaultOption("Feedforward PID", m_ppidController);
    
    // For normal operation
    m_joystick.y()
      .whileTrue(m_elevator.run(() -> runElevator(Constants.kSetpointMeters)));
    m_joystick.a()
      .whileTrue(m_elevator.run(() -> runElevator(0.0)));

    // For SysId
    m_joystick.povRight().whileTrue(m_elevator.sysIdQuasistaticCommand(Direction.kForward));
    m_joystick.povLeft().whileTrue(m_elevator.sysIdQuasistaticCommand(Direction.kReverse));
    m_joystick.povUp().whileTrue(m_elevator.sysIdDynamicCommand(Direction.kForward));
    m_joystick.povDown().whileTrue(m_elevator.sysIdDynamicCommand(Direction.kReverse));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_elevator.stop();
  }

  @Override
  public void close() {
    m_elevator.close();
    super.close();
  }
}
