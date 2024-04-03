// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {
  private final CommandXboxController m_joystick = new CommandXboxController(Constants.kJoystickPort);
  private final Elevator m_elevator = new Elevator();

  @Override
  public void robotInit() {
    m_joystick.a()
      .whileTrue(m_elevator.run(
        () -> m_elevator.reachGoal(Constants.kSetpointMeters)
      ))
      .whileFalse(m_elevator.run(
        () -> m_elevator.reachGoal(0.0)
      ));
  }

  @Override
  public void robotPeriodic() {
    // Update the telemetry, including mechanism visualization, regardless of mode.
    m_elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    m_elevator.simulationPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
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