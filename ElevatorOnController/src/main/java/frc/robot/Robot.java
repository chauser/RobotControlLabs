// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {
  static class Constants {
    static final Distance kSetpoint = Meters.of(0.75);
    static final int kJoystickPort = 0;
  }

  private final CommandXboxController m_joystick = new CommandXboxController(Constants.kJoystickPort);
  private final Elevator m_elevator = new Elevator();
  
  private void runElevator(Distance setPoint) {
    m_elevator.setSetpoint(setPoint);
  }
  
  @Override
  public void robotInit() {
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    
    // For normal operation
    m_joystick.y()
      .onTrue(m_elevator.run(() -> runElevator(Constants.kSetpoint)).withName("High"));
    m_joystick.a()
      .onTrue(m_elevator.run(() -> runElevator(Meters.of(0.0))).withName("Low"));

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
