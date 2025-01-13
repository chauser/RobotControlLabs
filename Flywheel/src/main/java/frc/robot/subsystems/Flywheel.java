// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/*
 * This class implements the Flywheel simulation which is controlled
 * with various controllers in the controllers/ folder.
 */

public class Flywheel extends SubsystemBase implements AutoCloseable {
  public static class Constants {
    static int kEncoderAChannel = 0;
    static int kEncoderBChannel = 1;
    static int kMotorPort = 0;
    public static double kGearing = 10.0;
    public static double kMomentOfInertia = 0.01; //Kg*M^2 -- 1kg at .1M radius e.g.
    static double kEncoderDistPerPulse = 1.0; // 1 pulse per revolution
    public static double kSlewRateLimit = 2000; // RPM/second
    // These constants determined by running SydId on this simulated flywheel
    public static double kS = 0.011;   // Volts
    public static double kV = 0.00637; // Volts/RPM
    public static double kA = 0.0017;  // (Volts/RPM)/second 
  }
  // This gearbox represents a gearbox containing 1 Vex 775pro.
  private final DCMotor m_flywheelGearbox = DCMotor.getVex775Pro(1);

  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);

  // Simulation classes help us simulate what's going on
  private final FlywheelSim m_flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(m_flywheelGearbox, Constants.kMomentOfInertia, Constants.kGearing),
          m_flywheelGearbox
      );

  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // Support for SysId
  private final SysIdRoutine m_sysid = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      (Voltage v) -> setVoltage(v.in(Volts)),
      this::logData, 
      this, 
      "Flywheel Lab")
  );
  private final MutVoltage m_appliedVoltage = Volts.of(0).mutableCopy();
  private final MutAngularVelocity m_velocity = RotationsPerSecond.of(0).mutableCopy();
  private final MutAngle m_position = Rotations.of(0).mutableCopy();

  /** Subsystem constructor. */
  public Flywheel() {
    m_encoder.setDistancePerPulse(Constants.kEncoderDistPerPulse);
  }

  double m_prevTime = Timer.getFPGATimestamp();
  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our flywheel is doing
    // First, we set our "inputs" (voltages)
    m_flywheelSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_flywheelSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setRate(m_flywheelSim.getAngularVelocityRPM());
    var now = Timer.getFPGATimestamp();
    m_encoderSim.setDistance(m_encoderSim.getDistance()+(now-m_prevTime)*m_flywheelSim.getAngularVelocityRPM()/60.0);
    m_prevTime = now;
    // BatterySim estimates loaded battery voltages
    var currentDraw = m_flywheelSim.getCurrentDrawAmps();
    var batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(currentDraw);
    RoboRioSim.setVInVoltage(batteryVoltage);
    SmartDashboard.putNumber("Battery voltage", batteryVoltage);
    SmartDashboard.putNumber("Current", currentDraw);
    }

  @Override
  public void periodic () {
    updateTelemetry();
  }

  /** Set the motor output voltage */
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_motor.set(0.0);
  }

  public double getRPM() {
    return m_encoder.getRate();
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update flywheel info on dashboard
    SmartDashboard.putNumber("Flywheel/RPM", m_encoder.getRate());
    SmartDashboard.putNumber("Flywheel/p.u.", m_motor.get());
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
  }

  public Command sysIdQuasistaticCommand(Direction dir) {
    return m_sysid.quasistatic(dir);
  }
  
  public Command sysIdDynamicCommand(Direction dir) {
    return m_sysid.dynamic(dir);
  }
  
  private void logData(SysIdRoutineLog log) {
    log.motor("flywheel")
      .voltage(
        m_appliedVoltage.mut_replace(
          m_motor.get() * RobotController.getBatteryVoltage(), Volts))
      .angularVelocity(
        m_velocity.mut_replace(m_encoder.getRate()/60, RotationsPerSecond)
      )
      .angularPosition(
        m_position.mut_replace(m_encoder.getDistance(), Rotations)
      );   
  }
}
