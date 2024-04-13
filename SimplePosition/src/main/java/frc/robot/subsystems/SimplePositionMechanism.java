// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
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

public class SimplePositionMechanism extends SubsystemBase implements AutoCloseable {
  public static class Constants {
    static int kEncoderAChannel = 0;
    static int kEncoderBChannel = 1;
    static int kMotorPort = 0;
    static double kGearing = 10.0;
    static double kWheelRadius = 0.05; //Meter
    static double kMassKg = 50;
    static double kTrackWidthMeters = 3.0/4.0;
    static double kMOI = kMassKg * Math.pow(Constants.kTrackWidthMeters/4.0, 2);
    static double kEncoderDistPerPulse = Math.PI*2*kWheelRadius/kGearing; // 1 pulse per motor revolution
    public static final int kControllerPort = 0;
    public static final double kSetpointDistance = 3.0;
    public static final double kSetpointTolerance = 0.025;
    // These constants determined by running SydId on this mechanism
    // static double kS = 0.012;   // Volts
    // static double kV = 4.23; // Volts/(M/s)
    // static double kA = 0.30;  // Volts/(M/s^2)

    // What if we get them a bit wrong -- these are 10% more than the correct values
    // static double kS = 0.013; // Volts
    // static double kV = 4.65;  // Volts/(M/s)
    // static double kA = 0.33;  // Volts/(M/s^2)

    // ... or a lot wrong -- these are 50% of the correct values
    static double kS = 0.006; // Volts
    static double kV = 2.12;  // Volts/(M/s)
    static double kA = 0.15;  // Volts/(M/s^2)
    
    static double maxVel = 2.5; // Meters/second
    static double maxAccel = 8; // Meters/second^2
  }

  // This gearbox represents a gearbox containing 2 Vex 775pro motors.
  private final DCMotor m_gearbox = DCMotor.getNEO(2);

  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private final DifferentialDrivetrainSim m_drivetrainSim =
      new DifferentialDrivetrainSim(
        m_gearbox,
        Constants.kGearing,
        Constants.kMOI,
        Constants.kMassKg,
        Constants.kWheelRadius,
        Constants.kTrackWidthMeters,
        null);

  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // Support for SysId
  private final SysIdRoutine m_sysid = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> v) -> setVoltage(v.in(Volts)),
      this::logData, 
      this, 
      "SimplePosition Lab")
  );
  private final MutableMeasure<Voltage> m_appliedVoltage = Volts.of(0).mutableCopy();
  private final MutableMeasure<Velocity<Distance>> m_velocity = MetersPerSecond.of(0).mutableCopy();
  private final MutableMeasure<Distance> m_position = Meters.of(0).mutableCopy();

  /** Subsystem constructor. */
  public SimplePositionMechanism() {
    m_encoder.setDistancePerPulse(Constants.kEncoderDistPerPulse);
  }

  double m_prevTime = Timer.getFPGATimestamp();
  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our flywheel is doing
    // First, we set our "inputs" (voltages)
    var speed = m_motorSim.getSpeed() * RobotController.getBatteryVoltage();
    m_drivetrainSim.setInputs(speed, speed);

    // Next, we update it. The standard loop time is 20ms.
    m_drivetrainSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_encoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());

    // BatterySim estimates loaded battery voltages
    var currentDraw = m_drivetrainSim.getCurrentDrawAmps();
    var batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(currentDraw);
    RoboRioSim.setVInVoltage(batteryVoltage);
    SmartDashboard.putNumber("Battery voltage", batteryVoltage);
    SmartDashboard.putNumber("Current", currentDraw);
    }

  @Override
  public void periodic () {
    updateTelemetry();
  }

  /** Set both motor output voltages */
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    setVoltage(0.0);
  }

  public double getVelocity() {
    return m_encoder.getRate();
  }

  public double getDistance() {
    return m_encoder.getDistance();
  }

/** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update flywheel info on dashboard
    SmartDashboard.putNumber("SimplePosition/Velocity", getVelocity());
    SmartDashboard.putNumber("SimplePosition/p.u.", m_motor.get());
    SmartDashboard.putNumber("SimplePosition/Position", getDistance());
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
    log.motor("position")
      .voltage(
        m_appliedVoltage.mut_replace(
          m_motor.get() * RobotController.getBatteryVoltage(), Volts))
      .linearVelocity(
        m_velocity.mut_replace(m_encoder.getRate(), MetersPerSecond)
      )
      .linearPosition(
        m_position.mut_replace(m_encoder.getDistance(), Meters)
      );   
  }
}
