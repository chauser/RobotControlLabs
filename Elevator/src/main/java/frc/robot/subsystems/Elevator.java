// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Elevator extends SubsystemBase implements AutoCloseable {

  public class Constants {
    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;

    // Empirical
    // public static final double kElevatorKp = 1.0;
    // public static final double kElevatorKi = 0.0;
    // public static final double kElevatorKd = 0.0;

    // From WPILib example
    public static final double kElevatorKp = 5.0;
    public static final double kElevatorKi = 0.0;
    public static final double kElevatorKd = 0.0;

    // Constants from sysId
    // public static final double kElevatorkS = 0.019; // volts (V)
    // public static final double kElevatorkG = 0.837; // volts (V)
    // public static final double kElevatorkV = 1.19; // volt per velocity (V/(m/s))
    // public static final double kElevatorkA = 0.086; // volt per acceleration (V/(m/s²))
    
    // Constants from WPILib example
    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.762; // volts (V)
    public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorGearing = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg

    public static final double kSetpointMeters = 0.75;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 1.25;

    // distance per pulse = (distance per revolution) / (pulses per revolution)
    //  = (Pi * D) / ppr
    public static final double kElevatorEncoderDistPerPulse =
        2.0 * Math.PI * kElevatorDrumRadius / 4096;
  }

  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          Constants.kElevatorKp,
          Constants.kElevatorKi,
          Constants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);
  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          Constants.kElevatorGearing,
          Constants.kCarriageMass,
          Constants.kElevatorDrumRadius,
          Constants.kMinElevatorHeightMeters,
          Constants.kMaxElevatorHeightMeters,
          true,
          0,
          VecBuilder.fill(0.01));
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(2, 2);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 1, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));


   // Support for SysId
  private final SysIdRoutine m_sysid = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> v) -> m_motor.setVoltage(v.in(Volts)),
      this::logData, 
      this, 
      "Elevator Lab")
  );
  private final MutableMeasure<Voltage> m_appliedVoltage = Volts.of(0).mutableCopy();
  private final MutableMeasure<Velocity<Distance>> m_velocity = MetersPerSecond.of(0).mutableCopy();
  private final MutableMeasure<Distance> m_position = Meters.of(0).mutableCopy();

  /** Subsystem constructor. */
  public Elevator() {
    m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
    SmartDashboard.putData("Elevator/PID", m_controller);
  }

  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    m_encoderSim.setRate(m_elevatorSim.getVelocityMetersPerSecond());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */

  public void initGoal(double goal) {
    m_controller.setGoal(goal);
  }

  public void reachGoal() {
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(getPosition());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    SmartDashboard.putNumber("Elevator/Feedback", pidOutput);
    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }

  public double getPosition() {
    return m_encoder.getDistance();
  }

  public double getVelocity() {
    return m_encoder.getRate();
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_controller.setGoal(0.0);
    m_motor.setVoltage(Constants.kElevatorkG);
  }

  /** Update telemetry, including the mechanism visualization. */
  private void updateTelemetry() {
    SmartDashboard.putNumber("Elevator/Position", getPosition());
    SmartDashboard.putNumber("Elevator/Velocity", getVelocity());
    var setPoint = m_controller.getSetpoint();
    SmartDashboard.putNumber("Elevator/Profiled Position", setPoint.position);
    SmartDashboard.putNumber("Elevator/Profiled Velocity", setPoint.velocity);
    SmartDashboard.putNumber("Elevator/p.u.", m_motor.get());
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getDistance());
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
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
