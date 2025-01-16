// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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
    public static final int kBaseMotorId = 0;
 

    public static final double kElevatorGearing = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kElevatorDrumCircumference = kElevatorDrumRadius * Math.PI * 2;
    public static final double kCarriageMass = 4.0; // kg

    public static final double kSetpointMeters = 0.75;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 1.25;

    }

    public class SysId {
      public static final double kS = 0.065403;
      public static final double kV = 3.5919;
      public static final double kA = 0.068492;
      public static final double kG = 0.16775;
      // The following using "CTRE Phoenix 6" gain presets in SysId
      public static final double kP = 38.677; // for Max Position Error = 0.05
      public static final double kD = 0.65899;
      // note kA, kP, and kD likely need to be adjusted for the on-board 
      // Motion Magic because they are calculated based on position and velocity
      // meters and meters/sec rather than position in motor rotations and velocity
      // in RPS.
      // Since kS, kV, and kG are in volts they should not require adjustment

    }
  private final TalonFX m_motor1 = new TalonFX(Constants.kBaseMotorId);
  private final TalonFXConfiguration m_motor1Config = 
    new TalonFXConfiguration()
      .withFeedback(new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(Constants.kElevatorGearing/Constants.kElevatorDrumCircumference));
  private final TalonFX m_motor2 = new TalonFX(Constants.kBaseMotorId+1);

  private final Slot0Configs controllerConfig = 
    m_motor1Config.Slot0
      .withKS(SysId.kS)
      .withKV(SysId.kV)
      .withKA(SysId.kA)
      .withKP(SysId.kP)
      .withKD(SysId.kD)
      .withKG(SysId.kG);

  private final MotionMagicConfigs mmConfig = 
    m_motor1Config.MotionMagic
      .withMotionMagicCruiseVelocity(3.0)
      .withMotionMagicAcceleration(15);

  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  
   // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
    LinearSystemId.createElevatorSystem(
          DCMotor.getKrakenX60(2), 
          Constants.kCarriageMass, 
          Constants.kElevatorDrumRadius, 
          Constants.kElevatorGearing),
          DCMotor.getKrakenX60(2),
          Constants.kMinElevatorHeightMeters,
          Constants.kMaxElevatorHeightMeters,
          true,
          0.0
    );

  private final TalonFXSimState m_motor1Sim = m_motor1.getSimState();

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
      (Voltage v) -> m_motor1.setVoltage(v.in(Volts)),
      this::logData, 
      this, 
      "Elevator Lab")
  );
  private final MutVoltage m_appliedVoltage = Volts.of(0).mutableCopy();
  private Distance m_position = Meters.of(0);

  /** Subsystem constructor. */
  public Elevator() {
    m_motor1.getConfigurator().apply(m_motor1Config);
    m_motor1.setControl(m_request.withPosition(0));
    m_motor2.getConfigurator().apply(m_motor1Config);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  public void setSetpoint(Distance position) {
    m_motor1.setControl(m_request.withPosition(position.in(Meters)));
  }

  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    m_motor1Sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motor1Sim.getMotorVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motor1Sim.setRawRotorPosition(toRotorAngle(Meters.of(m_elevatorSim.getPositionMeters())));
    m_motor1Sim.setRotorVelocity(toRotorAngularVelocity(MetersPerSecond.of(m_elevatorSim.getVelocityMetersPerSecond()))); 
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  Distance toElevatorPosition(Angle rotorAngle) {
    return Meters.of(rotorAngle.in(Degrees)/360.0 * Constants.kElevatorDrumCircumference/Constants.kElevatorGearing);
  }

  Angle toRotorAngle(Distance elevatorPosition) {
    return Rotations.of(Constants.kElevatorGearing*elevatorPosition.in(Meters)/Constants.kElevatorDrumCircumference);
  }

  LinearVelocity toElevatorVelocity(AngularVelocity rotorVelocity) {
    return MetersPerSecond.of(rotorVelocity.in(RotationsPerSecond)*Constants.kElevatorDrumCircumference/Constants.kElevatorGearing);
  }

  AngularVelocity toRotorAngularVelocity(LinearVelocity elevatorVelocity) {
    return RotationsPerSecond.of(Constants.kElevatorGearing*elevatorVelocity.in(MetersPerSecond)/Constants.kElevatorDrumCircumference);
  }

  @Override
  public void periodic() {
    m_position = Meters.of(m_motor1.getPosition().getValue().in(Rotations));
    // m_filteredPosition = m_filteredPosition*Constants.kFilterFactor 
      // + m_encoder.getDistance()*(1-Constants.kFilterFactor);
    updateTelemetry();
  }

  public Distance getPosition() {
    return m_position;
  }

  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(m_motor1.getVelocity().getValue().in(RotationsPerSecond));
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    setVoltage(0.0);
  }

  public void setVoltage(double volts) {
    m_motor1.setVoltage(volts);
  }

  /** Update telemetry, including the mechanism visualization. */
  private void updateTelemetry() {
    SmartDashboard.putNumber("Elevator/Position", getPosition().baseUnitMagnitude());
    SmartDashboard.putNumber("Elevator/Velocity", getVelocity().baseUnitMagnitude());
    SmartDashboard.putNumber("Elevator/p.u.", m_motor1.get());
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(getPosition().baseUnitMagnitude());
  }

  @Override
  public void close() {
    m_motor1.close();
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
          m_motor1.get() * RobotController.getBatteryVoltage(), Volts))
      .linearVelocity(
        getVelocity()
      )
      .linearPosition(
        // use raw data here rather than filtered position
        getPosition()
      );   
  }
}
