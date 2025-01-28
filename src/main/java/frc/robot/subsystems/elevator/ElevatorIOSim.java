// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorIOSim implements ElevatorIO {
  private DCMotor gearbox = DCMotor.getNeoVortex(2);
  private SparkFlex masterMotor = new SparkFlex(11, MotorType.kBrushless);
  private SparkFlexSim simFlex = new SparkFlexSim(masterMotor, gearbox);
  private SparkClosedLoopController closedLoopController = masterMotor.getClosedLoopController();

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          gearbox,
          ElevatorConstants.motorReduction,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.PD22t,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);

  private double appliedVolts = 0.0;
  private double setpoint = 0.0;
  private boolean isHome = false;
  private boolean isAtSetpoint = false;

  public ElevatorIOSim() {
    var master_config = new SparkFlexConfig();

    master_config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    /*
    master_config
            .encoder
            .positionConversionFactor(2 * (PD22t / motorReduction)) // Meters
            .velocityConversionFactor(2 * (PD22t / (motorReduction * 60.))) // Meters per Seconds
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
    */

    master_config
        .closedLoop
        .maxMotion
        .maxVelocity(maxVelocity)
        .maxAcceleration(maxAcceleration)
        .allowedClosedLoopError(PIDTolerance);

    master_config
        .softLimit
        .forwardSoftLimit(forwardSoftLimit)
        .reverseSoftLimit(reverseSoftLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    tryUntilOk(
        masterMotor,
        5,
        () ->
            masterMotor.configure(
                master_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    resetEncoder();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    m_elevatorSim.setInputVoltage(simFlex.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_elevatorSim.update(0.020);
    double rpm = (m_elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.PD22t) / 60.;
    simFlex.iterate(rpm, RoboRioSim.getVInVoltage(), 0.020);
    simFlex.setPosition(m_elevatorSim.getPositionMeters());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    inputs.positionMeters = simFlex.getPosition();
    inputs.velocityMetersPerSec = simFlex.getVelocity();
    inputs.appliedVolts = simFlex.getBusVoltage();
    inputs.currentAmps = simFlex.getMotorCurrent();
    inputs.isHome = isHome;
    inputs.setpoint = setpoint;

    ifOk(
        masterMotor,
        simFlex::getPosition,
        (value) -> inputs.isAtSetpoint = Math.abs(value - setpoint) < PIDTolerance);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    masterMotor.set(appliedVolts);
  }

  @Override
  public void resetEncoder() {
    simFlex.setPosition(0);
  }

  @Override
  public void setHome(boolean isHomed) {
    isHome = isHomed;
  }

  @Override
  public void setHeight(double height) {
    setpoint = height;
    closedLoopController.setReference(
        height, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  public boolean isAtSetpoint() {
    return isAtSetpoint;
  }
}
