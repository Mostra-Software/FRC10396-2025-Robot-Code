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
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  private DCMotor gearbox = DCMotor.getNeoVortex(2);
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(2), 0.004, motorReduction),
          gearbox);
  private SparkFlex masterMotor = new SparkFlex(11, MotorType.kBrushless);
  private SparkFlexSim simFlex = new SparkFlexSim(masterMotor, gearbox);
  SparkRelativeEncoder encoder = (SparkRelativeEncoder) masterMotor.getEncoder();
  private SparkClosedLoopController closedLoopController = masterMotor.getClosedLoopController();

  private double appliedVolts = 0.0;
  private double setpoint = 0.0;
  private boolean isHome = false;
  private boolean isAtSetpoint = false;

  public ElevatorIOSim() {
    var master_config = new SparkMaxConfig();
    var slave_config = new SparkMaxConfig();

    master_config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);

    slave_config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);

    master_config
        .encoder
        .positionConversionFactor(2 * (PD22t / motorReduction)) // Meters
        .velocityConversionFactor(2 * (PD22t / (motorReduction * 60.))) // Meters per Seconds
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    master_config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(positionP)
        .i(positionI)
        .d(positionD)
        .outputRange(-1, 1);

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
    simFlex.iterate(maxVelocity, PD22t, PIDTolerance);

    // inputs.positionMeters = sim.getAngularPositionRad();
    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = simFlex.getMotorCurrent();
    inputs.isHome = isHome;
    inputs.setpoint = setpoint;

    ifOk(
        masterMotor,
        encoder::getPosition,
        (value) -> inputs.isAtSetpoint = Math.abs(value - setpoint) < PIDTolerance);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0);
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
