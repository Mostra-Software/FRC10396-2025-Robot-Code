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

package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

public class ClimbIOSpark implements ClimbIO {
  private final SparkFlex masterMotor = new SparkFlex(masterCanId, MotorType.kBrushless);
  private final RelativeEncoder masterEncoder = masterMotor.getEncoder();
  private final SparkFlex slaveMotor = new SparkFlex(slaveCanId, MotorType.kBrushless);
  private SparkClosedLoopController closedLoopController = masterMotor.getClosedLoopController();
  private double setpoint = 0.0;
  private boolean isAtSetpoint = false;

  public ClimbIOSpark() {

    var master_config = new SparkFlexConfig();
    var slave_config = new SparkFlexConfig();

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
        .positionConversionFactor(1) // No Unit
        .velocityConversionFactor(1) // No unit
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
        .softLimit
        .forwardSoftLimit(forwardSoftLimit)
        .reverseSoftLimit(reverseSoftLimit)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);

    slave_config
        .follow(masterCanId, true)
        .encoder
        .positionConversionFactor(1) // No unit
        .velocityConversionFactor(1) // No unit
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    slave_config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(positionP)
        .i(positionI)
        .d(positionD)
        .outputRange(-1, 1);

    slave_config
        .softLimit
        .forwardSoftLimit(forwardSoftLimit)
        .reverseSoftLimit(reverseSoftLimit)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);

    tryUntilOk(
        masterMotor,
        5,
        () ->
            masterMotor.configure(
                master_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        slaveMotor,
        5,
        () ->
            slaveMotor.configure(
                slave_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    resetEncoder();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    ifOk(masterMotor, masterEncoder::getPosition, (value) -> inputs.positionDeg = value);
    ifOk(masterMotor, masterEncoder::getVelocity, (value) -> inputs.velocityDegPerSec = value);
    ifOk(
        masterMotor,
        new DoubleSupplier[] {masterMotor::getAppliedOutput, masterMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(masterMotor, masterMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);

    ifOk(slaveMotor, masterEncoder::getPosition, (value) -> inputs.positionDeg = value);
    ifOk(slaveMotor, masterEncoder::getVelocity, (value) -> inputs.velocityDegPerSec = value);
    ifOk(
        slaveMotor,
        new DoubleSupplier[] {slaveMotor::getAppliedOutput, slaveMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(slaveMotor, slaveMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.setpoint = setpoint;

    ifOk(
        masterMotor,
        masterEncoder::getPosition,
        (value) -> inputs.isAtSetpoint = Math.abs(value - setpoint) < PIDTolerance);
  }

  @Override
  public void setVoltage(double volts) {
    masterMotor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  @Override
  public void setDegree(double degree) {
    setpoint = degree;
    closedLoopController.setReference(degree, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void resetEncoder() {
    masterEncoder.setPosition(0);
  }

  public boolean isAtSetpoint() {
    return isAtSetpoint;
  }
}
