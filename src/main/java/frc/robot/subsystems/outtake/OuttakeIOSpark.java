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

package frc.robot.subsystems.outtake;

import static frc.robot.subsystems.outtake.OuttakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
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
import java.util.function.DoubleSupplier;

public class OuttakeIOSpark implements OuttakeIO {

  private SparkFlex outtakeMotor = new SparkFlex(outtakeCanId, MotorType.kBrushless);

  private SparkFlex deAlgMotor = new SparkFlex(deAlgCanId, MotorType.kBrushless);
  private RelativeEncoder deAlgEncoder = deAlgMotor.getEncoder();
  private SparkClosedLoopController closedLoopController = deAlgMotor.getClosedLoopController();

  private LaserCan lc = new LaserCan(SensorID);

  private double setpoint = 0.0;
  private boolean isAtSetpoint = false;
  private boolean isHome = true;

  public OuttakeIOSpark() {

    var master_config = new SparkFlexConfig();
    var deAlg_config = new SparkFlexConfig();

    master_config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);

    deAlg_config
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);

    deAlg_config
        .encoder
        .positionConversionFactor(0.02528 * 360.) // No unit
        .velocityConversionFactor(1) // No unit
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    deAlg_config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(positionP)
        .i(positionI)
        .d(positionD)
        .outputRange(-1, 1);

    deAlg_config
        .softLimit
        .forwardSoftLimit(forwardSoftLimit)
        .reverseSoftLimit(reverseSoftLimit)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);

    tryUntilOk(
        outtakeMotor,
        5,
        () ->
            outtakeMotor.configure(
                master_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        deAlgMotor,
        5,
        () ->
            deAlgMotor.configure(
                deAlg_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(16, 16, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    resetEncoder();
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.hasGP = (measurement.distance_mm <= SensorTriggerDistance);
    } else inputs.hasGP = false;

    ifOk(
        outtakeMotor,
        new DoubleSupplier[] {outtakeMotor::getAppliedOutput, outtakeMotor::getBusVoltage},
        (values) -> inputs.appliedVoltsOuttake = values[0] * values[1]);
    ifOk(
        outtakeMotor, outtakeMotor::getOutputCurrent, (value) -> inputs.currentAmpsOuttake = value);
    ifOk(
        outtakeMotor,
        outtakeMotor::getMotorTemperature,
        (value) -> inputs.deviceTempOuttake = value);

    ifOk(
        deAlgMotor,
        new DoubleSupplier[] {deAlgMotor::getAppliedOutput, deAlgMotor::getBusVoltage},
        (values) -> inputs.appliedVoltsdeAlg = values[0] * values[1]);
    ifOk(deAlgMotor, deAlgMotor::getOutputCurrent, (value) -> inputs.currentAmpsdeAlg = value);
    ifOk(deAlgMotor, deAlgMotor::getMotorTemperature, (value) -> inputs.deviceTempdeAlg = value);

    inputs.hasGP = (lc.getMeasurement().distance_mm <= SensorTriggerDistance);
    inputs.distance_mm = lc.getMeasurement().distance_mm;
    inputs.isHome = isHome;
    inputs.angle = deAlgEncoder.getPosition();

    ifOk(
        deAlgMotor,
        deAlgEncoder::getPosition,
        (value) -> inputs.isAtSetpoint = Math.abs(value - setpoint) < PIDtolerance);
  }

  @Override
  public void setVoltage(double volts) {
    outtakeMotor.setVoltage(volts);
  }

  @Override
  public void setAngle(double angle) {
    setpoint = angle;
    closedLoopController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setArmPercent(double percent) {
    deAlgMotor.setVoltage(percent * 12.0);
  }

  @Override
  public void resetEncoder() {
    deAlgEncoder.setPosition(0);
  }

  @Override
  public void setHome(boolean home) {
    isHome = home;
  }
}
