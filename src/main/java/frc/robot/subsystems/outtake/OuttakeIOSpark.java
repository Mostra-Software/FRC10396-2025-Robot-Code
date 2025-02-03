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

import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.function.DoubleSupplier;

public class OuttakeIOSpark implements OuttakeIO {
  private final SparkFlex outtakeMotor = new SparkFlex(outtakeCanId, MotorType.kBrushless);
  private final LaserCan rearSensor = new LaserCan(SensorID);

  public OuttakeIOSpark() {

    var master_config = new SparkFlexConfig();

    master_config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);

    tryUntilOk(
        outtakeMotor,
        5,
        () ->
            outtakeMotor.configure(
                master_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    ifOk(
        outtakeMotor,
        new DoubleSupplier[] {outtakeMotor::getAppliedOutput, outtakeMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(outtakeMotor, outtakeMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.hasGP = (rearSensor.getMeasurement().distance_mm <= SensorTriggerDistance);
    inputs.distance_mm = rearSensor.getMeasurement().distance_mm;
  }

  @Override
  public void setVoltage(double volts) {
    outtakeMotor.setVoltage(volts);
  }
}
