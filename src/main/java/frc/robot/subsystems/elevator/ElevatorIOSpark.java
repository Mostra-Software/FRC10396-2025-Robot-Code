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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax masterMotor = new SparkMax(masterCanId, MotorType.kBrushless);
  private SparkClosedLoopController closedLoopController;
  private final RelativeEncoder masterEncoder = masterMotor.getEncoder();

  private final SparkMax slaveMotor = new SparkMax(slaveCanId, MotorType.kBrushless);
 // Because right motor will be slave of left motor, no need for encoder?
 // private final RelativeEncoder right_encoder = elevtator_right.getEncoder();

  public ElevatorIOSpark() {

    closedLoopController = masterMotor.getClosedLoopController();

    var master_config = new SparkMaxConfig();
    var slave_config = new SparkMaxConfig();

    master_config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    slave_config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);

    master_config
        .encoder
        .positionConversionFactor(22 * Units.inchesToMeters(0.25)/ motorReduction) //Meters
        .velocityConversionFactor(22 * Units.inchesToMeters(0.25) / 60.0 / motorReduction)//Meters per Seconds
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

      master_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(positionP)
        .i(positionI)
        .d(positionD)
        .outputRange(-1, 1);

      master_config.closedLoop.maxMotion
        .maxVelocity(maxVelocity)
        .maxAcceleration(maxAcceleration)
        .allowedClosedLoopError(1);

      slave_config
        .follow(masterCanId, true)
        .encoder
        .positionConversionFactor(22 * Units.inchesToMeters(0.25)/ motorReduction) //Meters
        .velocityConversionFactor(22 * Units.inchesToMeters(0.25) / 60.0 / motorReduction)//Meters per Seconds
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

      slave_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(positionP)
        .i(positionI)
        .d(positionD)
        .outputRange(-1, 1);

      slave_config.closedLoop.maxMotion
        .maxVelocity(maxVelocity)
        .maxAcceleration(maxAcceleration)
        .allowedClosedLoopError(1);

      
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

        

  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(masterMotor, masterEncoder::getPosition, (value) -> inputs.positionMeters = value);
    ifOk(masterMotor, masterEncoder::getVelocity, (value) -> inputs.velocityMetersPerSec = value);
    ifOk(
       masterMotor,
        new DoubleSupplier[] {masterMotor::getAppliedOutput, masterMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(masterMotor, masterMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);

    ifOk(slaveMotor, masterEncoder::getPosition, (value) -> inputs.positionMeters = value);
    ifOk(slaveMotor, masterEncoder::getVelocity, (value) -> inputs.velocityMetersPerSec = value);
    ifOk(
       slaveMotor,
        new DoubleSupplier[] {slaveMotor::getAppliedOutput, slaveMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(slaveMotor, slaveMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    masterMotor.setVoltage(volts);
  }

  @Override
  public void elevatorRunMaxMotion(double height){
      closedLoopController.setReference(height, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }
}
