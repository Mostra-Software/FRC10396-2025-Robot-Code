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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class OuttakeIOSpark implements OuttakeIO {
  private final SparkMax outtakeMotor = new SparkMax(outtakeCanId, MotorType.kBrushless);
  private SparkClosedLoopController closedLoopController;
  private final RelativeEncoder outtakeEncoder = outtakeMotor.getEncoder();

  public OuttakeIOSpark() {

    closedLoopController = outtakeMotor.getClosedLoopController();

    var master_config = new SparkMaxConfig();

    master_config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);

    master_config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / motorReduction) // Rotor Rotations -> Roller Radians
        .velocityConversionFactor(1) // Motor RPM
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    master_config
        .closedLoop
        // Posiiton Config
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(positionP)
        .i(positionI)
        .d(positionD)
        .outputRange(-1, 1)
        // Velocity Config
        .p(velocityP, ClosedLoopSlot.kSlot1)
        .i(velocityI, ClosedLoopSlot.kSlot1)
        .d(velocityD, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    master_config
        .closedLoop
        .maxMotion
        .maxVelocity(maxPositionVelocity)
        .maxAcceleration(maxPositionAcceleration)
        .allowedClosedLoopError(1)
        // Velocity Config
        .maxAcceleration(maxVelocityVelocity, ClosedLoopSlot.kSlot1)
        .maxVelocity(maxVelocityAcceleration, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

    tryUntilOk(
        outtakeMotor,
        5,
        () ->
            outtakeMotor.configure(
                master_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    ifOk(outtakeMotor, outtakeEncoder::getPosition, (value) -> inputs.position = value);
    ifOk(outtakeMotor, outtakeEncoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        outtakeMotor,
        new DoubleSupplier[] {outtakeMotor::getAppliedOutput, outtakeMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(outtakeMotor, outtakeMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    outtakeMotor.setVoltage(volts);
  }

  @Override
  public void outtakeRunClosedLoopVelocity(int rpm) {
    closedLoopController.setReference(
        rpm, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }
}
