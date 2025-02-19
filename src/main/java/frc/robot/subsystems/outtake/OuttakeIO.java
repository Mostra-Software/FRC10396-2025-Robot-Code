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

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public static class OuttakeIOInputs {
    public double appliedVoltsOuttake = 0.0;
    public double currentAmpsOuttake = 0.0;
    public double deviceTempOuttake = 0.0;

    public double appliedVoltsdeAlg = 0.0;
    public double currentAmpsdeAlg = 0.0;
    public double deviceTempdeAlg = 0.0;

    public double setpoint = 0.0;
    public boolean isAtSetpoint = false;

    public double distance_mm = 0.0;
    public boolean hasGP = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(OuttakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void resetEncoder() {}

  public default void setAngle(double angle) {}
}
