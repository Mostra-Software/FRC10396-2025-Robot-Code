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

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

  public static final int sensorID = 0;
  public static final int masterCanId = 41;
  public static final int slaveCanId = 42;
  public static final double motorReduction = (50. / 46.) * 9.;
  public static final int currentLimit = 40;

  public static final double homeSequenceSlowPoint = 15;

  public static final double forwardSoftLimit = 0.615;
  public static final double reverseSoftLimit = 0;
  public static final double PD22t =
      Units.inchesToMeters(0.25) / Math.sin(Units.degreesToRadians(180. / 22.));

  // Position PID Parameters
  public static final double positionP = 15;
  public static final double positionI = 0.0;
  public static final double positionD = 0.5;
  public static final double PIDTolerance = 0.100;
  public static final int maxVelocity = 5000;
  public static final int maxAcceleration = 12000;
  public static final double preHomingPosition = 0.015;
  public static final int homingCurrent = 30;

  public static final double kCarriageMass = 10;
  public static final double kMinElevatorHeightMeters = 0;
  public static final double kMaxElevatorHeightMeters = 2;

  public static final double L1Height = 0.0;
  public static final double L2Height = 0.144;
  public static final double L3Height = 0.323;
  public static final double L4Height = 0.612;
}
