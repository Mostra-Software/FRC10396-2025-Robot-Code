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

public class OuttakeConstants {

  public static final int frontSensorID = 0;
  public static final int rearSensorID = 0;
  public static final int outtakeCanId = 0;
  public static final int currentLimit = 40;

  public static final double motorReduction = 1;

  // Position PID Parameters
  public static final double positionP = 0.4;
  public static final double positionI = 0.0;
  public static final double positionD = 0.0;
  public static final int maxPositionVelocity = 1000;
  public static final int maxPositionAcceleration = 1000;

  // Velocity PID Paramteres
  public static final double velocityP = 0.0001;
  public static final double velocityI = 0;
  public static final double velocityD = 0;
  public static final int maxVelocityVelocity = 500;
  public static final int maxVelocityAcceleration = 6000;

  // Sensor Constants
  public static final int frontSensorTriggerDistance = 12;
  public static final int rearSensorTriggerDistance = 12;
}
