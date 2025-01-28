// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.FieldConstants.*;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class TargetingSystem {

  private AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private ReefBranch targetBranch;
  private ReefBranchLevel targetBranchLevel;
  private Transform2d robotBranchScoringOffset =
      new Transform2d(Inches.of(12).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));

  public double getTargetBranchHeightMeters() {
    switch (targetBranchLevel) {
      case L2 -> {
        return ReefHeight.L2.height;
      }
      case L3 -> {
        return ReefHeight.L3.height;
      }
      case L4 -> {
        return ReefHeight.L4.height;
      }
    }
    return 0;
  }

  public void setTarget(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel) {
    this.targetBranch = targetBranch;
    this.targetBranchLevel = targetBranchLevel;
  }

  public void left() {
    if (targetBranch == ReefBranch.H) {
      targetBranch = ReefBranch.I;
    }
  }

  public Pose2d getTargetPose() {
    Pose2d scoringPose = Pose2d.kZero;
    if (targetBranch != null)
      scoringPose =
          Reef.branchPositions
              .get(targetBranch.ordinal())
              .get(ReefHeight.L2)
              .toPose2d()
              .plus(robotBranchScoringOffset);
    return AllianceFlipUtil.apply(scoringPose);
  }

  public int getNearestReefFace(Pose2d robotPose) {
    int face = 0;
    try {
      List<Pose2d> reefFaces = Arrays.asList(Reef.centerFaces);
      Pose2d targetFace = robotPose.nearest(reefFaces);
      face = reefFaces.indexOf(targetFace);
    } catch (Exception e) {
    }
    return face;
  }

  public enum ReefBranch {
    A,
    B,
    K,
    L,
    I,
    J,
    G,
    H,
    E,
    F,
    C,
    D
  }

  public final HashMap<Double[], Integer> slopeToReefFace = new HashMap<Double[], Integer>();

  public enum ReefBranchLevel {
    L2,
    L3,
    L4
  }
}
