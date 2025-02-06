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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
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

  public void setTarget(ReefBranchLevel targetBranchLevel) {
    this.targetBranchLevel = targetBranchLevel;
  }

  public ReefBranchLevel getTargetBranchLevel() {
    if (targetBranchLevel != null) return targetBranchLevel;
    else return ReefBranchLevel.L1;
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

  public Pose2d getHPZone(Pose2d robotPose) {
    Pose2d nearestHP =
        robotPose.nearest(
            Arrays.asList(
                AllianceFlipUtil.apply(CoralStation.leftCenterFace),
                AllianceFlipUtil.apply(CoralStation.rightCenterFace)));
    Transform2d delta = robotPose.minus(nearestHP);
    Measure distanceX = delta.getMeasureX();
    Measure distanceY = delta.getMeasureY();
    if (distanceX.abs(Meters) < 2 && distanceY.abs(Meters) < 2)
      return nearestHP.plus(new Transform2d(Translation2d.kZero, Rotation2d.fromDegrees(180)));
    else return null;
  }

  public List<Pose2d> getFlippedReefFaces() {
    if (AllianceFlipUtil.shouldFlip()) {
      return Arrays.asList(
          AllianceFlipUtil.apply(Reef.centerFaces[0]),
          AllianceFlipUtil.apply(Reef.centerFaces[1]),
          AllianceFlipUtil.apply(Reef.centerFaces[2]),
          AllianceFlipUtil.apply(Reef.centerFaces[3]),
          AllianceFlipUtil.apply(Reef.centerFaces[4]),
          AllianceFlipUtil.apply(Reef.centerFaces[5]));
    } else return Arrays.asList(Reef.centerFaces);
  }

  public int getNearestReefFace(Pose2d robotPose) {
    int face = 0;
    try {
      List<Pose2d> reefFaces = getFlippedReefFaces();
      Pose2d targetFace = robotPose.nearest(reefFaces);
      face = reefFaces.indexOf(targetFace);
    } catch (Exception e) {
    }
    return face;
  }

  public boolean isInHpZone(Pose2d pose){
    return (getHPZone(pose) != null);
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
    L1,
    L2,
    L3,
    L4
  }
}
