// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.OptionalLong;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.RotationalPID;
import frc.robot.subsystems.swerve.Swerve;
import util.NumericDebouncer;
import frc.robot.subsystems.Vision;

public class AprilTags extends CommandBase {

  private static final double MAX_OFFSET_DEGREES = 5;

  private Swerve swerveDrive;
  private Vision vision;
  private RotationalPID rotationPID = new RotationalPID("LimelightRotationPID", .16, 0, 0, MAX_OFFSET_DEGREES);
  private NumericDebouncer aprilTagMeasurement = new NumericDebouncer (new Debouncer(0.2, DebounceType.kFalling));
  private SlewRateLimiter turnRateLimiter = new SlewRateLimiter(8);

  public AprilTags(
    Swerve swerveDrive,
    Vision vision) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    addRequirements(swerveDrive);
  }

  //Get Swerve to drive to the tag given as input
  Transform3d transform;
  public void driveToTag () {
    if (vision.seesTarget()) swerveDrive.moveFieldRelative(new ChassisSpeeds(0, vision.getDistance(), 0));
  }

  public double getTurnToTag (Optional<Double> targetRotationDegrees) {
    double turnSpeed;

    if (targetRotationDegrees.isPresent()) {
      Rotation2d targetRotation = Rotation2d.fromDegrees(targetRotationDegrees.get());
      Rotation2d robotRotation = Rotation2d.fromDegrees(swerveDrive.getRobotYaw());
      turnSpeed = rotationPID.calculate(robotRotation, targetRotation);
    } else {
      turnSpeed = 0;
    }

    return turnRateLimiter.calculate(turnSpeed);
  }

  // Check if any cameras have been activated, if true, continue normally, if false start a new camera
  @Override
  public void initialize() {
    swerveDrive.stop();
  }

  // Search for AprilTags on the camera. If any are found, drive toward them
  @Override
  public void execute() {
    Optional<Double> targetRotation;

    if (vision.seesTarget()) {
      Rotation2d rotationMeasurement = Rotation2d.fromDegrees(swerveDrive.getRobotYaw() + vision.getHorizontalOffset());
      targetRotation = aprilTagMeasurement.calculate(Optional.of(rotationMeasurement.getDegrees()));
    } else {
      targetRotation = aprilTagMeasurement.calculate(Optional.empty());
    }

    double turnRate = getTurnToTag(targetRotation);

    swerveDrive.moveRobotRelative(new ChassisSpeeds(0, 0, turnRate));
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
