// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.swerve.Swerve;

public class PlacementSetupCommand extends CommandBase {
  
  private final Swerve swerveDrive;

  public PlacementSetupCommand(Swerve swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  public enum Alliance {
    RED (
      Limelight.INTAKE_LIMELIGHT.getFieldRelativeBotPose(false)
    ),
    BLUE (
      Limelight.INTAKE_LIMELIGHT.getFieldRelativeBotPose(true)
    );

    private double[] fieldRelativeBotPose;

    Alliance (double[] fieldRelativeBotPose) {
      this.fieldRelativeBotPose = fieldRelativeBotPose;
    }
  }

  public enum HubPosition {
    LEFT (
      3, 6
    ),
    RIGHT (
      1, 8
    ),
    MIDDLE (
      2, 7
    );

    private int tagIDRed, tagIDBlue;

    HubPosition (int tagIDRed, int tagIDBlue) {
      this.tagIDRed = tagIDRed;
      this.tagIDBlue = tagIDBlue;
    }
  }

  public enum NodeSide {
    LEFT (
      -0.21
    ),
    MIDDLE (
      0
    ),
    RIGHT (
      0.21
    ); 

    private double metersFromTag;

    NodeSide (double metersFromTag) {

    }
  }

  // public enum NodePlacement {
  //   HIGH (
  //     ArmPosition.HIGH.rotation
  //   ),
  //   MIDDLE (
  //     ArmPosition.MIDDLE.rotation
  //   ),
  //   LOW (
  //     ArmPosition.LOW.rotation
  //   );

  //   private Rotation2d armRotation;

  //   NodePlacement (Rotation2d armRotation) {
  //     this.armRotation = armRotation;
  //   } 
  // }

  double tagID;
  private void runSetupSequence (Alliance alliance, HubPosition hub, NodeSide side, ArmPosition armPosition) {
    
    tagID = alliance == Alliance.BLUE ? hub.tagIDBlue : hub.tagIDRed;

    

    swerveDrive.stop();
  }

  @Override
  public void initialize() {
    swerveDrive.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
