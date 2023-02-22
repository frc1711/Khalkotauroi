// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class TeleopIntake extends CommandBase {
  
  private final Conveyor conveyor;
  private final BooleanSupplier doRunIntake, reverse;
  
  // private final Debouncer intakeDebouncer = new Debouncer(2, DebounceType.kFalling);

  public TeleopIntake(
      Conveyor conveyor,
      BooleanSupplier doRunIntake,
      BooleanSupplier reverse
  ) {
    this.conveyor = conveyor;
    this.doRunIntake = doRunIntake;
    this.reverse = reverse;
    addRequirements(conveyor);
  }

  // public void runIntake (boolean reverseMode) {
  //   boolean isIntakeRunning = intakeDebouncer.calculate();
  // }
  
  @Override
  public void initialize() {
    conveyor.stop();
  }

  
  @Override
  public void execute() {
    int reverseMultiplier = 1;
    if (reverse.getAsBoolean()) reverseMultiplier = -1;
    if (doRunIntake.getAsBoolean()) conveyor.setSpeed((12 * .25) * reverseMultiplier);
    else conveyor.stop();
  }


  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
