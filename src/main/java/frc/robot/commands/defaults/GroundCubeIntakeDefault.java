// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.GroundCubeIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class GroundCubeIntakeDefault extends CommandBase {
  /** Creates a new GroundCubeIntakeDefault. */
  private GroundCubeIntake Intake;
  public GroundCubeIntakeDefault(GroundCubeIntake Intake) {
    this.Intake = Intake;
    addRequirements(this.Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.setIntakeRotation(0);
    Intake.setIntakeTorqueOutput(45, 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
