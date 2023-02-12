// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Peripherals;

public class SetLimelightPipeline extends CommandBase {
  /** Creates a new SetLimelightPipeline. */
  private Peripherals peripherals;
  private int pipeline;
  public SetLimelightPipeline(Peripherals peripherals, int pipeline) {
    this.peripherals = peripherals;
    this.pipeline = pipeline;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    peripherals.setFrontPipeline(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peripherals.setFrontPipeline(pipeline);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Pipeline: " + peripherals.getFrontLimelightPipeline());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(peripherals.getFrontLimelightPipeline() == pipeline) {
      return true;
    }
    return false;
  }
}
