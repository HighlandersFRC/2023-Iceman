// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Peripherals;

public class SetBackLimelightPipeline extends CommandBase {
  /** Creates a new SetBackLimelightPipeline. */
  private Peripherals peripherals;
  private int pipeline;
  public SetBackLimelightPipeline(Peripherals peripherals, int pipeline) {
    this.peripherals = peripherals;
    this.pipeline = pipeline;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    peripherals.setBackPipeline(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peripherals.setBackPipeline(pipeline);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Pipeline: " + peripherals.getBackLimelightPipeline());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(peripherals.getBackLimelightPipeline() == pipeline) {
      return true;
    }
    return false;
  }
}
