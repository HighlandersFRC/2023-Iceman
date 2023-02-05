// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONArray;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignForPlacement extends SequentialCommandGroup {
  /** Creates a new AlignForPlacement. */
  // -1, 0, 1 for left, middle, right
  public AlignForPlacement(Drive drive, ArmExtension armExtension, ArmRotation armRotation, Wrist wrist, int rowOffset) {
    addRequirements(drive, armExtension, armRotation, wrist);
    addCommands(new AutonomousFollower(drive, armExtension, armRotation, wrist, true, false, rowOffset));
  }
}
