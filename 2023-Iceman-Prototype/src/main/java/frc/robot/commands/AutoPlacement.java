// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlacement extends SequentialCommandGroup {
  /** Creates a new AutoPlacement. */
  public AutoPlacement(Drive drive, ArmRotation arm, ArmExtension armExtension, Wrist wrist, int rowOffset) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive, arm, armExtension, wrist);
    addCommands(
      new ParallelCommandGroup(new AlignForPlacement(drive, armExtension, arm, wrist, rowOffset)),
      new SetArmExtensionPosition(armExtension, 25)
    );
  }
}
