// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlacementCube extends SequentialCommandGroup {
  /** Creates a new AutoPlacement. */
  public AutoPlacementCube(Drive drive, Peripherals peripherals, Lights lights, int rowOffset) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive);
    addCommands(
      new ParallelCommandGroup(new AlignForPlacement(drive, rowOffset))
    );
  }
}
