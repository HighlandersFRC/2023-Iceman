// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RotateWrist;
import frc.robot.commands.SetArmExtensionPosition;
import frc.robot.commands.SetArmRotationPosition;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.FlipChecker;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighPlacementPreset extends SequentialCommandGroup {
  /** Creates a new HighPlacementPreset. */
  public HighPlacementPreset(ArmExtension armExtension, ArmRotation armRotation, FlipChecker flipChecker, Wrist wrist, Lights lights, Peripherals periphals) {
    addCommands(
      new SetArmRotationPosition(armRotation, periphals, flipChecker, Constants.PRESET.HIGH_PLACEMENT),
      new ParallelCommandGroup(
        new RotateWrist(wrist, flipChecker, periphals,  Constants.PRESET.HIGH_PLACEMENT),
        new SetArmExtensionPosition(lights, armExtension, armRotation, Constants.HIGH_PLACEMENT_ARM_EXTENSION)
      )
    );
  }
}
