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
public class MidPlacementPreset extends SequentialCommandGroup {
  /** Creates a new MidPlacementPreset. */
  public MidPlacementPreset(ArmExtension armExtension, ArmRotation armRotation, FlipChecker flipChecker, Wrist wrist, Lights lights, Peripherals peripherals) {
    addCommands(
      new SetArmRotationPosition(armRotation, peripherals, flipChecker, Constants.PRESET.MID_PLACEMENT),
      new ParallelCommandGroup(
        new RotateWrist(wrist, flipChecker, peripherals, Constants.PRESET.MID_PLACEMENT),
        new SetArmExtensionPosition(lights, armExtension, armRotation, Constants.MID_PLACEMENT_ARM_EXTENSION)
      )
    );
  }
}
