// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RotateWrist;
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
public class LowPreset extends ParallelCommandGroup {
  /** Creates a new LowPreset. */
  public LowPreset(ArmExtension armExtension, ArmRotation armRotation, Peripherals peripherals, FlipChecker flipChecker, Wrist wrist, Lights lights) {
    addCommands(
      new SetArmRotationPosition(armRotation, peripherals, flipChecker, Constants.PRESET.LOW_PLACEMENT),
      new RotateWrist(wrist, flipChecker, peripherals, Constants.PRESET.LOW_PLACEMENT)
    );
  }
}
