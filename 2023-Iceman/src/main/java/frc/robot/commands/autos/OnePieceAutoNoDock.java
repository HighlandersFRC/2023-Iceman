// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.RotateWrist;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetArmExtensionPosition;
import frc.robot.commands.SetArmRotationPosition;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FlipChecker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Wrist;

public class OnePieceAutoNoDock extends SequentialCommandGroup {
  /** Creates a new OnePieceAutoNoDock. */

  public OnePieceAutoNoDock(Drive drive, ArmExtension armExtension, ArmRotation armRotation, Wrist wrist, FlipChecker flipChecker, Peripherals peripherals, Lights lights, Intake intake) {
    addCommands(
      new ParallelCommandGroup(
        // new RotateWrist(wrist, flipChecker, Constants.HIGH_PLACEMENT_FLIPPED_WRIST_ROTATION),
        new SetArmRotationPosition(armRotation, flipChecker, Constants.HIGH_PLACEMENT_FLIPPED_ARM_ROTATION),
        new SetArmExtensionPosition(lights, armExtension, armRotation, 39)
      ),
      new WaitCommand(0.25),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25),
        new RunIntake(intake, -55, 1)
      ),
      new ParallelDeadlineGroup(
        new SetArmExtensionPosition(lights, armExtension, armRotation, 5)//,
        // new RotateWrist(wrist, flipChecker, 132)
      )
    );
  }
}
