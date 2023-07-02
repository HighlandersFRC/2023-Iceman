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
import frc.robot.commands.AprilTagBalance;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.DriveBackOnChargeStation;
import frc.robot.commands.DriveOverChargeStation;
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

public class OnePieceAuto extends SequentialCommandGroup {
  /** Creates a new OnePieceAuto. */
  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;

  public OnePieceAuto(Drive drive, ArmExtension armExtension, ArmRotation armRotation, Wrist wrist, FlipChecker flipChecker, Peripherals peripherals, Lights lights, Intake intake) {
    
    try {
      pathingFile = new File("/home/lvuser/deploy/1PieceDock.json");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    addCommands(
      new ParallelCommandGroup(
        new RunIntake(intake, -35, 0.1),
        new RotateWrist(wrist, flipChecker, Constants.HIGH_PLACEMENT_BACKSIDE_WRIST_ROTATION),
        new SetArmRotationPosition(armRotation, flipChecker, Constants.HIGH_PLACEMENT_BACKSIDE_ARM_ROTATION),
        new SequentialCommandGroup(
          new WaitCommand(0.3),
          new SetArmExtensionPosition(lights, armExtension, armRotation, Constants.HIGH_PLACEMENT_ARM_EXTENSION)
        )
      ),
      new WaitCommand(0.25),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25),
        new RunIntake(intake, 55, 1)
      ),
      new ParallelDeadlineGroup(
        new SetArmExtensionPosition(lights, armExtension, armRotation, 3),
        new RotateWrist(wrist, flipChecker, 180),
        new RunIntake(intake, -35, 0.1)

      ),
      new SetArmRotationPosition(armRotation, flipChecker, 180),
      new DriveOverChargeStation(drive, peripherals),
      new WaitCommand(0.58),
      new DriveBackOnChargeStation(drive, peripherals),
      new WaitCommand(1),
      new AutoBalance(drive, peripherals)
    );
  }
}
