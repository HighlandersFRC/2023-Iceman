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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.MoveToPieceBackwards;
import frc.robot.commands.MoveToPieceForwards;
import frc.robot.commands.RotateArm;
import frc.robot.commands.RotateWrist;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetArmExtensionPosition;
import frc.robot.commands.SetArmRotationPosition;
import frc.robot.commands.SetBackLimelightPipeline;
import frc.robot.commands.SetFrontLimelightPipeline;
import frc.robot.commands.VisionAlignment;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FlipChecker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAutoNoDock extends SequentialCommandGroup {
  /** Creates a new TwoPieceAutoNoDock. */
  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;

  private File pathingFile2;
  private JSONArray pathJSON2;
  private JSONObject pathRead2;

  public TwoPieceAutoNoDock(Drive drive, ArmExtension armExtension, ArmRotation armRotation, Wrist wrist, FlipChecker flipChecker, Peripherals peripherals, Lights lights, Intake intake) {

    try {
      pathingFile = new File("/home/lvuser/deploy/2PiecePart1.json");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile2 = new File("/home/lvuser/deploy/2PiecePart2.json");
      FileReader scanner2 = new FileReader(pathingFile2);
      pathRead2 = new JSONObject(new JSONTokener(scanner2));
      pathJSON2 = (JSONArray) pathRead2.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    addRequirements(drive, armExtension, armRotation, wrist, flipChecker);
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
      ),
      new ParallelDeadlineGroup(
        new ParallelDeadlineGroup(
          new AutonomousFollower(drive, pathJSON, false),
          new SetArmRotationPosition(armRotation, flipChecker, Constants.CUBE_ARM_ROTATION),
          // new RotateWrist(wrist, flipChecker, Constants.CUBE_WRIST_ROTATION),
          new RunIntake(intake, 55, 1)
        ),
        new SetArmExtensionPosition(lights, armExtension, armRotation, 1),
        new SetFrontLimelightPipeline(peripherals, 2)
      ),
      new ParallelDeadlineGroup(
        new MoveToPieceForwards(drive, peripherals, lights),
        new SetArmRotationPosition(armRotation, flipChecker, 96)
      ),
      new WaitCommand(0.25),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, pathJSON2, false),
        new SequentialCommandGroup(
          new ParallelCommandGroup(new RotateWrist(wrist, flipChecker, 0), new SetArmRotationPosition(armRotation, flipChecker, 180)),
          new WaitCommand(1.5),
          new SetArmRotationPosition(armRotation, flipChecker, Constants.MID_PLACEMENT_FLIPPED_ARM_ROTATION)
        ),
        new RunIntake(intake, 35, 0.1)
      ),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new SetArmExtensionPosition(lights, armExtension, armRotation, 37.5),
          new SetArmRotationPosition(armRotation, flipChecker, Constants.HIGH_PLACEMENT_FLIPPED_ARM_ROTATION)//,
          // new RotateWrist(wrist, flipChecker, Constants.HIGH_PLACEMENT_FLIPPED_WRIST_ROTATION)
        ),
        new SetBackLimelightPipeline(peripherals, 0)
      ),
      new WaitCommand(0.25),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25),
        new RunIntake(intake, -45, 1)
      ),
      new ParallelDeadlineGroup(
        new SetArmExtensionPosition(lights, armExtension, armRotation, 2)//,
        // new RotateWrist(wrist, flipChecker, 0)
      )
    );
  }
}
