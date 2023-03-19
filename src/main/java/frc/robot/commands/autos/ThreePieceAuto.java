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
import frc.robot.commands.AprilTagBalance;
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
public class ThreePieceAuto extends SequentialCommandGroup {
  /** Creates a new ThreePieceAuto. */
  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;

  private File pathingFile2;
  private JSONArray pathJSON2;
  private JSONObject pathRead2;

  private File pathingFile3;
  private JSONArray pathJSON3;
  private JSONObject pathRead3;

  private File pathingFile4;
  private JSONArray pathJSON4;
  private JSONObject pathRead4;

  public ThreePieceAuto(Drive drive, ArmExtension armExtension, ArmRotation armRotation, Wrist wrist, FlipChecker flipChecker, Peripherals peripherals, Lights lights, Intake intake) {

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

    try {
        pathingFile3 = new File("/home/lvuser/deploy/2PiecePart3.json");
        FileReader scanner3 = new FileReader(pathingFile3);
        pathRead3 = new JSONObject(new JSONTokener(scanner3));
        pathJSON3 = (JSONArray) pathRead3.get("sampled_points");
    }
    catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
        pathingFile4 = new File("/home/lvuser/deploy/3PiecePart4.json");
        FileReader scanner4 = new FileReader(pathingFile4);
        pathRead4 = new JSONObject(new JSONTokener(scanner4));
        pathJSON4 = (JSONArray) pathRead4.get("sampled_points");
    }
    catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
    }

    addRequirements(drive, armExtension, armRotation, wrist, flipChecker);
    addCommands(
      new ParallelCommandGroup(
        new RunIntake(intake, -35, 0.1),
        new RotateWrist(wrist, flipChecker, Constants.MID_PLACEMENT_BACKSIDE_WRIST_ROTATION),
        new SetArmRotationPosition(armRotation, flipChecker, Constants.MID_PLACEMENT_BACKSIDE_ARM_ROTATION),
        new SequentialCommandGroup(
          new WaitCommand(0.15),
          new SetArmExtensionPosition(lights, armExtension, armRotation, Constants.MID_PLACEMENT_ARM_EXTENSION)
        )
      ),
      new WaitCommand(0.3),
      new ParallelDeadlineGroup(
        new WaitCommand(0.1),
        new RunIntake(intake, 55, 1)
      ),
      new ParallelDeadlineGroup(
        new SetArmExtensionPosition(lights, armExtension, armRotation, 0),
        new RotateWrist(wrist, flipChecker, 180)
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.1),
          new AutonomousFollower(drive, pathJSON, false)
        ),
        new SetArmRotationPosition(armRotation, flipChecker, Constants.CUBE_FRONTSIDE_ARM_ROTATION),
        new RotateWrist(wrist, flipChecker, Constants.CUBE_FRONTSIDE_WRIST_ROTATION),
        new RunIntake(intake, -55, 1),
        new SetArmExtensionPosition(lights, armExtension, armRotation, 0),
        new SetFrontLimelightPipeline(peripherals, 2)
      ),
      new ParallelDeadlineGroup(
        new MoveToPieceForwards(drive, peripherals, lights),
        new RunIntake(intake, -55, 1),
        new SetArmRotationPosition(armRotation, flipChecker, Constants.CUBE_FRONTSIDE_ARM_ROTATION)
      ),
      new WaitCommand(0.1),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, pathJSON2, false),
        new ParallelCommandGroup(
          new RotateWrist(wrist, flipChecker, 215),
          new SetArmRotationPosition(armRotation, flipChecker, Constants.HIGH_PLACEMENT_BACKSIDE_ARM_ROTATION),
          new SequentialCommandGroup(
            new WaitCommand(0.75),
            new SetArmExtensionPosition(lights, armExtension, armRotation, 18)
          )
        ),
        new WaitCommand(0.25),
        new RunIntake(intake, -35, 0.1)
      ),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new SetArmExtensionPosition(lights, armExtension, armRotation, Constants.MID_PLACEMENT_ARM_EXTENSION),
          new SetArmRotationPosition(armRotation, flipChecker, Constants.HIGH_PLACEMENT_BACKSIDE_ARM_ROTATION),
          new RotateWrist(wrist, flipChecker, 215)
        ),
        new SetBackLimelightPipeline(peripherals, 0)
      ),
      new WaitCommand(0.1),
      new ParallelDeadlineGroup(
        new WaitCommand(0.1),
        new RunIntake(intake, 55, 0.4)
      ),
      new ParallelDeadlineGroup(
        new SetArmExtensionPosition(lights, armExtension, armRotation, 6),
        new RotateWrist(wrist, flipChecker, 180)
      ),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, pathJSON3, false),
        new SetArmRotationPosition(armRotation, flipChecker, Constants.CUBE_FRONTSIDE_ARM_ROTATION),
        new RotateWrist(wrist, flipChecker, Constants.CUBE_FRONTSIDE_WRIST_ROTATION),
        new RunIntake(intake, -55, 1),
        new SetArmExtensionPosition(lights, armExtension, armRotation, 0),
        new SetFrontLimelightPipeline(peripherals, 2)
      ),
      new ParallelDeadlineGroup(
        new MoveToPieceForwards(drive, peripherals, lights),
        new RunIntake(intake, -55, 1),
        new SetArmRotationPosition(armRotation, flipChecker, Constants.CUBE_FRONTSIDE_ARM_ROTATION)
      ),
      new WaitCommand(0.1),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, pathJSON4, false),
        new ParallelCommandGroup(
          new RotateWrist(wrist, flipChecker, 240),
          new SetArmRotationPosition(armRotation, flipChecker, Constants.MID_PLACEMENT_BACKSIDE_ARM_ROTATION),
          new SequentialCommandGroup(
            new WaitCommand(0.75),
            new SetArmExtensionPosition(lights, armExtension, armRotation, 3)
          )
        ),
        new RunIntake(intake, -35, 0.1)
      ),
      new WaitCommand(0.35),
      new RunIntake(intake, 55, 0.5),
      new WaitCommand(3)
    );
  }
}