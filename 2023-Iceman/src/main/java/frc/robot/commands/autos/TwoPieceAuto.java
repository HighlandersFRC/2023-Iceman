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
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.MoveToPieceBackwards;
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
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
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

  public TwoPieceAuto(Drive drive, ArmExtension armExtension, ArmRotation armRotation, Wrist wrist, Peripherals peripherals, Lights lights) {

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
      pathingFile3 = new File("/home/lvuser/deploy/2PiecePart3Dock.json");
      FileReader scanner3 = new FileReader(pathingFile3);
      pathRead3 = new JSONObject(new JSONTokener(scanner3));
      pathJSON3 = (JSONArray) pathRead3.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    addRequirements(drive, armExtension, armRotation, wrist);
    addCommands(
      new ParallelCommandGroup(
        new RotateWrist(wrist, 127),
        new SetArmRotationPosition(armRotation, 139),
        new SetArmExtensionPosition(armExtension, armRotation, 37)
      ),
      new WaitCommand(0.25),
      new SetArmExtensionPosition(armExtension, armRotation, 2),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new AutonomousFollower(drive, pathJSON, false),
          new SetArmRotationPosition(armRotation, 270.5),
          new RotateWrist(wrist, -63)
        ),
        new SetBackLimelightPipeline(peripherals, 2)
      ),
      new ParallelDeadlineGroup(
        new MoveToPieceBackwards(drive, peripherals, lights),
        new SetArmRotationPosition(armRotation, 269)
      ),
      new WaitCommand(0.25),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, pathJSON2, false),
        new SequentialCommandGroup(
          new SetArmRotationPosition(armRotation, 180),
          new WaitCommand(1.75),
          new ParallelCommandGroup(
            new SetFrontLimelightPipeline(peripherals, 1),
            new SetArmRotationPosition(armRotation, 131)
          )
        )
      ),
      new VisionAlignment(drive, peripherals, lights),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new SetArmExtensionPosition(armExtension, armRotation, 14),
          new SetArmRotationPosition(armRotation, 139),
          new RotateWrist(wrist, 123)
        ),
        new SetFrontLimelightPipeline(peripherals, 0)
      ),
      new WaitCommand(0.25),
      new SetArmExtensionPosition(armExtension, armRotation, 2),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, pathJSON3, false),
        new SequentialCommandGroup(
          new WaitCommand(0.9),
          new SetArmRotationPosition(armRotation, 240)
        )
      ),
      new AutoBalance(drive, 0.55)
    );
  }
}
