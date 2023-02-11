// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    try {
      pathingFile = new File("/home/lvuser/deploy/2PiecePart1.json");
      FileReader scanner = new FileReader(pathingFile);
      // pathRead = new JSONTokener(scanner);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
      JSONArray first = pathJSON.getJSONArray(0);
      double firstTime = first.getDouble(0);
      System.out.println(first);
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile2 = new File("/home/lvuser/deploy/2PiecePart2.json");
      FileReader scanner2 = new FileReader(pathingFile2);
      // pathRead = new JSONTokener(scanner);
      pathRead2 = new JSONObject(new JSONTokener(scanner2));
      pathJSON2 = (JSONArray) pathRead2.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile3 = new File("/home/lvuser/deploy/2PiecePart3Dock.json");
      FileReader scanner3 = new FileReader(pathingFile3);
      // pathRead = new JSONTokener(scanner);
      pathRead3 = new JSONObject(new JSONTokener(scanner3));
      pathJSON3 = (JSONArray) pathRead3.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    addRequirements(drive, armExtension, armRotation, wrist);
    addCommands(
      new SetArmRotationPosition(armRotation, 131),
      new SetArmExtensionPosition(armExtension, 21),
      new WaitCommand(0.25),
      new RunWrist(wrist, -1, 0.5),
      new SetArmExtensionPosition(armExtension, 1),
      new ParallelDeadlineGroup(
        new AutonomousFollower(drive, pathJSON, false),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new SetArmRotationPosition(armRotation, 91),
          new RunWrist(wrist, 1, 4)
        )
      ),
      new ParallelCommandGroup(
        new RunWrist(wrist, -0.1, 4),
        new AutonomousFollower(drive, pathJSON2, false),
        new SequentialCommandGroup(
          new SetArmRotationPosition(armRotation, 180),
          new WaitCommand(2.5),
          new ParallelCommandGroup(
            new SetVisionAlignmentPipeline(peripherals),
            new SetArmRotationPosition(armRotation, 131)
          )
        )
      ),
      new VisionAlignment(drive, peripherals, lights),
      new SetArmExtensionPosition(armExtension, 8),
      new WaitCommand(0.25),
      new RunWrist(wrist, -1, 0.5),
      new SetArmExtensionPosition(armExtension, 1),
      new ParallelCommandGroup(
        new AutonomousFollower(drive, pathJSON3, false),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new SetArmRotationPosition(armRotation, 240)
        )
      ),
      new AutoBalance(drive)
    );
  }
}
