package frc.robot.commands;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;


public class PathAndPickup extends SequentialCommandGroup {
  public PathAndPickup(Drive drive, Peripherals peripherals, Lights lights, Intake intake, JSONArray pathJSON) {
    addRequirements(drive, peripherals, lights);
    addCommands(
      new AutonomousFollower(drive, pathJSON, false, false),
      new MoveToPieceForwards(drive, peripherals, lights, intake)
    );
  }
}
