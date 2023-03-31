package frc.robot.commands.defaults;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drive;

public class DriveDefault extends CommandBase {
  /** Creates a new DriveDefault. */
  private Drive drive;  

  private ArrayList<double[]> recordedOdometry = new ArrayList<double[]>();

  private double initTime;

  public DriveDefault(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);
    // System.out.println("INSIDE DEFAULT CONSTRUCTOR");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
      // System.out.println("INSIDE DRIVE DEFAULT!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double odometryFusedX = drive.getFusedOdometryX();
    double odometryFusedY = drive.getFusedOdometryY();
    double odometryFusedTheta = drive.getFusedOdometryTheta();
    double currentTime = Timer.getFPGATimestamp() - initTime;

    recordedOdometry.add(new double[] {currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta});
    drive.teleopDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy-MM-dd-hh-mm-ss");
      LocalDateTime now = LocalDateTime.now();
      String filename = "/home/lvuser/deploy/recordings/" + dtf.format(now) + ".csv";
      File file = new File(filename);
      if (!file.exists()){
        file.createNewFile();
      }
      FileWriter fw = new FileWriter(file);
      BufferedWriter bw = new BufferedWriter(fw);
      for (int i = 0; i <recordedOdometry.size(); i ++){
        String line = "";
        for (double val : recordedOdometry.get(i)){
          line += val + ",";
        }
        line = line.substring(0, line.length() - 1);
        line += "\n";
        // System.out.println(line);
        bw.write(line);
      }
      
      bw.close();
    } catch (Exception e) {
      System.out.println(e);
      System.out.println("CSV file error");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}