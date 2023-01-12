// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONArray;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.File;
import java.util.*;
import java.time.format.DateTimeFormatter;
import java.time.LocalDateTime;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.tools.math.Vector;

public class AutonomousFollower extends CommandBase {
    private Drive drive;
    private JSONArray path;

    private double initTime;
    private double currentTime;
    private double previousTime;
    private double timeDiff;

    private double previousX = 0;
    private double previousY = 0;
    private double previousTheta = 0;

    private double currentXVelocity = 0;
    private double currentYVelocity = 0;
    private double currentThetaVelocity = 0;

    private double odometryFusedX = 0;
    private double odometryFusedY = 0;
    private double odometryFusedTheta = 0;

    private double previousVelocityX = 0;
    private double previousVelocityY = 0;
    private double previousVelocityTheta = 0;

    private double[] desiredVelocityArray = new double[3];
    private double desiredThetaChange = 0;
    private ArrayList<double[]> recordedOdometry = new ArrayList<double[]>();
    /** Creates a new AutonomousFollower. */
  public AutonomousFollower(Drive drive, JSONArray pathPoints) {
    this.drive = drive;
    this.path = pathPoints;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();

    currentTime = Timer.getFPGATimestamp() - initTime;
    timeDiff = currentTime - previousTime;

    // determine current velocities based on current position minus previous position divided by time difference
    currentXVelocity = (odometryFusedX - previousX)/timeDiff;
    currentYVelocity = (odometryFusedY - previousY)/timeDiff;
    currentThetaVelocity = (odometryFusedTheta - previousTheta)/timeDiff;

    currentXVelocity = previousVelocityX;
    currentYVelocity = previousVelocityY;
    currentThetaVelocity = previousVelocityTheta;

    // call PIDController function
    desiredVelocityArray = drive.pidController(odometryFusedX, odometryFusedY, odometryFusedTheta, currentTime, path);
    
    // create velocity vector and set desired theta change
    Vector velocityVector = new Vector(desiredVelocityArray[0], desiredVelocityArray[1]);
    desiredThetaChange = desiredVelocityArray[2];

    drive.autoDrive(velocityVector, desiredThetaChange);

    // set all previous variables to current variables to stay up to date
    previousX = odometryFusedX;
    previousY = odometryFusedY;
    previousTheta = odometryFusedTheta;
    previousTime = currentTime;

    // previousVelocityX = desiredVelocityArray[0];
    // previousVelocityY = desiredVelocityArray[1];
    // previousVelocityTheta = desiredThetaChange;
    recordedOdometry.add(new double[] {currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta});
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Vector velocityVector = new Vector(0, 0);
    double desiredThetaChange = 0;
    drive.autoDrive(velocityVector, desiredThetaChange);

    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();
    currentTime = Timer.getFPGATimestamp() - initTime;

    recordedOdometry.add(new double[] {currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta});

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
    if(currentTime > path.getJSONArray(path.length() - 1).getDouble(0)) {
      return true;
    }
    else {
      return false;
    }
  }
}
