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
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Wrist;
import frc.robot.tools.math.Vector;

public class AutonomousFollower extends CommandBase {
    private Drive drive;

    private JSONArray path;
    private JSONArray commands;

    private double initTime;
    private double currentTime;
    private double previousTime;

    private double odometryFusedX = 0;
    private double odometryFusedY = 0;
    private double odometryFusedTheta = 0;

    private double[] desiredVelocityArray = new double[3];
    private double desiredThetaChange = 0;

    private boolean record;
    private String fieldSide;
    private int rowOffset = 0;

    private ArrayList<double[]> recordedOdometry = new ArrayList<double[]>();
    private boolean generatePath = false;
    /** Creates a new AutonomousFollower. */
  public AutonomousFollower(Drive drive, JSONArray pathPoints, boolean record) {
    this.drive = drive;
    this.path = pathPoints;
    this.record = record;
    addRequirements(this.drive);
  }

  public AutonomousFollower(Drive drive, boolean generatePath, boolean record, int rowOffset){
    this.drive = drive;
    this.rowOffset = rowOffset;
    this.record = record;
    this.generatePath = generatePath;
    this.commands = new JSONArray();
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(generatePath == true) {
      this.fieldSide = drive.getFieldSide();
      // if (this.fieldSide == "red"){
        // System.out.println("Before: " + drive.getNavxAngle());
        while (drive.getNavxAngle() <= -180.0){
          drive.setNavxAngle(360.0);
        }
        if (drive.getNavxAngle() <= 0){
          drive.setNavxAngle(180.0);
        } else {
          drive.setNavxAngle(-180.0);
        }
        // System.out.println("After: " + drive.getNavxAngle());
      // }
      int row = drive.getClosestPlacementGroup(this.fieldSide, drive.getFusedOdometryX(), drive.getFusedOdometryY()) + this.rowOffset;
      this.path = drive.generatePlacementPathOnTheFly(row, this.fieldSide);
      // System.out.println("Path: " + this.path.toString());
    }

    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive.updateOdometryFusedArray();
    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();

    // System.out.println("X: " + odometryFusedX + " Y: " + odometryFusedY + " Theta: " + odometryFusedTheta);

    currentTime = Timer.getFPGATimestamp() - initTime;
    
    // call PIDController function
    desiredVelocityArray = drive.pidController(odometryFusedX, odometryFusedY, odometryFusedTheta, currentTime, path);
    
    // create velocity vector and set desired theta change
    Vector velocityVector = new Vector(desiredVelocityArray[0], desiredVelocityArray[1]);
    desiredThetaChange = desiredVelocityArray[2];

    drive.autoDrive(velocityVector, desiredThetaChange);

    previousTime = currentTime;

    // previousVelocityX = desiredVelocityArray[0];
    // previousVelocityY = desiredVelocityArray[1];
    // previousVelocityTheta = desiredThetaChange;
    if (this.record){
      recordedOdometry.add(new double[] {currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta});
      // recordedOdometry.add(new double[] {currentTime, desiredVelocityArray[0], desiredVelocityArray[1], drive.getFrontRightModuleDistance(), drive.getFrontRightModuleVelocity()});
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.useCameraInOdometry();
    // drive.setNavxAfterAuto();
    Vector velocityVector = new Vector(0, 0);
    double desiredThetaChange = 0;
    drive.autoDrive(velocityVector, desiredThetaChange);

    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();
    currentTime = Timer.getFPGATimestamp() - initTime;
    if (this.generatePath){
      if (this.fieldSide == "red"){
        drive.setNavxAngle(-180.0);
      }
    }
    if (this.record){
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(generatePath == true) {
    //   if(currentTime > (path.getJSONArray(path.length() - 1).getDouble(0) + 0.5)) {
    //     return true;
    //   }
    //   // if(Math.abs(drive.getFusedOdometryX() - path.getJSONArray(path.length() - 1).getDouble(1)) < 0.05) {
    //   //   if(Math.abs(drive.getFusedOdometryY() - path.getJSONArray(path.length() - 1).getDouble(2)) < 0.1) {
    //   //     return true;
    //   //   }
    //   // }
    // }
    if(currentTime > path.getJSONArray(path.length() - 1).getDouble(0)) {
      return true;
    }
    else {
      return false;
    }
  }
}
