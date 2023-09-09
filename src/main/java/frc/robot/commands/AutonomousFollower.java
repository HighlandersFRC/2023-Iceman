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
    private double endPathTime;

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
    private boolean stopAtEnd = true;
    private ArrayList<Vector> endingVelocities = new ArrayList<Vector>();
    /** Creates a new AutonomousFollower. */
  public AutonomousFollower(Drive drive, JSONArray pathPoints, boolean record) {
    this.drive = drive;
    this.path = pathPoints;
    this.record = record;
    this.generatePath = false;
    this.stopAtEnd = true;
    addRequirements(this.drive);
  }

  public AutonomousFollower(Drive drive, JSONArray pathPoints, boolean record, boolean stopAtEnd) {
    this.drive = drive;
    this.path = pathPoints;
    this.record = record;
    this.generatePath = false;
    this.stopAtEnd = stopAtEnd;
    addRequirements(this.drive);
  }

  public AutonomousFollower(Drive drive, boolean generatePath, boolean record, int rowOffset){
    this.drive = drive;
    this.rowOffset = rowOffset;
    this.record = record;
    this.generatePath = generatePath;
    this.commands = new JSONArray();
    this.stopAtEnd = true;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.endPathTime = path.getJSONArray(path.length() - 1).getDouble(0);
    if(generatePath == true) {
      this.fieldSide = drive.getFieldSide();
        while (drive.getNavxAngle() <= -180.0){
          drive.setNavxAngle(360.0);
        }
        if (drive.getNavxAngle() <= 0){
          drive.setNavxAngle(180.0);
        } else {
          drive.setNavxAngle(-180.0);
        }
      int row = drive.getClosestPlacementGroup(this.fieldSide, drive.getFusedOdometryX(), drive.getFusedOdometryY()) + this.rowOffset;
      this.path = drive.generatePlacementPathOnTheFly(row, this.fieldSide);
    }

    initTime = Timer.getFPGATimestamp();
  }

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

    if (this.record){
      recordedOdometry.add(new double[] {currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta});
    }
    // System.out.println("End Time: " + this.endPathTime);
    // System.out.println("Current Time: " + currentTime);
    if (Math.abs(this.endPathTime - currentTime) < 1){
      this.endingVelocities.add(velocityVector);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (this.stopAtEnd){
      Vector velocityVector = new Vector(0, 0);
      double desiredThetaChange = 0;
      drive.autoDrive(velocityVector, desiredThetaChange);
      drive.setWheelsStraight();
      System.out.println("Stopping at end");
    } else {
      System.out.println("Start end values: " + this.endingVelocities.toString());
      for (int i = 0; i < this.endingVelocities.size(); i ++){
        if (this.endingVelocities.get(i).magnitude() > 4){
          this.endingVelocities.remove(i);
          i --;
        }
      }
      double avgI = 0;
      double avgJ = 0;
      for (int i = 0; i < this.endingVelocities.size(); i ++){
        avgI += this.endingVelocities.get(i).getI();
        avgJ += this.endingVelocities.get(i).getJ();
      }
      avgI /= this.endingVelocities.size();
      avgJ /= this.endingVelocities.size();
      Vector endVector = new Vector(avgI, avgJ);
      drive.autoDrive(endVector, 0);
      System.out.println("End Values: " + this.endingVelocities.toString());
      System.out.println("End Vector: {" + endVector.getI() + ", " + endVector.getJ() + "}");
    }

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
          bw.write(line);
        }
        
        bw.close();
      } catch (Exception e) {
        System.out.println(e);
        System.out.println("CSV file error");
      }
    }
  }

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
