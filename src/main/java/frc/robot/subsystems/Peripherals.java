// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import org.json.JSONArray;
import org.json.JSONObject;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.PeripheralsDefault;
import frc.robot.sensors.Navx;

public class Peripherals extends SubsystemBase {
  private final static AHRS ahrs = new AHRS(Port.kMXP);

  private final static Navx navx = new Navx(ahrs);

  private Lights lights;

  private double limeLightHFOV = 59.6;

  private NetworkTable backLimeLightTable = NetworkTableInstance.getDefault().getTable("limelight-back");
  private NetworkTableEntry backTableX = backLimeLightTable.getEntry("tx");
  private NetworkTableEntry backTableY = backLimeLightTable.getEntry("ty");
  private NetworkTableEntry backTablePipelineLatency = backLimeLightTable.getEntry("tl");
  private NetworkTableEntry backTableCameraLatency = backLimeLightTable.getEntry("cl");
  private NetworkTableEntry backTableArea = backLimeLightTable.getEntry("ta");
  private NetworkTableEntry backRobotPose = backLimeLightTable.getEntry("botpose");
  private NetworkTableEntry backTagPose = backLimeLightTable.getEntry("targetpose_cameraspace");
  private NetworkTableEntry backJson = backLimeLightTable.getEntry("json");
  private NetworkTableEntry backTagToRobotPose = backLimeLightTable.getEntry("botpose_targetspace");

  private NetworkTable frontLimeLightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTableEntry frontTableX = frontLimeLightTable.getEntry("tx");
  private NetworkTableEntry frontTableY = frontLimeLightTable.getEntry("ty");
  private NetworkTableEntry frontTablePipelineLatency = frontLimeLightTable.getEntry("tl");
  private NetworkTableEntry frontTableCameraLatency = frontLimeLightTable.getEntry("cl");
  private NetworkTableEntry frontTableArea = frontLimeLightTable.getEntry("ta");
  private NetworkTableEntry frontRobotPose = frontLimeLightTable.getEntry("botpose");
  private NetworkTableEntry frontTagPose = frontLimeLightTable.getEntry("targetpose_cameraspace");
  private NetworkTableEntry frontJson = frontLimeLightTable.getEntry("json");
  private NetworkTableEntry frontTagToRobotPose = frontLimeLightTable.getEntry("botpose_targetspace");

  private double limeLightX = -1.0;
  private double limeLightY = -1.0;

  private double[] noTrackLimelightArray = new double[6];

  private double navxStartingRoll = 0;

  // static PhotonCamera camera = new PhotonCamera(Constants.CAMERA_NAME);

  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
  /** Creates a new Peripherals. */
  public Peripherals(Lights lights) {
   this.lights = lights;
  }

  public void init() {
    zeroNavx();
    noTrackLimelightArray[0] = 0;
    noTrackLimelightArray[1] = 0;
    noTrackLimelightArray[2] = 0;
    noTrackLimelightArray[3] = 0;
    noTrackLimelightArray[4] = 0;
    noTrackLimelightArray[5] = 0;
    setDefaultCommand(new PeripheralsDefault(this));
  }

  public String getFrontJSONString(){
    return frontJson.getString("");
  }

  public double getBackHorizontalDistToTag(){
    double[] pose = backTagToRobotPose.getDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
    return -pose[2];
  }

  public void setFrontPipeline(int pipeline){
    frontLimeLightTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void setBackPipeline(int pipeline){
    backLimeLightTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void setAprilTagPipeline(){
    backLimeLightTable.getEntry("pipeline").setNumber(0);
    frontLimeLightTable.getEntry("pipeline").setNumber(0);
  }

  public void setRetroreflectivePipeline(){
    backLimeLightTable.getEntry("pipeline").setNumber(1);
    frontLimeLightTable.getEntry("pipeline").setNumber(1);
  }

  public void setConeTrackingPipeline(){
    frontLimeLightTable.getEntry("pipeline").setNumber(2);
  }

  public int getFrontLimelightPipeline(){
    return (int) frontLimeLightTable.getEntry("pipeline").getInteger(5);
  }

  public int getBackLimelightPipeline(){
    return (int) backLimeLightTable.getEntry("pipeline").getInteger(5);
  }

  public double getFrontLimelightAngleToTarget(){
    return frontTableX.getDouble(0) * Math.PI / 180;
  }

  public double getBackLimelightAngleToTarget(){
    return backTableX.getDouble(0) * Math.PI / 180;
  }

  public double getBackTargetArea() {
    return backTableArea.getDouble(0);
  }

  public double getLimelightAngleToTarget(){
    double frontX = Math.PI * (frontTableX.getDouble(0)) / 180;
    double backX = Math.PI * (frontTableX.getDouble(0)) / 180;
    double frontArea = frontTableArea.getDouble(0);
    double backArea = backTableArea.getDouble(0);
    // System.out.println("Front: " + frontX + " Back: " + backX);
    if (frontArea > backArea){
      return frontX;
    } else {
      return backX;
    }
  }

  

  public JSONArray getLimelightBasedPosition(){
    JSONArray noTrack = new JSONArray();
    noTrack.put(0, 0.0);
    noTrack.put(1, 0.0);
    if (frontLimeLightTable.getEntry("pipeline").getInteger(5) != 0 && backLimeLightTable.getEntry("pipeline").getInteger(5) != 0){
      return noTrack;
    }
    try {
      double[] frontPose = frontRobotPose.getDoubleArray(noTrackLimelightArray);
      double[] backPose = backRobotPose.getDoubleArray(noTrackLimelightArray);
      double frontArea = frontTableArea.getDouble(0.0);
      double backArea = backTableArea.getDouble(0.0);
      double[] frontTargetPose = frontTagPose.getDoubleArray(noTrackLimelightArray);
      double[] backTargetPose = backTagPose.getDoubleArray(noTrackLimelightArray);
      JSONArray frontTargetFieldPose = new JSONArray(new double[] {0, 0});
      JSONArray backTargetFieldPose = new JSONArray(new double[] {0, 0});
      try {
        frontTargetFieldPose = new JSONObject(frontJson.getString("")).getJSONObject("Results").getJSONArray("Fiducial").getJSONObject(0).getJSONArray("t6r_fs");
      } catch(Exception e){}
      try {
        backTargetFieldPose = new JSONObject(backJson.getString("")).getJSONObject("Results").getJSONArray("Fiducial").getJSONObject(0).getJSONArray("t6r_fs");
      } catch(Exception e){}
        double frontTargetDist = Math.sqrt(Math.pow(frontTargetPose[0], 2) + Math.pow(frontTargetPose[1], 2));
      double backTargetDist = Math.sqrt(Math.pow(backTargetPose[0], 2) + Math.pow(backTargetPose[1], 2));
      int numTracksFront = 1;
      int numTracksBack = 1;
      if (Math.abs(frontTargetFieldPose.getDouble(0) - frontPose[0]) > 0.01 && Math.abs(frontTargetFieldPose.getDouble(1) - frontPose[1]) > 0.01){
        numTracksFront = 2;
      }
      if (Math.abs(backTargetFieldPose.getDouble(0) - backPose[0]) > 0.01 && Math.abs(backTargetFieldPose.getDouble(1) - backPose[1]) > 0.01){
        numTracksBack = 2;
      }
      JSONArray pose = new JSONArray(new double[] {0.0, 0.0});
      // System.out.println("Num Tracks:");
      // System.out.println(numTracksBack);
      // System.out.println("Megapose X: " + backPose[0]);
      // System.out.println("Tag X: " + backTargetFieldPose.getDouble(0));
      // System.out.println("Dif X: " + Math.abs(backTargetFieldPose.getDouble(0) - backPose[0]));
      
      if (frontArea > backArea){
        if (frontPose[0] < 1.0 && frontPose[1] < 0.5){
          return noTrack;
        }
        if (frontTargetDist > 5 && numTracksFront <= 1){
          System.out.println("Dist: " + frontTargetDist);
          return noTrack;
        }
        pose.put(0, frontPose[0] + Constants.FIELD_LENGTH / 2.0);
        pose.put(1, frontPose[1] + Constants.FIELD_WIDTH / 2.0);
      } else {
        if (backPose[0] < 1.0 && backPose[1] < 0.5){
          return noTrack;
        }
        if (backTargetDist > 5 && numTracksBack <= 1){
          return noTrack;
        }
        pose.put(0, backPose[0] + Constants.FIELD_LENGTH / 2.0);
        pose.put(1, backPose[1] + Constants.FIELD_WIDTH / 2.0);
      }
      return pose;
    } catch (Exception e){
      // System.out.println(e.toString());
      JSONArray noTarget = new JSONArray();
      noTarget.put(0);
      return noTarget;
    }
  }

  public double getBackLimeLightX() {
    limeLightX = Math.PI * (backTableX.getDouble(0))/180;
    return limeLightX;
  }

  public double getBackLimeLightY() {
    limeLightY = backTableY.getDouble(-100);
    return limeLightY;
  }

  public double getBackCameraLatency() {
    double latency = backTablePipelineLatency.getDouble(-1) + backTableCameraLatency.getDouble(-1);
    return latency;
  }

  public JSONArray getBackLimelightBasedPosition() {
    JSONArray fieldPosArray = new JSONArray();
    double[] result = new double[7];
    double tagDist = 99999;
    JSONArray noTrack = new JSONArray();
    noTrack.put(0);
    noTrack.put(0);
    try {
      result = backRobotPose.getDoubleArray(noTrackLimelightArray);
      tagDist = getBackHorizontalDistToTag();
    } catch (Exception e) {
      return noTrack;
    }
    if (tagDist > 2.25 || tagDist == 0){
      return noTrack;
    }
    if (result[0] == 0 || result[1] == 0){
      return noTrack;
    }
    double fieldX = result[0] + Constants.FIELD_LENGTH / 2;
    double fieldY = result[1] + Constants.FIELD_WIDTH / 2;
    fieldPosArray.put(0, fieldX);
    fieldPosArray.put(1, fieldY);
    System.out.println("Back X: " + fieldX + " Y: " + fieldY + " Dist: " + tagDist);
    return fieldPosArray;
  }

  public double getFrontLimeLightX() {
    limeLightX = Math.PI * (frontTableX.getDouble(0))/180;
    return limeLightX;
  }

  public double getFrontLimeLightY() {
    limeLightY = frontTableY.getDouble(-100);
    return limeLightY;
  }

  public double getFrontCameraLatency() {
    double latency = frontTablePipelineLatency.getDouble(-1) + frontTableCameraLatency.getDouble(-1);
    return latency;
  }

  public double getFrontTargetArea() {
    double area = frontTableArea.getDouble(-1);
    return area;
  }

  public JSONArray getFrontLimelightBasedPosition() {
    JSONArray robotPosArray = new JSONArray();
    robotPosArray.put(0, 0);
    try {
      String networkTableResult = frontRobotPose.getString("");
      JSONObject camResult = new JSONObject(networkTableResult).getJSONObject("Results");
      JSONArray tagList = camResult.getJSONArray("Fiducial");
      double averagedX = 0;
      double averagedY = 0;
      double count = 0;
      if(tagList.length() >= 1) {
        for(int i = 0; i < tagList.length(); i++) {
          JSONObject tag = (JSONObject) tagList.get(0);
          robotPosArray = tag.getJSONArray("t6r_fs");
          averagedX = averagedX + robotPosArray.getDouble(0);
          averagedY = averagedY + robotPosArray.getDouble(1);
          count++;
        }
        averagedX = averagedX/count;
        averagedY = averagedY/count;
        JSONObject bestTag = (JSONObject) tagList.get(0);
        robotPosArray.put(0, averagedX + (Constants.FIELD_LENGTH/2));
        robotPosArray.put(1, averagedY + (Constants.FIELD_WIDTH/2));
        // System.out.println(robotPosArray);
    } 
    } catch (Exception e) {
        JSONArray noTarget = new JSONArray();
        noTarget.put(0);
        return noTarget;
        // TODO: handle exception
    }
    return robotPosArray;
  }

  public double getNavxRollOffset() {
    if(navxStartingRoll == 0) {
      navxStartingRoll = getNavxRoll();
      if(navxStartingRoll == 0) {
        navxStartingRoll = 0.0001;
      }
      return navxStartingRoll;
    } else {
      double offset = getNavxRoll() - navxStartingRoll;
      return offset;
    }
  }

  public static double getNavxAngle() {
    // System.out.println("ANGLE: " + navx.getRawAngle() + " YAW: " + navx.getRawYaw() + " PITCH: " + navx.getRawPitch() + " ROLL: " + navx.getRawRoll());
    return navx.currentAngle();
}

  public double getRawNavxAngle() {
    return navx.getRawAngle();
  }

  public double getNavxAngleRotating() {
    return navx.currentAngle();
  }

  public void zeroNavx() {
    navx.softResetYaw();
    navx.softResetAngle();
}

  public double getNavxYaw(){
    return navx.currentYaw();
  }

  public double getNavxPitch(){
    return -navx.currentPitch();
  }

  public double getNavxRoll(){
    return navx.currentRoll();
  }

  public double getNavxRate() {
    return navx.getAngleRate();
  }

  public void setNavxAngle(double angle) {
    navx.setNavxAngle(angle);
  }

  // public double getLimeLightDistanceToTarget() {
  //   double yOffsetToTarget = getLimeLightYOffssetToTarget();
  //   if(yOffsetToTarget != -1.0) {
  //     double angleToTarget = (getLimeLightX());
  //     double xOffsetToTarget = yOffsetToTarget * (Math.tan(angleToTarget));
  //     double realDistanceToTarget = Math.sqrt((Math.pow(yOffsetToTarget, 2)) + (Math.pow(xOffsetToTarget, 2)));
  //     return realDistanceToTarget;
  //   }
  //   else {
  //     return -1.0;
  //   }    
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
