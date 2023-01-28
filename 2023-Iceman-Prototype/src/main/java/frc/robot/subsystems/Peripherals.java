// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTableEntry tableX = limeLightTable.getEntry("tx");
  private NetworkTableEntry tableY = limeLightTable.getEntry("ty");
  private NetworkTableEntry tableLatency = limeLightTable.getEntry("tl");
  private NetworkTableEntry switchLights = limeLightTable.getEntry("ledMode");
  private NetworkTableEntry robotPose = limeLightTable.getEntry("json");
  private NetworkTableEntry botPose = limeLightTable.getEntry("botpose");

  private double limeLightX = -1.0;
  private double limeLightY = -1.0;

  private double[] noTrackLimelightArray = new double[6];

  // static PhotonCamera camera = new PhotonCamera(Constants.CAMERA_NAME);

  static PhotonCameraWrapper camera = new PhotonCameraWrapper();

  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
  /** Creates a new Peripherals. */
  public Peripherals(Lights lights) {
   this.lights = lights;
  }

  public void init() {
    System.out.print("INSIDE PERIPHERALS INIT");
    zeroNavx();
    turnLightRingOn();
    noTrackLimelightArray[0] = 0;
    noTrackLimelightArray[1] = 0;
    noTrackLimelightArray[2] = 0;
    noTrackLimelightArray[3] = 0;
    noTrackLimelightArray[4] = 0;
    noTrackLimelightArray[5] = 0;
    setDefaultCommand(new PeripheralsDefault(this));
  }

  public Pose2d getCameraBasedRobotLocation() {
    return camera.getEstimatedGlobalPose().getFirst();
  }

  // public boolean cameraHasTargets() {
  //   PhotonPipelineResult cameraResult = camera.getLatestResult();
  //   return cameraResult.hasTargets();
  // }

  // public static double cameraYawToTarget() {
  //   PhotonPipelineResult cameraResult = camera.getLatestResult();
  //   double targetYaw = 0.0;
  //   if(cameraResult.hasTargets() == true) {
  //     PhotonTrackedTarget target = cameraResult.getBestTarget();
  //     targetYaw = target.getYaw();
  //   }
  //   return targetYaw;
  // }

  // public static double cameraPitchToTarget() {
  //   PhotonPipelineResult cameraResult = camera.getLatestResult();
  //   double targetPitch = 0.0;
  //   if(cameraResult.hasTargets() == true) {
  //     PhotonTrackedTarget target = cameraResult.getBestTarget();
  //     targetPitch = target.getPitch();
  //   }
  //   return targetPitch;
  // }

  // public static Transform3d cameraToTarget() {
  //   PhotonPipelineResult cameraResult = camera.getLatestResult();
  //   Transform3d cameraToTarget = new Transform3d();
  //   if(cameraResult.hasTargets() == true) {
  //     PhotonTrackedTarget target = cameraResult.getBestTarget();
  //     cameraToTarget = target.getBestCameraToTarget();
  //     Rotation3d test = target.getBestCameraToTarget().getRotation();
  //     System.out.println("ROTATION: " + test);
  //   }
  //   return cameraToTarget;
  // }

  // public static double getTargetID() {
  //   PhotonPipelineResult cameraResult = camera.getLatestResult();
  //   if(cameraResult.hasTargets() == true) {
  //     PhotonTrackedTarget target = cameraResult.getBestTarget();
  //     return target.getFiducialId();
  //   }
  //   return -1;    
  // }

  public double getLimeLightX() {
    limeLightX = Math.PI * (tableX.getDouble(0))/180;
    return limeLightX;
  }

  public double getLimeLightY() {
    limeLightY = tableY.getDouble(-100);
    return limeLightY;
  }

  public double getCameraLatency() {
    double latency = tableLatency.getDouble(-1);
    return latency;
  }

  // public JSONArray getLimelightBasedPosition() {
  //   // System.out.println(robotPose.getString(""));
  //   JSONArray robotPosArray = new JSONArray();
  //   robotPosArray.put(0, 0);
  //   try {
  //     String networkTableResult = robotPose.getString("");
  //     JSONObject camResult = new JSONObject(networkTableResult).getJSONObject("Results");
  //     JSONArray botPose = camResult.getJSONArray("botpose");
  //       robotPosArray.put(0, botPose.getDouble(0) + (Constants.FIELD_LENGTH/2));
  //       robotPosArray.put(1, botPose.getDouble(1) + (Constants.FIELD_WIDTH/2));
  //       // System.out.println(robotPosArray);
  //   } catch (Exception e) {
  //       JSONArray noTarget = new JSONArray();
  //       noTarget.put(0);
  //       return noTarget;
  //       // TODO: handle exception
  //   }
  //   return robotPosArray;
  // }

  public void getBotPose(){
    try {
      String result = botPose.getString("");
      System.out.println(result);
    } catch (Exception e){
      System.out.println(e);
    }
  }

  public JSONArray getLimelightBasedPosition() {
    // System.out.println(robotPose.getString(""));
    JSONArray robotPosArray = new JSONArray();
    robotPosArray.put(0, 0);
    try {
      String networkTableResult = robotPose.getString("");
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

  // public double getLimeLightYOffssetToTarget() {
  //   double x = getLimeLightY();
  //   if(x != -100) {
  //     double distance = 133.185 - (3.3797 * x) + (0.0484411 * Math.pow(x, 2)) + (-0.0025609 *  Math.pow(x, 3)) + (0.000100128 * Math.pow(x, 4));
  //     return distance;
  //   }
  //   return -1.0;
  // }

  public void turnLightRingOn() {
    //m_pdh.setSwitchableChannel(true);
    switchLights.setNumber(3);
  }

  public void turnLightRingOff() {
    switchLights.setNumber(1);
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
