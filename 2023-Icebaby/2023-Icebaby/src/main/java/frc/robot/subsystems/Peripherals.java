// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.PeripheralsDefault;
import frc.robot.sensors.Navx;

public class Peripherals extends SubsystemBase {
  private final AHRS ahrs = new AHRS(Port.kMXP);

  private final Navx navx = new Navx(ahrs);

  private Lights lights;

  private double limeLightHFOV = 59.6;

  private NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tableX = limeLightTable.getEntry("tx");
  private NetworkTableEntry tableY = limeLightTable.getEntry("ty");
  private NetworkTableEntry switchLights = limeLightTable.getEntry("ledMode");

  private double limeLightX = -1.0;
  private double limeLightY = -1.0;


  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
  /** Creates a new Peripherals. */
  public Peripherals(Lights lights) {
   this.lights = lights;
  }

  public void init() {
    System.out.print("INSIDE PERIPHERALS INIT");
    zeroNavx();
    turnLightRingOn();
    setDefaultCommand(new PeripheralsDefault(this));
  }

  public double getLimeLightX() {
    limeLightX = Math.PI * (tableX.getDouble(0))/180;
    return limeLightX;
  }

  public double getLimeLightY() {
    limeLightY = tableY.getDouble(-100);
    return limeLightY;
  }

  public double getLimeLightYOffssetToTarget() {
    double x = getLimeLightY();
    if(x != -100) {
      double distance = 133.185 - (3.3797 * x) + (0.0484411 * Math.pow(x, 2)) + (-0.0025609 *  Math.pow(x, 3)) + (0.000100128 * Math.pow(x, 4));
      return distance;
    }
    return -1.0;
  }

  public void turnLightRingOn() {
    //m_pdh.setSwitchableChannel(true);
    switchLights.setNumber(3);
  }

  public void turnLightRingOff() {
    switchLights.setNumber(1);
  }

  public double getNavxAngle() {
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

  public double getLimeLightDistanceToTarget() {
    double yOffsetToTarget = getLimeLightYOffssetToTarget();
    if(yOffsetToTarget != -1.0) {
      double angleToTarget = (getLimeLightX());
      double xOffsetToTarget = yOffsetToTarget * (Math.tan(angleToTarget));
      double realDistanceToTarget = Math.sqrt((Math.pow(yOffsetToTarget, 2)) + (Math.pow(xOffsetToTarget, 2)));
      return realDistanceToTarget;
    }
    else {
      return -1.0;
    }    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
