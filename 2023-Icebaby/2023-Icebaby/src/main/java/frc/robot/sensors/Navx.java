// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.sensors;

// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Navx {
//   private double originalAngle;
//   private double originalYaw;
//   private double originalPitch;
//   private AHRS imu;
//   /** Creates a new Navx. */
 
//     public Navx(AHRS navx) {
//       imu = navx;
//       originalAngle = imu.getAngle();
//       originalYaw = imu.getYaw();
//   }

//   public Navx(AHRS navx, Double startAngle) {
//       imu = navx;
//       originalAngle = startAngle;
//       originalYaw = imu.getYaw();
//   }

//   public double currentAngle() {
//         double raw = imu.getAngle()%360;
//         SmartDashboard.putNumber("RAW", raw);
//         SmartDashboard.putNumber("OFFSET", originalAngle);
//         double angle = (-(raw - originalAngle));
//         SmartDashboard.putNumber("Original", angle);
//         if(angle > 360) {
//             angle = angle%360;
//         }
//         else if (angle < -360) {
//             angle = (angle%360) - 360;
//         }

//         if(angle < 0) {
//             angle = 360 + angle;
//         }
//         SmartDashboard.putNumber("ANGLE", angle);
//         return angle;
//   }

//   public double getRawAngle() {
//       return -(imu.getAngle());
//   }

//   public double getRawPitch() {
//       return imu.getPitch();
//   }

//   public double getRawYaw() {
//       return imu.getYaw();
//   }

//   public double getRawRoll() {
//       return imu.getRoll();
//   }

//   public double currentPitch() {
//       return imu.getPitch() - originalPitch;
//   }

//   public double currentRoll() {
//       return imu.getRoll();
//   }

//   public double currentYaw() {
//       return (-((imu.getYaw() - originalYaw)));
//   }

//   public boolean isMoving() {
//       return imu.isMoving();
//   }
    
//   public double currentAccelerometerX() {
//       return imu.getWorldLinearAccelX();
//   }

//   public double currentAccelerometerY() {
//       return imu.getWorldLinearAccelY();
//   }

//   public double currentAccelerometerZ() {
//       return imu.getWorldLinearAccelZ();
//   }

//   public boolean isOn() {
//       return imu.isConnected();
//   }

//   public boolean isMagCalibrated() {
//       return imu.isMagnetometerCalibrated();
//   }

//   public boolean isAutoCalibrating() {
//       return imu.isCalibrating();
//   }

//   public boolean isMagInerference() {
//       return imu.isMagneticDisturbance();
//   }

//   public void softResetAngle() {
//       originalAngle = imu.getAngle();
//   }

//   public void setNavxAngle(double angle) {
//       originalYaw = originalYaw + angle;
//   }

//   public void softResetYaw() {
//       originalYaw = imu.getYaw();
//   }

//   public void softResetPitch(){
//       originalPitch = imu.getPitch();
//   }

//   public double getAngleRate() {
//       return (imu.getRate());
//   }
  
// }


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Navx {
  private double originalAngle;
  private double originalYaw;
  private AHRS imu;
  /** Creates a new Navx. */
 
    public Navx(AHRS navx) {
      imu = navx;
      originalAngle = imu.getAngle();
      originalYaw = imu.getYaw();
  }

  public Navx(AHRS navx, Double startAngle) {
      imu = navx;
      originalAngle = startAngle;
      originalYaw = imu.getYaw();
  }

  public double currentAngle() {
        SmartDashboard.putNumber("Angle", -(imu.getAngle() - originalAngle));
        return -(imu.getAngle() - originalAngle);
  }

  public double getRawAngle() {
      return -(imu.getAngle());
  }

  public double getRawPitch() {
      return imu.getPitch();
  }

  public double getRawYaw() {
      return imu.getYaw();
  }

  public double getRawRoll() {
      return imu.getRoll();
  }

  public double currentPitch() {
      return imu.getPitch();
  }

  public double currentRoll() {
      return imu.getRoll();
  }

  public double currentYaw() {
      return -((imu.getYaw()) - originalYaw);
  }

  public boolean isMoving() {
      return imu.isMoving();
  }
    
  public double currentAccelerometerX() {
      return imu.getWorldLinearAccelX();
  }

  public double currentAccelerometerY() {
      return imu.getWorldLinearAccelY();
  }

  public double currentAccelerometerZ() {
      return imu.getWorldLinearAccelZ();
  }

  public boolean isOn() {
      return imu.isConnected();
  }

  public boolean isMagCalibrated() {
      return imu.isMagnetometerCalibrated();
  }

  public boolean isAutoCalibrating() {
      return imu.isCalibrating();
  }

  public boolean isMagInerference() {
      return imu.isMagneticDisturbance();
  }

  public void softResetAngle() {
      originalAngle = imu.getAngle();
  }

  public void setNavxAngle(double angle) {
      originalAngle = originalAngle + angle;
  }

  public void softResetYaw() {
      originalYaw = imu.getYaw();
  }

  public double getAngleRate() {
      return (imu.getRate());
  }
  
}