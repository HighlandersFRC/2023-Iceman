// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import com.ctre.phoenix.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Peripherals;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double FIELD_WIDTH = 8.2;
    public static final double FIELD_LENGTH = 16.63;

    public static final double ROBOT_RADIUS = inchesToMeters(15.375); //m

    public static final double WHEEL_DIAMETER = inchesToMeters(4); //m

    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; // m

    // number found through experimentation at top speed, official speed is around 16.3 f/sec
    public static final double TOP_SPEED = feetToMeters(20); // m/sec

    public static final double GEAR_RATIO = 6.75;

    public static final double STEER_GEAR_RATIO = 21.43;

    public static final double FALCON_TICS_PER_ROTATION = 2048;

    public static final double CANCODER_TICS_PER_ROTATION = 4096;

    public static final double NEO_TICS_PER_ROTATION = 42;

    public static final double ROBOT_WIDTH = inchesToMeters(25); //m

    public static final double ROBOT_LENGTH = inchesToMeters(29); //m

    public static final double MODULE_OFFSET = inchesToMeters(2.5);

    public static final String CAMERA_NAME = "Limelight";

    public static double wristOffsetMidMatch = 0;

    public static double wristOffsetDiffArm = 0;

    public static enum PRESET {
        HIGH_PLACEMENT,
        MID_PLACEMENT,
        LOW_PLACEMENT,
        UPRIGHT_CONE,
        TIPPED_CONE,
        CUBE,
        SHELF
    }

    // FRONT SIDE (battery), extensions are same on both sides
    // High row placement
    public static final double HIGH_PLACEMENT_FRONTSIDE_ARM_ROTATION = 135;
    public static final double HIGH_PLACEMENT_FRONTSIDE_WRIST_ROTATION = 86 + wristOffsetDiffArm;
    public static final double HIGH_PLACEMENT_ARM_EXTENSION = 36.5;

    // Mid row placement
    public static final double MID_PLACEMENT_FRONTSIDE_ARM_ROTATION = 138.5;
    public static final double MID_PLACEMENT_FRONTSIDE_WRIST_ROTATION = 79 + wristOffsetDiffArm;
    public static final double MID_PLACEMENT_ARM_EXTENSION = 17.5;

    // low placement
    public static final double LOW_PLACEMENT_FRONTSIDE_WRIST_ROTATION = 62 + wristOffsetDiffArm;
    public static final double LOW_PLACEMENT_FRONTSIDE_ARM_ROTATION = 150;

    // Upright cone intaking
    public static final double UPRIGHT_CONE_FRONTSIDE_ARM_ROTATION = 101;
    public static final double UPRIGHT_CONE_FRONTSIDE_WRIST_ROTATION = 120 + wristOffsetDiffArm;

    // Tipped cone intaking
    public static final double TIPPED_CONE_FRONTSIDE_ARM_ROTATION = 69.5;
    public static final double TIPPED_CONE_FRONTSIDE_WRIST_ROTATION = 183 + wristOffsetDiffArm;

    // Cube intaking
    public static final double CUBE_FRONTSIDE_ARM_ROTATION = 93.2;
    public static final double CUBE_FRONTSIDE_WRIST_ROTATION = 118 + wristOffsetDiffArm;

    // Shelf intaking
    public static final double SHELF_FRONTSIDE_ARM_ROTATION = 162.5;
    public static final double SHELF_FRONTSIDE_WRIST_ROTATION = 66 + wristOffsetDiffArm;
    public static final double SHELF_ARM_EXTENSION = 17.5;

    // BACK SIDE (worm gear)
    // High row placement
    public static final double HIGH_PLACEMENT_BACKSIDE_ARM_ROTATION = 223.5;
    public static final double HIGH_PLACEMENT_BACKSIDE_WRIST_ROTATION = 273 + wristOffsetDiffArm;

    // Mid row placement
    public static final double MID_PLACEMENT_BACKSIDE_ARM_ROTATION = 220.8;
    public static final double MID_PLACEMENT_BACKSIDE_WRIST_ROTATION = 281 + wristOffsetDiffArm;

    // low placement
    public static final double LOW_PLACEMENT_BACKSIDE_WRIST_ROTATION = 290 + wristOffsetDiffArm;
    public static final double LOW_PLACEMENT_BACKSIDE_ARM_ROTATION = 210;

    // Upright cone intaking
    public static final double UPRIGHT_CONE_BACKSIDE_ARM_ROTATION = 263;
    public static final double UPRIGHT_CONE_BACKSIDE_WRIST_ROTATION = 231 + wristOffsetDiffArm;

    // Tipped cone intaking
    public static final double TIPPED_CONE_BACKSIDE_ARM_ROTATION = 290;
    public static final double TIPPED_CONE_BACKSIDE_WRIST_ROTATION = 175.4 + wristOffsetDiffArm;

    // Cube intaking
    public static final double CUBE_BACKSIDE_ARM_ROTATION = 268.5;
    public static final double CUBE_BACKSIDE_WRIST_ROTATION = 241 + wristOffsetDiffArm;

    // Shelf intaking
    public static final double SHELF_BACKSIDE_ARM_ROTATION = 199;
    public static final double SHELF_BACKSIDE_WRIST_ROTATION = 296 + wristOffsetDiffArm;


    public static final double PLACEMENT_LOCATION_X_RED = 14.75;
    public static final double PLACEMENT_LOCATION_X_BLUE = FIELD_LENGTH - PLACEMENT_LOCATION_X_RED;
    public static final double[] PLACEMENT_LOCATIONS_Y_RED = new double[] {0.52, 1.07, 1.62, 2.2, 2.74, 3.3, 3.85, 4.43, 4.98};
    public static final double[] PLACEMENT_LOCATIONS_Y_BLUE = new double[] {4.98, 4.43, 3.85, 3.3, 2.74, 2.2, 1.07, 0.52};

    public static final double PLACEMENT_PATH_MIDPOINT_X_RED = 14.41;
    public static final double PLACEMENT_PATH_MIDPOINT_X_BLUE = FIELD_LENGTH - PLACEMENT_PATH_MIDPOINT_X_RED;
    public static final double[] PLACEMENT_PATH_MIDPOINT_Y_RED = new double[] {0.52, 1.07, 1.62, 2.2, 2.74, 3.3, 3.85, 4.43, 4.98};
    public static final double[] PLACEMENT_PATH_MIDPOINT_Y_BLUE = new double[] {4.98, 4.43, 3.85, 3.3, 2.74, 2.2, 1.07, 0.52};

    //Poses of all 16 AprilTags, {x, y, z, theta}, in meters and radians
    public static final double[][] TAG_POSES = {
        {15.079502159004317, 0.2458724917449835, 1.3558547117094235, 2.0943951023931953},
        {16.18516637033274, 0.8836677673355348, 1.3558547117094235, 2.0943951023931953},  
        {16.57937515875032, 4.982727965455931, 1.4511049022098046, 3.141592653589793},    
        {16.57937515875032, 5.547879095758192, 1.4511049022098046, 3.141592653589793},    
        {14.700787401574804, 8.204216408432817, 1.3558547117094235, 4.71238898038469},    
        {1.841503683007366, 8.204216408432817, 1.3558547117094235, 4.71238898038469},     
        {-0.038100076200152405, 5.547879095758192, 1.4511049022098046, 0.0},
        {-0.038100076200152405, 4.982727965455931, 1.4511049022098046, 0.0},
        {0.35610871221742446, 0.8836677673355348, 1.3558547117094235, 1.0471975511965976},
        {1.4615189230378463, 0.2458724917449835, 1.3558547117094235, 1.0471975511965976}, 
        {11.90474980949962, 3.713233426466853, 1.3208026416052834, 5.235987755982989},    
        {11.90474980949962, 4.4983489966979935, 1.3208026416052834, 1.0471975511965976},  
        {11.220218440436883, 4.105156210312421, 1.3208026416052834, 3.141592653589793},   
        {5.320802641605283, 4.105156210312421, 1.3208026416052834, 0.0},
        {4.641351282702566, 4.4983489966979935, 1.3208026416052834, 2.0943951023931953},  
        {4.641351282702566, 3.713233426466853, 1.3208026416052834, 4.1887902047863905}
    };

    //Field of view angles
    public static final double LIMELIGHT_HFOV_DEG = 63.3;
    public static final double LIMELIGHT_VFOV_DEG = 49.7;
    public static final double LIMELIGHT_HFOV_RAD = LIMELIGHT_HFOV_DEG * Math.PI / 180;
    public static final double LIMELIGHT_VFOV_RAD = LIMELIGHT_VFOV_DEG * Math.PI / 180;

    //Poses of cameras relative to robot, {x, y, z, rx, ry, rz}, in meters and radians
    public static final double[] BACK_CAMERA_POSE = {-0.219075, -0.1524, 0.1524, 0, 0, Math.PI};
    public static final double[] FRONT_CAMERA_POSE = {0.219075, 0.1524, 0.288925, 0, 33 * Math.PI / 180, 0};
    public static final double[] BACK_CAMERA_POSITION_POLAR = {getDistance(0, 0, BACK_CAMERA_POSE[0], BACK_CAMERA_POSE[1]), Math.atan2(BACK_CAMERA_POSE[1], BACK_CAMERA_POSE[0])};
    public static final double[] FRONT_CAMERA_POSITION_POLAR = {getDistance(0, 0, FRONT_CAMERA_POSE[0], FRONT_CAMERA_POSE[1]), Math.atan2(FRONT_CAMERA_POSE[1], FRONT_CAMERA_POSE[0])};

    public static final double EXTENSION_GEAR_RATIO = 10.13;
    public static final double EXTENSION_INCHES_PER_ROTATION = 1.5038 * Math.PI;
    public static final double MAX_EXTENSION = 40;

    public static final double SIDE_INTAKE_GEAR_RATIO = 75;

    // max of falcon with regular firmware = 6380 rpm, with phoenix pro ~ 6000 rpm, converting to rps
    public static final double MAX_FALCON_ROTATIONS_PER_SECOND = 80;
    // max acceleration of a falcon - calculated by deciding that we need to reach max speed in 1 second
    public static final double MAX_FALCON_ROTATIONS_PER_SECOND_PER_SECOND = 200;
    // max jerk of a falcon - calculated by deciding that we need to reach max acceleration in 1 second
    public static final double MAX_FALCON_ROTATIONS_PER_SECOND_PER_SECOND_PER_SECOND = 2000;

    public static final double MAX_FALCON_TICS_PER_SECOND = (25 * FALCON_TICS_PER_ROTATION)/10; // raw sensor units/100 ms
    
    public static final double MAX_FALCON_TICS_PER_SECOND_PER_SECOND = (75 * FALCON_TICS_PER_ROTATION)/10; // raw sensor units/100 ms

    public static final double MAX_FALCON_TICS_PER_SECOND_PER_SECOND_PER_SECOND = (MAX_FALCON_ROTATIONS_PER_SECOND_PER_SECOND_PER_SECOND * FALCON_TICS_PER_ROTATION)/10; // raw sensor units/100 ms

    public static final double TOTAL_TICS_PER_INCH = (2.0 * ((1.0 / FALCON_TICS_PER_ROTATION) / EXTENSION_GEAR_RATIO) * EXTENSION_INCHES_PER_ROTATION);

    public static void increaseWristOffset() {
        wristOffsetMidMatch += 3.75;
    }

    public static void decreaseWristOffset() {
        wristOffsetMidMatch -= 3.75;
    }

    public static double convertArmRotationDegreesToTics(double degrees) {
        return (CANCODER_TICS_PER_ROTATION * degrees)/360;
    }

    public static double convertArmRotationTicsToDegrees(double tics) {
        return (360 * tics)/CANCODER_TICS_PER_ROTATION;
    }

    public static double getArmExtensionInches(double tics) {
        return tics * TOTAL_TICS_PER_INCH;
    }

    public static double getArmExtensionTics(double inches) {
        return inches/TOTAL_TICS_PER_INCH;
    }

    public static double getArmExtensionRotations(double inches) {
        return getArmExtensionTics(inches)/FALCON_TICS_PER_ROTATION;
    }

    public static double getArmExtensionInchesFromRotations(double rotations) {
        return getArmExtensionInches(rotations) * FALCON_TICS_PER_ROTATION;
    }

    public static double getSideIntakeDegreesFromRotations(double rotations) {
        return 360 * (rotations/SIDE_INTAKE_GEAR_RATIO);
    }

    public static double getSideIntakeRotationsFromDegrees(double degrees) {
        return (degrees * SIDE_INTAKE_GEAR_RATIO)/360;
    }

    public static final Transform3d ROBOT_TO_CAM =
                new Transform3d(
                        new Translation3d(0.4064, 0.0, 0.3302),
                        new Rotation3d(0, 0,0));

    public static RealMatrix getXRotationMatrix(double radians) {
        double[][] XMatrixData = {{1, 0, 0},
                                  {0, Math.cos(radians), -Math.sin(radians)},
                                  {0, Math.sin(radians), Math.cos(radians)}};
        return MatrixUtils.createRealMatrix(XMatrixData);
    }

    public static RealMatrix getYRotationMatrix(double radians) {
        double[][] yMatrixData = {{Math.cos(radians), 0, Math.sin(radians)},
                                  {0, 1, 0},
                                  {-Math.sin(radians), 0, Math.cos(radians)}};
        return MatrixUtils.createRealMatrix(yMatrixData);
    }

    public static RealMatrix getZRotationMatrix(double radians) {
        double[][] ZMatrixData = {{Math.cos(radians), -Math.sin(radians), 0},
                                  {Math.sin(radians), Math.cos(radians), 0},
                                  {0, 0, 1}};
        return MatrixUtils.createRealMatrix(ZMatrixData);
    }

    public static RealMatrix getRotationMatrix(double xRot, double yRot, double zRot) {
        RealMatrix xRotation = getXRotationMatrix(xRot);//should never exist unles tag is rotated on surface its mounted to
        RealMatrix yRotation = getYRotationMatrix(yRot);//when surface tag is mounted to is not perpendigular to ground
        RealMatrix zRotation = getZRotationMatrix(zRot);//when the tag is not directly infront of the robot, will chage most often

        RealMatrix xyzRotation = xRotation.multiply(yRotation).multiply(zRotation);        
        return xyzRotation;
    }
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double feetToMeters(double feet) {
        double inches = feet * 12;
        return inchesToMeters(inches);
    }

    public static int shooterRPMToUnitsPer100MS(double rpm) {
        return (int) (Math.round((rpm / 600.0) * FALCON_TICS_PER_ROTATION));
    }

    public static double unitsPer100MsToRPM(double units) {
        return (units * 600) / (Constants.FALCON_TICS_PER_ROTATION);
    }

    public static double getDistance(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
}
