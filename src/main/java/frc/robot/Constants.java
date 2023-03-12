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

    public static enum PRESET {
        HIGH_PLACEMENT,
        MID_PLACEMENT,
        UPRIGHT_CONE,
        TIPPED_CONE,
        CUBE,
        SHELF
    }

    // FRONT SIDE (battery), extensions are same on both sides
    // High row placement
    public static final double HIGH_PLACEMENT_FRONTSIDE_ARM_ROTATION = 135.2;
    public static final double HIGH_PLACEMENT_FRONTSIDE_WRIST_ROTATION = 79;
    public static final double HIGH_PLACEMENT_ARM_EXTENSION = 40;

    // Mid row placement
    public static final double MID_PLACEMENT_FRONTSIDE_ARM_ROTATION = 138.7;
    public static final double MID_PLACEMENT_FRONTSIDE_WRIST_ROTATION = 79.9;
    public static final double MID_PLACEMENT_ARM_EXTENSION = 18;

    // Upright cone intaking
    public static final double UPRIGHT_CONE_FRONTSIDE_ARM_ROTATION = 89.7;
    public static final double UPRIGHT_CONE_FRONTSIDE_WRIST_ROTATION = 152;

    // Tipped cone intaking
    public static final double TIPPED_CONE_FRONTSIDE_ARM_ROTATION = 66.0;
    public static final double TIPPED_CONE_FRONTSIDE_WRIST_ROTATION = 186.2;

    // Cube intaking
    public static final double CUBE_FRONTSIDE_ARM_ROTATION = 92.9;
    public static final double CUBE_FRONTSIDE_WRIST_ROTATION = 124.8;

    // Shelf intaking
    public static final double SHELF_FRONTSIDE_ARM_ROTATION = 161;
    public static final double SHELF_FRONTSIDE_WRIST_ROTATION = 68;
    public static final double SHELF_ARM_EXTENSION = 18.7;

    // BACK SIDE (worm gear)
    // High row placement
    public static final double HIGH_PLACEMENT_BACKSIDE_ARM_ROTATION = 223.3;
    public static final double HIGH_PLACEMENT_BACKSIDE_WRIST_ROTATION = 280;

    // Mid row placement
    public static final double MID_PLACEMENT_BACKSIDE_ARM_ROTATION = 219.3;
    public static final double MID_PLACEMENT_BACKSIDE_WRIST_ROTATION = 280.3;

    // Upright cone intaking
    public static final double UPRIGHT_CONE_BACKSIDE_ARM_ROTATION = 267.7;
    public static final double UPRIGHT_CONE_BACKSIDE_WRIST_ROTATION = 207;

    // Tipped cone intaking
    public static final double TIPPED_CONE_BACKSIDE_ARM_ROTATION = 292.2;
    public static final double TIPPED_CONE_BACKSIDE_WRIST_ROTATION = 175.4;

    // Cube intaking
    public static final double CUBE_BACKSIDE_ARM_ROTATION = 264.6;
    public static final double CUBE_BACKSIDE_WRIST_ROTATION = 235;

    // Shelf intaking
    public static final double SHELF_BACKSIDE_ARM_ROTATION = 198;
    public static final double SHELF_BACKSIDE_WRIST_ROTATION = 292;


    public static final double PLACEMENT_LOCATION_X_RED = 14.75;
    public static final double PLACEMENT_LOCATION_X_BLUE = FIELD_LENGTH - PLACEMENT_LOCATION_X_RED;
    public static final double[] PLACEMENT_LOCATIONS_Y_RED = new double[] {0.52, 1.07, 1.62, 2.2, 2.74, 3.3, 3.85, 4.43, 4.98};
    public static final double[] PLACEMENT_LOCATIONS_Y_BLUE = new double[] {4.98, 4.43, 3.85, 3.3, 2.74, 2.2, 1.07, 0.52};

    public static final double PLACEMENT_PATH_MIDPOINT_X_RED = 14.41;
    public static final double PLACEMENT_PATH_MIDPOINT_X_BLUE = FIELD_LENGTH - PLACEMENT_PATH_MIDPOINT_X_RED;
    public static final double[] PLACEMENT_PATH_MIDPOINT_Y_RED = new double[] {0.52, 1.07, 1.62, 2.2, 2.74, 3.3, 3.85, 4.43, 4.98};
    public static final double[] PLACEMENT_PATH_MIDPOINT_Y_BLUE = new double[] {4.98, 4.43, 3.85, 3.3, 2.74, 2.2, 1.07, 0.52};

    // 3D locations of all 8 april tags {x, y, z} in meters
    public static final double[][] TAG_LOCATIONS = new double[][] {
        {15.513558, 1.071626, 0.462788},
        {15.513558, 2.748026, 0.462788},
        {15.513558, 4.424426, 0.462788},
        {16.178784, 6.749796, 0.695452},
        {0.36195, 6.749796, 0.695452},
        {1.02743, 4.4224426, 0.462788},
        {1.02743, 2.748026, 0.462788},
        {1.02743, 1.071626, 0.462788},
    };

    public static final double EXTENSION_GEAR_RATIO = 10.13;
    public static final double EXTENSION_INCHES_PER_ROTATION = 1.5038 * Math.PI;
    public static final double MAX_EXTENSION = 40;

    // max of falcon with regular firmware = 6380 rpm, with phoenix pro ~ 6000 rpm, converting to rps
    public static final double MAX_FALCON_ROTATIONS_PER_SECOND = 75;
    // max acceleration of a falcon - calculated by deciding that we need to reach max speed in 1 second
    public static final double MAX_FALCON_ROTATIONS_PER_SECOND_PER_SECOND = 200;
    // max jerk of a falcon - calculated by deciding that we need to reach max acceleration in 1 second
    public static final double MAX_FALCON_ROTATIONS_PER_SECOND_PER_SECOND_PER_SECOND = 8000;

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

}
