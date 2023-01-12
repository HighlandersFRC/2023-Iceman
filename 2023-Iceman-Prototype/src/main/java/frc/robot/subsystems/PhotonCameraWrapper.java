/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.subsystems;

 import edu.wpi.first.apriltag.AprilTag;
 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.math.Pair;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.wpilibj.Timer;
 import frc.robot.Constants;
 import java.util.ArrayList;
 import java.util.Optional;
 import org.photonvision.PhotonCamera;
 import org.photonvision.RobotPoseEstimator;
 import org.photonvision.RobotPoseEstimator.PoseStrategy;
 import java.util.*;
 
 public class PhotonCameraWrapper {
     public PhotonCamera photonCamera;
     public RobotPoseEstimator robotPoseEstimator;

     private AprilTagFieldLayout aprilTagFieldLayout;

 
     public PhotonCameraWrapper() {
         // Set up a test arena of two apriltags at the center of each driver station set
        final AprilTag tag1 = new AprilTag(1, new Pose3d(15.514, 1.072, 0.463, new Rotation3d(0, 0, Math.toRadians(180))));
		final AprilTag tag2 = new AprilTag(2, new Pose3d(15.514, 2.748, 0.463, new Rotation3d(0, 0, Math.toRadians(180))));
		final AprilTag tag3 = new AprilTag(3, new Pose3d(15.514, 4.424, 0.463, new Rotation3d(0, 0, Math.toRadians(180))));
		final AprilTag tag4 = new AprilTag(4, new Pose3d(16.179, 6.750, 0.695, new Rotation3d(0, 0, Math.toRadians(180))));
		final AprilTag tag5 = new AprilTag(5, new Pose3d(0.362, 6.750, 0.695, new Rotation3d(0, 0, 0)));
		final AprilTag tag6 = new AprilTag(6, new Pose3d(1.027, 4.424, 0.463, new Rotation3d(0, 0, 0)));
		final AprilTag tag7 = new AprilTag(7, new Pose3d(1.027, 2.748, 0.463, new Rotation3d(0, 0, 0)));
		final AprilTag tag8 = new AprilTag(8, new Pose3d(1.027, 1.072, 0.463, new Rotation3d(0, 0, 0)));

        ArrayList<AprilTag> TagList = new ArrayList<AprilTag>();
		TagList.add(tag1);
		TagList.add(tag2);
		TagList.add(tag3);
		TagList.add(tag4);
		TagList.add(tag5);
		TagList.add(tag6);
		TagList.add(tag7);
		TagList.add(tag8);
 
        // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
        AprilTagFieldLayout atfl =
                new AprilTagFieldLayout(TagList, Constants.feetToMeters(54), Constants.feetToMeters(27));
 
        // Forward Camera
        photonCamera =
                 new PhotonCamera(
                         Constants
                                 .CAMERA_NAME); // Change the name of your camera here to whatever it is in the
         // PhotonVision UI.
 
         // ... Add other cameras here
 
         // Assemble the list of cameras & mount locations
         var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
         camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, Constants.ROBOT_TO_CAM));
 
         robotPoseEstimator =
                 new RobotPoseEstimator(atfl, PoseStrategy.AVERAGE_BEST_TARGETS, camList);
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
      *     of the observation. Assumes a planar field and the robot is always firmly on the ground
      */
     public Pair<Pose2d, Double> getEstimatedGlobalPose() {
        //  robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
 
         double currentTime = Timer.getFPGATimestamp();
         Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
         if (result.isPresent()) {
             return new Pair<Pose2d, Double>(
                     result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
         } else {
             return new Pair<Pose2d, Double>(null, 0.0);
         }
     }
 }