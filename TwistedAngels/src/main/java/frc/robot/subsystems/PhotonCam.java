package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PhotonCam extends SubsystemBase { 
      PhotonCamera cam ;
          
      // The field from AprilTagFields will be different depending on the game.
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Construct PhotonPoseEstimator
  PhotonPoseEstimator photonPoseEstimator;

  public PhotonCam (String camera, Transform3d robotToCam)  {
    cam = new PhotonCamera(camera);
   photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);
  }
   public void periodic (){
    Optional <EstimatedRobotPose> o = photonPoseEstimator.update();
    if (o.isPresent()) {
      EstimatedRobotPose e = o.get();
    
      //Display on map
      RobotContainer.s_Swerve.field.getObject(cam.getName()).setPose(e.estimatedPose.toPose2d());
    }
   }
      
      
}