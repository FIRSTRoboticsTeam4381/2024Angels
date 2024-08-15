package frc.robot.subsystems;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PhotonCam extends SubsystemBase { 
  PhotonCamera cam ;
  StructPublisher<Pose3d> publisher;
          
      // The field from AprilTagFields will be different depending on the game.
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Construct PhotonPoseEstimator
  PhotonPoseEstimator photonPoseEstimator;

  public PhotonCam (String camera, Transform3d robotToCam)  {
    cam = new PhotonCamera(camera);
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);

    publisher = NetworkTableInstance.getDefault().getStructTopic(camera, Pose3d.struct).publish();

    SmartDashboard.putNumber("Photon XY Confidence", 0);
    SmartDashboard.putNumber("Photon R Confidence", 0);
    SmartDashboard.putBoolean("Photon use calculated", false);
  }

  public void periodic (){
  Optional <EstimatedRobotPose> o = photonPoseEstimator.update();
    if (o.isPresent()) {
      EstimatedRobotPose e = o.get();
    
      publisher.set(e.estimatedPose);

      //Display on map
      RobotContainer.s_Swerve.field.getObject(cam.getName()).setPose(e.estimatedPose.toPose2d());


      // Calculate our current equation, just for dashbard display to compare
      double area = 0;
      for(PhotonTrackedTarget x : e.targetsUsed)
      {
        area += x.getArea();
      }

      double calculatedConf = Math.pow(area,2) * 8.0/3.0 - 0.83333333;

      SmartDashboard.putNumber(cam.getName()+" total area", area);
      SmartDashboard.putNumber(cam.getName()+" calculated conf", calculatedConf);

      double xy; SmartDashboard.getNumber("Photon XY Confidence", 0);
      double r; SmartDashboard.getNumber("Photon R Confidence", 0);

      if(SmartDashboard.getBoolean("Photon use calculated", false))
      {
        xy = calculatedConf;
        r = calculatedConf;
      }
      else
      {
        xy = SmartDashboard.getNumber("Photon XY Confidence", 0);
        r = SmartDashboard.getNumber("Photon R Confidence", 0);
      }

      RobotContainer.s_Swerve.swerveOdometry.addVisionMeasurement(
        e.estimatedPose.toPose2d(), 
        e.timestampSeconds,
        new Matrix<N3,N1>(new SimpleMatrix(new double[]{xy, xy, r})));

    }
  }
      
      
}