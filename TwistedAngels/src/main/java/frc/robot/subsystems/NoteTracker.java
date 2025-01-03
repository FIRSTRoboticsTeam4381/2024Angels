package frc.robot.subsystems;

import javax.print.DocFlavor.STRING;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LogOrDash;

public class NoteTracker extends SubsystemBase {
    PhotonCamera cam;
  
    
    public NoteTracker( String camname) {
     
        cam = new PhotonCamera(camname);
    }

    //get best note
    public PhotonTrackedTarget getBestNote () {
      PhotonPipelineResult result = cam.getLatestResult() ; 

     return result.getBestTarget();
        
    }
    public boolean hasTarget() {
        return getBestNote() != null;
    }
    @Override
    public void periodic() {
        PhotonTrackedTarget note = getBestNote();
        if (note != null){

        
            LogOrDash.logNumber("note/x", note.getYaw());
            LogOrDash.logNumber("note/y", note.getPitch());
        }   
    }
}
