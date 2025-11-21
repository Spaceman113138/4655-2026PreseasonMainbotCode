// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LoggingFiles;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
@CustomLoggerFor(EstimatedRobotPose.class)
public class EstimatedRobotPoseLogger extends ClassSpecificLogger<EstimatedRobotPose> {

    public EstimatedRobotPoseLogger() {
        super(EstimatedRobotPose.class);
    }
    
    @Override
    protected void update(EpilogueBackend backend, EstimatedRobotPose estimatedPose) {
        if (estimatedPose == null) {
            return;
        }
        backend.log("Pose", estimatedPose.estimatedPose, Pose3d.struct);
        backend.log("Timestamp Seconds", estimatedPose.timestampSeconds);
        backend.log("Strategy", estimatedPose.strategy.toString());
        backend.log("Number Of Targets Used", estimatedPose.targetsUsed.size());
        if (Epilogue.shouldLog(Importance.DEBUG)) {
            for (int i = 0; i < estimatedPose.targetsUsed.size(); i++) {
                var target = estimatedPose.targetsUsed.get(i);
                backend.log("Targets/" + i + "/Cam to Target", target.bestCameraToTarget, Transform3d.struct);
                backend.log("Targets/" + i + "/Fiducial Id", target.fiducialId);
                backend.log("Targets/" + i + "/area", target.area);
                backend.log("Targets/" + i + "/yaw", target.yaw);
                backend.log("Targets/" + i + "/pitch", target.pitch);
                backend.log("Targets/" + i + "/ambiguity", target.poseAmbiguity);
                backend.log("Targets/" + i + "/Object Id", target.objDetectId);
                backend.log("Targets/" + i + "/Object Confidence", target.objDetectConf);
            }
        }
        
    }

}
