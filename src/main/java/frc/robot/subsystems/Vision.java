// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

@Logged
public class Vision extends SubsystemBase {
  // Sim stuff
  VisionSystemSim visionSim = new VisionSystemSim("main");
  Supplier<Pose3d> poseSupplier;
  private boolean useSim = true;

  private EstimateConsumer estimateConsumer;

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private Camera rightCamera =
      new Camera(
          "Right",
          new Transform3d(0.22, -0.285, 0.494, new Rotation3d(0.0, 0.0, Math.toRadians(30))),
          visionSim,
          useSim);
  private Camera leftCamera =
      new Camera(
          "Left",
          new Transform3d(0.22, 0.285, 0.494, new Rotation3d(0.0, 0.0, Math.toRadians(-30))),
          visionSim,
          useSim);
  private Camera lowerRightCamera =
      new Camera(
          "Back",
          new Transform3d(
              0.22, 0.285, (0.494 - 0.245) + 0.0381, new Rotation3d(0.0, 0.0, Math.toRadians(-30))),
          visionSim,
          useSim);
  private Camera lowerLeftCamera =
      new Camera(
          "LowerLeft",
          new Transform3d(
              0.22, -0.285, (0.494 - 0.245) + 0.0381, new Rotation3d(0.0, 0.0, Math.toRadians(30))),
          visionSim,
          useSim);
  private Camera[] cameras = {rightCamera, leftCamera, lowerRightCamera, lowerLeftCamera};

  /** Creates a new Vision. */
  public Vision(EstimateConsumer poseConsumer, Supplier<Pose3d> simPoseSupplier) {
    estimateConsumer = poseConsumer;
    poseSupplier = simPoseSupplier;

    if (Robot.isSimulation() && useSim) {
      visionSim.addAprilTags(kTagLayout);
    } else {
      visionSim = null;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    for (var camera : cameras) {
      camera.update(estimateConsumer);
    }
  }

  @Override
  public void simulationPeriodic() {
    visionSim.update(poseSupplier.get());
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }

  @Logged
  public class Camera {
    private Transform3d transform;
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private EstimatedRobotPose estimatedPose;
    private double xyStd = 0.0;
    private double angStd = 0.0;

    private static final double constrainedPnpXyStd = 0.4;
    private static final double constrainedPnpAngStd = 0.14;
    private static final Optional<ConstrainedSolvepnpParams> constrainedSolvePNPparam =
        Optional.of(new ConstrainedSolvepnpParams(true, 0.0));
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public Camera(
        String carmeraName,
        Transform3d robotToCameraTransform,
        VisionSystemSim visionSim,
        boolean useSim) {
      camera = new PhotonCamera(carmeraName);
      transform = robotToCameraTransform;
      poseEstimator =
          new PhotonPoseEstimator(kTagLayout, PoseStrategy.CONSTRAINED_SOLVEPNP, transform);

      if (Robot.isSimulation() && useSim) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(10);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim simCamera = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(simCamera, robotToCameraTransform);
      }
    }

    public void update(EstimateConsumer visionConsumer) {
      poseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
      poseEstimator.addHeadingData(Timer.getFPGATimestamp(), poseSupplier.get().getRotation());
      for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
        var estimate =
            poseEstimator.update(
                result, Optional.empty(), Optional.empty(), constrainedSolvePNPparam);

        if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) {
          estimatedPose = null;
          continue;
        }

        var tempEstimatedPose = estimate.get().estimatedPose;
        // Check if estimated pose is within the field
        if (tempEstimatedPose.getX() < 0
            || tempEstimatedPose.getX() > kTagLayout.getFieldLength()
            || tempEstimatedPose.getY() < 0
                && tempEstimatedPose.getY() > kTagLayout.getFieldWidth()) {
          estimatedPose = null;
          continue;
        }

        estimatedPose = estimate.get();
        var target = estimatedPose.targetsUsed.get(0);

        // Determine the distance from the camera to the tag.
        double distance = target.bestCameraToTarget.getTranslation().getNorm();

        // Calculate the pose estimation weights for X/Y location. As
        // distance increases, the tag is trusted exponentially less.
        xyStd = constrainedPnpXyStd * distance * distance;

        angStd = constrainedPnpAngStd * distance * distance;

        if (Robot.isReal()) {
          visionConsumer.accept(
              estimatedPose.estimatedPose.toPose2d(),
              estimatedPose.timestampSeconds,
              VecBuilder.fill(xyStd, xyStd, angStd));
        }
      }
    }
  }
}
