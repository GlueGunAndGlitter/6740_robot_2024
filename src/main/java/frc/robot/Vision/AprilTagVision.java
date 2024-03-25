// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

/** Add your docs here. */
public class AprilTagVision {

    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera aprilTagsCamera;
    static PhotonPoseEstimator photonPoseEstimator;

    public AprilTagVision() {
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // Forward Camera
        aprilTagsCamera = new PhotonCamera("AprilTagCamera");
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // Cam
                                                                                                             // mounted
                                                                                                             // facing
                                                                                                             // forward,
                                                                                                             // half a
                                                                                                             // meter
                                                                                                             // forward
                                                                                                             // of
                                                                                                             // center,
                                                                                                             // half a
                                                                                                             // meter up
                                                                                                             // from
                                                                                                             // center.

        // Construct PhotonPoseEstimator
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, aprilTagsCamera, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public PhotonPipelineResult getLatestResult() {
        return aprilTagsCamera.getLatestResult();
    }

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return photonPoseEstimator.update();
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public int seesSpecificAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();

            for (int i = 0; i < targets.size();) {
                if (targets.get(i).getFiducialId() == aprilTag_ID) {
                    return i;
                }
            }
        }
        return -1;
    }

    public double distanceFromAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(seesSpecificAprilTag(aprilTag_ID)).getBestCameraToTarget().getX();
        } else {
            return 1.5;
        }
    }

    public double angleFromAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(seesSpecificAprilTag(aprilTag_ID)).getBestCameraToTarget().getRotation().getX();
        } else {
            return 0;
        }
    }

    public boolean seesAprilTags() {
        var result = aprilTagsCamera.getLatestResult();
        return result.hasTargets();
    }

}
