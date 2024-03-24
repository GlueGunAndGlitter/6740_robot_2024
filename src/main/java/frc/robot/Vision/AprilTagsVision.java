// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class AprilTagsVision {
    public static PhotonCamera aprilTagsCamera;

    public AprilTagsVision() {
        aprilTagsCamera = new PhotonCamera("AprilTagVision");
    }

    private int getSpesificAprilTag(int aprilTag_ID) {
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

    public double distenceFromSpesificAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(getSpesificAprilTag(aprilTag_ID)).getBestCameraToTarget().getX();
        } else {
            return -1;
        }
    }

    public double engelFromSpesificAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(getSpesificAprilTag(aprilTag_ID)).getBestCameraToTarget().getRotation().getX();
        } else {
            return 0;
        }
    }

    public boolean seeAprilTags() {
        var result = aprilTagsCamera.getLatestResult();
        return result.hasTargets();
    }
}