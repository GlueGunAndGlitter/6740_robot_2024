// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class NoteVision {

    public static PhotonCamera notesCamera;

    public NoteVision() {
        notesCamera = new PhotonCamera("Notes-Limelight");
    }

    public double getRobotToNoteYaw() {

        var result = notesCamera.getLatestResult();
        if (!result.hasTargets())
            return 0.0;
        else
            return result.getBestTarget().getYaw();
    }

    public boolean seesNote() {
        return notesCamera.getLatestResult().hasTargets();
    }

}
