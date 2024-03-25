// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Vision.AprilTagVision;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToStage extends PIDCommand {
  static AprilTagVision aprilTagVision;
  static Swerve swerve;

  /** Creates a new DriveToStage. */
  public DriveToStage(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotationSup, AprilTagVision aprilTagVision) {
    super(
        // The controller that the command will use
        new PIDController(0.8 / 15, 0, 0.8 / 150),
        // This should return the measurement
        () -> aprilTagVision.distanceFromAprilTag(whichAprilTagAreTheRobotLookingAt()),
        // This should return the setpoint (can also be a constant)
        () -> 1.5,
        // This uses the output
        output -> {
          // Use the output here
          double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
          double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

          /* Drive */
          swerve.drive(
              new Translation2d(output, strafeVal).times(Constants.Swerve.maxSpeed),
              rotationVal * Constants.Swerve.maxAngularVelocity,
              false,
              true);
        },
        swerve);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    DriveToStage.aprilTagVision = aprilTagVision;
    DriveToStage.swerve = swerve;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static int whichAprilTagAreTheRobotLookingAt() {
    if (aprilTagVision.seesAprilTags()) {
      if (isRedAlince()) {

        if (aprilTagVision.getAprilTag(11) != -1) {
          return 11;
        } else if (aprilTagVision.getAprilTag(12) != -1) {
          return 12;
        } else if (aprilTagVision.getAprilTag(13) != -1) {
          return 13;
        } else {
          return -1;
        }

      } else {

        if (aprilTagVision.getAprilTag(14) != -1) {
          return 14;
        } else if (aprilTagVision.getAprilTag(15) != -1) {
          return 15;
        } else if (aprilTagVision.getAprilTag(16) != -1) {
          return 16;
        } else {
          return -1;
        }

      }

    } else {
      return -1;
    }
  }

  public static boolean isRedAlince() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return true;
    } else {
      return false;
    }
  }

}
