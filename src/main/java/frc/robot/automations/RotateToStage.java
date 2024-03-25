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
public class RotateToStage extends PIDCommand {
  /** Creates a new RotateToStage. */
  static AprilTagVision aprilTagVision;
  static Swerve swerve;

  public RotateToStage(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotationSup, AprilTagVision aprilTagVision) {

    super(
        // The controller that the command will use
        new PIDController(7 / 1800, 0, 7 / 18000),
        // This should return the measurement
        () -> swerve.getHeading().getDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> angelToStage(),
        // This uses the output
        output -> {
          // Use the output here
          double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);

          /* Drive */
          swerve.drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
              -output * Constants.Swerve.maxAngularVelocity,
              true,
              true);
        },
        swerve);

    RotateToStage.aprilTagVision = aprilTagVision;
    RotateToStage.swerve = swerve;
  };
  // Use addRequirements() here to declare subsystem dependencies.
  // Configure additional PID options by calling `getController` here

  public static double angelToStage() {
    if (aprilTagVision.seesAprilTags()) {
      if (isRedAlince()) {

        if (aprilTagVision.seesSpecificAprilTag(11) != -1) {
          return 60;
        } else if (aprilTagVision.seesSpecificAprilTag(12) != -1) {
          return -60;
        } else if (aprilTagVision.seesSpecificAprilTag(13) != -1) {
          return 180;
        } else {
          return swerve.getHeading().getDegrees();
        }

      } else {

        if (aprilTagVision.seesSpecificAprilTag(14) != -1) {
          return 180;
        } else if (aprilTagVision.seesSpecificAprilTag(15) != -1) {
          return 60;
        } else if (aprilTagVision.seesSpecificAprilTag(16) != -1) {
          return -60;
        } else {
          return swerve.getHeading().getDegrees();
        }

      }

    } else {
      return swerve.getHeading().getDegrees();
    }
  }

  public static boolean isRedAlince() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return true;
    } else {
      return false;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
