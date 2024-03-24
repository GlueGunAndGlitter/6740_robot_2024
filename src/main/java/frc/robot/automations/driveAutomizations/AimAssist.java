// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.driveAutomizations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAssist extends PIDCommand {
  /** Creates a new Test. */
  public AimAssist(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotationSup) {
    super(
        // The controller that the command will use
        new PIDController(0.008, 0, 0.0008),
        // This should return the measurement
        () -> RobotContainer.note_vision.getRobotToNoteYaw(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
          double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

          /* Drive */
          if (RobotContainer.note_vision.seesNote()) {
            if (RobotContainer.xboxController.getYButton()) {
              swerve.drive(
                  new Translation2d(0.6, -strafeVal).times(Constants.Swerve.maxSpeed),
                  -output * Constants.Swerve.maxAngularVelocity,
                  false,
                  true);
            } else {
              swerve.drive(
                  new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                  -output * Constants.Swerve.maxAngularVelocity,
                  true,
                  true);
            }
          } else {
            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                true,
                true);
          }

        },
        swerve);
  };
  // Use addRequirements() here to declare subsystem dependencies.
  // Configure additional PID options by calling `getController` here.

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
