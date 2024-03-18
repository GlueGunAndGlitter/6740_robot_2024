// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimbSubsystem extends SubsystemBase {
  CANSparkMax climbRightMotor = new CANSparkMax(Constants.ClimbConstants.CLIMB_RIGHT_MOTOR_PORT,
      MotorType.kBrushless);
  CANSparkMax climbLeftMotor = new CANSparkMax(Constants.ClimbConstants.CLIMB_LEFT_MOTOR_PORT,
      MotorType.kBrushless);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
  }

  @Override
  public void periodic() {
    climbLeftMotor.set(RobotContainer.xboxController.getRightTriggerAxis());
    climbRightMotor.set(RobotContainer.xboxController.getRightTriggerAxis());
  }
}