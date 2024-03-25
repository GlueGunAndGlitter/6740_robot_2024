// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkFlex nonStaticMotor = new CANSparkFlex(Constants.ShooterConstants.NON_STATIC_MOTOR_PORT,
      MotorType.kBrushless);
  CANSparkFlex staticMotor = new CANSparkFlex(Constants.ShooterConstants.STATIC_MOTOR_PORT,
      MotorType.kBrushless);

  public double nonStaticSpeed;
  public double staticMotorSpeed;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    nonStaticMotor.setInverted(false);
    staticMotor.setInverted(true);

    staticMotorSpeed = 0;
    nonStaticSpeed = 0;

  }

  public void shootUp(double nonStaticMotorSpeed, double staticMotorSpeed) {
    nonStaticMotor.set(-nonStaticMotorSpeed);
    staticMotor.set(staticMotorSpeed);

  }

  public void shootDown(double nonStaticMotorSpeed, double staticMotorSpeed) {
    nonStaticMotor.set(nonStaticMotorSpeed);
    staticMotor.set(staticMotorSpeed);
  }

  public void stopMotors() {
    nonStaticMotor.stopMotor();
    staticMotor.stopMotor();
  }

  public Command shootUpAutoCommand(double stopTimeShootUp) {
    return this.run(() -> shootUp(0.8, 0.8)).withTimeout(stopTimeShootUp);
  }

  public Command shootDownAutoCommand(double stopTimeShootDown) {
    return this.run(() -> shootDown(0.4, 0.4)).withTimeout(stopTimeShootDown);
  }

  public Command shootUpCommand() {
    return this.run(() -> shootUp(nonStaticSpeed, staticMotorSpeed))
        .withTimeout(ShooterConstants.SHOOT_TIMEOUT);
  }

  public Command shooterDownCommand() {
    return this.run(() -> shootDown(staticMotorSpeed - 0.3, nonStaticSpeed - 0.4))
        .withTimeout(ShooterConstants.SHOOT_TIMEOUT);
  }

  public Command stopMotorsCommand() {
    return this.run(this::stopMotors);
  }

  @Override
  public void periodic() {
    nonStaticSpeed = Robot.nonStaticShooterMotorSpeed.getDouble(0);
    staticMotorSpeed = Robot.staticShooterMotorSpeed.getDouble(0);
  }
}
