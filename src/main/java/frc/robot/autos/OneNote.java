// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import org.ejml.dense.row.misc.TransposeAlgs_DDRM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportationSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNote extends SequentialCommandGroup {
  /** Creates a new OneNote. */
  TransportationSubsystem m_TransportationSubsystem;
  ShooterSubsystem m_ShooterSubsystem;

  public OneNote(TransportationSubsystem m_TransportationSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    this.m_TransportationSubsystem = m_TransportationSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        m_TransportationSubsystem.transportUpAutoCommand(4));
    // m_ShooterSubsystem.shootUpAutoCommand(3));
    // .alongWith(new WaitCommand(0.5)
    // .andThen(m_TransportationSubsystem.transportUpAutoCommand(2))));
  }
}
