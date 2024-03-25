package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Vision.AprilTagVision;
import frc.robot.Vision.NoteVision;
import frc.robot.automations.AimAssist;
import frc.robot.automations.DriveToStage;
import frc.robot.automations.RotateToAmp;
import frc.robot.automations.RotateToStage;
import frc.robot.commands.driveCommands.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	/* Controllers */
	public static final XboxController xboxController = new XboxController(0);
	public static final CommandXboxController commandXBoxController = new CommandXboxController(0);
	private final static Joystick driver = new Joystick(0);

	/* Drive Controls */
	private static final int translationAxis = XboxController.Axis.kLeftY.value;
	private static final int strafeAxis = XboxController.Axis.kLeftX.value;
	private static final int rotationAxis = XboxController.Axis.kRightX.value;

	/* Driver Buttons */

	private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
	private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

	POVButton d_Uppov = new POVButton(driver, 0);
	POVButton d_Rightpov = new POVButton(driver, 90);
	POVButton d_Downpov = new POVButton(driver, 180);
	POVButton d_Leftpov = new POVButton(driver, 270);

	// private final JoystickButton IntakeEnableCommand = new JoystickButton(driver,
	// XboxController.Button.kRightBumper.value);
	/* Subsystems */

	/* Defining all of the subsystems and additional systems. */
	public final static Swerve swerve = new Swerve();
	public static final ShooterSubsystem shooter = new ShooterSubsystem();
	public static final TransportationSubsystem transportation = new TransportationSubsystem();
	public static final ClimbSubsystem climb = new ClimbSubsystem();
	public static final NoteVision note_vision = new NoteVision();
	public static final AprilTagVision aprilTag_Vision = new AprilTagVision();

	private final SendableChooser<Command> autoChooser;

	/**
	 * The container
	 * for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Another option that allows you to specify the default auto by its name
		// autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
		configureButtonBindings();
		setDefaultCommands();
		registerCommand();
		autoChooser = AutoBuilder.buildAutoChooser();

		Shuffleboard.getTab("Robot")
				.add("Auto", autoChooser);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		/* Driver Buttons */
		zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));

		// Shooter Triggers

		// Speaker Shot
		commandXBoxController.rightBumper().whileTrue(
				shooter.shootUpCommand()
						.alongWith(waitAndLoadCommand())
						.deadlineWith(teleopSwerve(true)));

		// Amp Shot
		commandXBoxController.leftBumper().whileTrue(
				shooter.shooterDownCommand()
						.alongWith(waitAndLoadCommand())
						.alongWith(new AimAssist(
								swerve,
								() -> -driver.getRawAxis(translationAxis),
								() -> -driver.getRawAxis(strafeAxis),
								() -> -driver.getRawAxis(rotationAxis)))
						.alongWith(new RotateToAmp(swerve,
								() -> -driver.getRawAxis(translationAxis),
								() -> -driver.getRawAxis(strafeAxis),
								() -> -driver.getRawAxis(rotationAxis))));

		// Transportation Triggers
		commandXBoxController.a().toggleOnTrue(
				transportation.transportUpCommand());
		commandXBoxController.x().toggleOnTrue(transportation.transportDownCommand());

		// Climb Triggers
		commandXBoxController.rightTrigger().whileTrue(new RotateToStage(
				swerve,
				() -> -driver.getRawAxis(translationAxis),
				() -> -driver.getRawAxis(strafeAxis),
				() -> -driver.getRawAxis(rotationAxis),
				aprilTag_Vision)
				.withTimeout(0.5)
				.andThen(new DriveToStage(swerve,
						() -> -driver.getRawAxis(translationAxis),
						() -> -driver.getRawAxis(strafeAxis),
						() -> -driver.getRawAxis(rotationAxis),
						aprilTag_Vision))
				.withTimeout(3)
				.andThen(swerve.driveToSideCommand(swerve,
						() -> -driver.getRawAxis(translationAxis),
						() -> -driver.getRawAxis(rotationAxis)))
				.withTimeout(2)
				.andThen(climb.setSpeedCommand()));

		// Aim Assist Triggers
		commandXBoxController.leftTrigger().whileTrue(new AimAssist(
				swerve,
				() -> -driver.getRawAxis(translationAxis),
				() -> -driver.getRawAxis(strafeAxis),
				() -> -driver.getRawAxis(rotationAxis))
				.alongWith(transportation.transportUpCommand()));

		// Test Triggers
	}

	/**
	 * Setting all of the commands that will run constantly when not scheduled.
	 */

	private void setDefaultCommands() {
		swerve.setDefaultCommand(teleopSwerve(false));

		shooter.setDefaultCommand(shooter.stopMotorsCommand());

		transportation.setDefaultCommand(transportation.stopMotorsCommand());

		// climbSubsystem.setDefaultCommand(climbSubsystem.stopMotorsCommand());

	}

	/**
	 * An additional command in order to replace the current swerve with a swerve
	 * with the ability to cross wheels.
	 * 
	 * @param crossWhileNotMoving true if to cross, false if not
	 * @return command
	 */
	private Command teleopSwerve(boolean crossWhileNotMoving) {
		return new TeleopSwerve(
				swerve,
				() -> -driver.getRawAxis(translationAxis),
				() -> -driver.getRawAxis(strafeAxis),
				() -> -driver.getRawAxis(rotationAxis),
				() -> robotCentric.getAsBoolean(),
				crossWhileNotMoving);
	}

	/**
	 * A command that waits 0.1 seconds, checks if the joysticks are moving, if they
	 * aren't it will wait 0.5 seconds and activate the transportation
	 * 
	 * @return Command
	 */

	private Command waitAndLoadCommand() {
		return new WaitCommand(0.1)
				.andThen(new WaitUntilCommand(() -> !areJoysticksMoving()))
				.andThen(new WaitCommand(0.5))
				.andThen(transportation.transportUpCommand());
	}

	/**
	 * Registers all commands inside it into the NamedCommands object
	 */

	public void registerCommand() {
		NamedCommands.registerCommand("ShooterUpDown",
				transportation.transportDowmAutoCommand(0.3)
						.andThen(shooter.shootUpAutoCommand(2)
								.alongWith(new WaitCommand(0.5)
										.andThen(transportation
												.transportUpAutoCommand(2)))));
		NamedCommands.registerCommand("StopShooter1",
				shooter.stopMotorsCommand());
		NamedCommands.registerCommand("TransportUp",
				transportation.transportUpAutoCommand(3));
		NamedCommands.registerCommand("Shooter",
				shooter.shootUpAutoCommand(2)
						.alongWith(new WaitCommand(0.6)
								.andThen(transportation.transportUpAutoCommand(1))));
		// NamedCommands.registerCommand("TransportDown",
		// transportationSubsystem.transportDowmAutoCommand(0.3));
		/*
		 * p
		 * NamedCommands.registerCommand("AutoAimAssist", new AimAssistAutonomus(
		 * swerve,
		 * () -> -driver.getRawAxis(translationAxis),
		 * () -> -driver.getRawAxis(strafeAxis),
		 * () -> -driver.getRawAxis(rotationAxis)));
		 */
		NamedCommands.registerCommand(
				"AimAtNote",
				new InstantCommand(
						() -> PPHolonomicDriveController
								.setRotationTargetOverride(swerve::getRotationTargetOverride)));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public static boolean areJoysticksMoving() {
		return Math.abs(MathUtil.applyDeadband(driver.getRawAxis(translationAxis), Constants.stickDeadband)) > 0.0 ||
				Math.abs(MathUtil.applyDeadband(driver.getRawAxis(strafeAxis), Constants.stickDeadband)) > 0.0 ||
				Math.abs(MathUtil.applyDeadband(driver.getRawAxis(rotationAxis), Constants.stickDeadband)) > 0.0;
	}
}
