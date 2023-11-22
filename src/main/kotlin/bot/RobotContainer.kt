package bot

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand

class RobotContainer {
	private val driver = CommandXboxController(0)
	private val operator = CommandXboxController(1)

	private val swerve = Swerve()
	private val intake = Intake()
	private val elevator = Elevator()
	private val rope = Rope()
	private val chooser = SendableChooser<Command>()

	private fun preset(e: Double, r: Double) = elevator.goToPos(e).alongWith(rope.goToPos(r))

	init {
		val defaultS = 0.75
		val slowS = 0.5
		val fastS = 1.0

		fun transSpeed(): Double {
			val slow = driver.leftBumper().getAsBoolean()
			val fast = driver.rightBumper().getAsBoolean()
			if (slow && fast) { return defaultS }
			if (slow) { return slowS }
			if (fast) { return fastS }
			return defaultS
		}

		// rotation speed doesn't go above default, even when fast button is pressed
		fun rotSpeed() = Math.min(transSpeed(), defaultS)
		
		swerve.setDefaultCommand(TeleopSwerve(
			driver::getLeftX,
			driver::getLeftY,
			driver::getRightX,
			::transSpeed,
			::rotSpeed,
			swerve))

		driver.a().whileTrue(swerve.run(swerve::lockModules))

		// shelf cube
		operator.a().whileTrue(preset(32.5, 37.143))
		// shelf cone
		operator.b().whileTrue(preset(35.0, 37.143))
		// cube mid
		operator.x().whileTrue(preset(47.0, 75.5))
		// cube high
		operator.y().whileTrue(preset(74.85, 65.76))

		operator.leftStick().whileTrue(preset(0.0, 115.5))

		operator.start().whileTrue(preset(0.0, 0.0))

		operator.leftBumper().whileTrue(intake.run(intake::intake))
		operator.rightBumper().whileTrue(intake.run(intake::eject))

		driver.start().onTrue(swerve.runOnce { swerve.setPose(Pose2d()) })
	}
	
	/// Autos
	init {
		chooser.setDefaultOption("do nothing", InstantCommand({}));
		fun taxi() = swerve.run { swerve.drive(Translation2d(-1.0, 0.0), 0.0, true) }.withTimeout(4.0)

		fun putCubeHigh() =
			preset(74.85, 65.76)
			.withTimeout(1.5)
			.andThen(
				intake.run(intake::eject)
				.withTimeout(1.0))
			.andThen(
				preset(0.0, 0.0)
				.withTimeout(2.5))

		fun putCubeMid() =
			preset(47.0, 75.5)
			.withTimeout(1.5)
			.andThen(
				intake.run(intake::eject)
				.withTimeout(1.0))
			.andThen(
				preset(0.0, 0.0)
				.withTimeout(2.5))

		chooser.addOption("put cube high", putCubeHigh())
		chooser.addOption("put cube mid", putCubeMid())
		chooser.addOption("put cube high, taxi", putCubeHigh().andThen(taxi()))
		chooser.addOption("put cube mid, taxi", putCubeMid().andThen(taxi()))
		chooser.addOption("just taxi", taxi())
		chooser.addOption("(cube high) balance?", putCubeHigh().andThen(AutoBalance(swerve, Translation2d(-1.5, 0.0))))
		SmartDashboard.putData("Auto", chooser)
	}

	init { CameraServer.startAutomaticCapture()  }

	fun getAutonomousCommand(): Command =
		swerve.runOnce { swerve.setPose(Pose2d()) }
		.andThen(chooser.getSelected())
		.andThen(swerve.runOnce { swerve.setRotation(Rotation2d.fromDegrees(180.0)) })

}
