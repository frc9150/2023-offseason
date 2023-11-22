package bot

import java.util.function.BooleanSupplier

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.PIDCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class AutoBalance(swerve: Swerve, t: Translation2d) : SequentialCommandGroup(
	swerve.run({ swerve.drive(t, 0.0, false) })
		.until(Trigger(BooleanSupplier { swerve.getTiltMagnitude() > 0.2 }))
		.withTimeout(1.5),
	// NOTE: These pid values and tolerances are from
	// memory after I accidentally modified the wrong
	// values in the pit, and so they're effectively
	// UNTESTED. Use with caution.
	PIDCommand(
		PIDController(2.1, 0.0075, 0.0).apply { setTolerance(0.03, 0.05) },
		swerve::getTiltMagnitude,
		// target tilt value
		0.0,
		{ outputRaw ->
			val output = Math.abs(outputRaw)
			val rot = swerve.getTiltDirection()
			if (output > 0.05) {
				swerve.drive(Translation2d(output * rot.getCos(), output * rot.getSin()), 0.0, false)
			} else {
				swerve.drive(Translation2d(0.0,0.0), 0.0, false)
			}
		},
		swerve),
	swerve.run(swerve::lockModules))
{
}
