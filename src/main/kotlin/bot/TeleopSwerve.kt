package bot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandBase

class TeleopSwerve(
    val joystickXRaw: () -> Double,
    val joystickYRaw: () -> Double,
    val joystickRRaw: () -> Double,
    val transSpeed: () -> Double,
    val rotSpeed: () -> Double,
    val swerve: Swerve) : CommandBase() {

    init { addRequirements(swerve) }

    override fun execute() {
        val transS = transSpeed()
        val rotS = rotSpeed()
        val x = MathUtil.applyDeadband(joystickXRaw(), 0.05) * Swerve.maxLinVel * transS
        val y = MathUtil.applyDeadband(-joystickYRaw(), 0.05) * Swerve.maxLinVel * transS
        val r = MathUtil.applyDeadband(-joystickRRaw(), 0.05) * Swerve.maxAngVel * rotS

		// Convert to WPILib coordinate system - "up" on
		// the joystick (+y) is "forward" for the robot,
		// which is the +x axis for WPILib. Similarly, +x
		// for the joystick maps to -y for WPILib.
        swerve.drive(Translation2d(y, -x), r, true)
    }
}
