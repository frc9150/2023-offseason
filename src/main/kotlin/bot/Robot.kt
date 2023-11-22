package bot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

class Robot : TimedRobot() {
	private var autoCmd : Command? = null
	private var robotContainer : RobotContainer? = null

	override fun robotInit() { robotContainer = RobotContainer() }
	override fun robotPeriodic() { CommandScheduler.getInstance().run() }

	override fun autonomousInit() {
		autoCmd = robotContainer?.getAutonomousCommand()
		autoCmd?.schedule()
	}

	override fun teleopInit() { autoCmd?.cancel() }

	override fun testInit() { CommandScheduler.getInstance().cancelAll() }
}
