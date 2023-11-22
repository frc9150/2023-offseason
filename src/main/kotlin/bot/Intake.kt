package bot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMaxLowLevel.MotorType

class Intake : SubsystemBase() {
	// left, right
	// TODO: performance implications of using lists?
	private val motors = arrayOf(7,8).map({ id -> CANSparkMax(id, MotorType.kBrushless) })

	init {
		motors.forEach {
			it.restoreFactoryDefaults()
			it.setIdleMode(IdleMode.kBrake)
			it.setSmartCurrentLimit(15)
			it.enableVoltageCompensation(11.0)
		}
		motors[1].setInverted(true)
	}

	// TODO: maybe don't always reset encoders?
	private val encoders = motors.map { motor -> motor.getEncoder().apply { setPosition(0.0) } }

	private val controllers = motors.zip(encoders) { motor, encoder -> motor.getPIDController().apply {
		setFeedbackDevice(encoder)
		// TODO
		setP(0.5)
		setI(0.0)
		setD(0.01)
		setOutputRange(-1.0, 1.0)
	}}

	init { motors.forEach(CANSparkMax::burnFlash) }

	init { setDefaultCommand(MaintainPositionCmd()) }

	fun set(output: Double) {
		motors.forEach { it.set(output) }
	}

	fun setPositions(positions: Iterable<Double>) {
		controllers.zip(positions) { controller, pos ->
			controller.setReference(pos, CANSparkMax.ControlType.kPosition)
		}
	}

	fun getPositions() = encoders.map { it.getPosition() }

	fun intake() = set(0.5)
	fun eject() = set(-0.25)

	override fun periodic() {
		SmartDashboard.putNumber("L Current", motors[0].getOutputCurrent())
		SmartDashboard.putNumber("R Current", motors[1].getOutputCurrent())
	}

	inner class MaintainPositionCmd : CommandBase() {
		init { addRequirements(this@Intake) }

		// Safe as long as execute is never called before initialize
		private lateinit var positions: List<Double>

		override fun initialize() { positions = getPositions() }
		override fun execute() { setPositions(positions) }
		override fun end(interrupted: Boolean) { set(0.0) }
	}
}
