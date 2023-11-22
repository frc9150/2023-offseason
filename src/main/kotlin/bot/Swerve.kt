package bot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

import com.kauailabs.navx.frc.AHRS

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.SparkMaxAbsoluteEncoder.Type

class Swerve : SubsystemBase() {
	companion object {
		val trackWidth = Units.inchesToMeters(24.0)
		val wheelBase = Units.inchesToMeters(24.0)

		// Coordinate space?
		val kinematics = SwerveDriveKinematics(
			Translation2d(wheelBase / 2, trackWidth / 2),
			Translation2d(wheelBase / 2, -trackWidth / 2),
			Translation2d(-wheelBase / 2, trackWidth / 2),
			Translation2d(-wheelBase / 2, -trackWidth / 2))

		// TODO
		val maxLinVel = Module.driveFreeSpeed
		val maxModVel = Module.driveFreeSpeed //maxLinVel
		val maxAngVel = 0.6 * Math.PI * 2.0
	}

	// frontLeft, frontRight, backLeft, backRight
	private val modules = arrayOf(
		Module(11, 10, -Math.PI/2),
		Module(15, 14, 0.0),
		Module(13, 12, Math.PI),
		Module(17, 16, Math.PI/2))

	private val gyro = AHRS().apply(AHRS::calibrate)

	private val odo = SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getModulePositions())

	fun drive(linVel: Translation2d, angVel: Double, fieldRel: Boolean) {
		var speeds = ChassisSpeeds(linVel.getX(), linVel.getY(), angVel)
		if (fieldRel) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation())
		val states = kinematics.toSwerveModuleStates(speeds)
		SwerveDriveKinematics.desaturateWheelSpeeds(states, speeds, maxModVel, maxLinVel, maxAngVel)
		modules.zip(states, Module::setDesiredState)
	}

	// Used in auto
	fun setModuleStates(states: Array<SwerveModuleState>) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, maxModVel)
		modules.zip(states, Module::setDesiredState)
	}

	fun lockModules() {
		modules.zip(arrayOf(45.0, -45.0, -45.0, 45.0)) { mod, ang -> mod.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(ang))) }
	}

	fun getPose() = odo.getPoseMeters()

	fun setPose(pose: Pose2d) {
		odo.resetPosition(gyro.getRotation2d(), getModulePositions(), pose)
	}

	fun setRotation(rot: Rotation2d) {
		setPose(Pose2d(getPose().getTranslation(), rot))
	}

	fun getModulePositions() = modules.map(Module::getPosition).toTypedArray()

	// Gyro things for auto-balance
	// Thanks MPU6050
	fun getGravity(): Array<Float> {
		gyro.apply {
			val w = getQuaternionW()
			val x = getQuaternionX()
			val y = getQuaternionY()
			val z = getQuaternionZ()

			return arrayOf(
				2 * (x * z - w * y),
				2 * (w * x + y * z),
				(w * w - x * x - y * y + z * z))
		}
	}

	// Magnitude of the x and y components of the vector that
	// gravity makes in the robot's coordinate system
	// (bumpers form the xy plane).
	// When the bot is fully tilted, the gravity vector
	// will be contained fully within the xy plane, and
	// thus the tilt magnitude will be one.
	fun getTiltMagnitude() = getGravity().let { Math.sqrt((it[0] * it[0] + it[1] * it[1]).toDouble()) }
	fun getTiltDirection() = getGravity().let { Rotation2d(it[0].toDouble(), it[1].toDouble()) }

	override fun periodic() {
		odo.update(gyro.getRotation2d(), getModulePositions())
		SmartDashboard.putNumber("Pose Yaw", getPose().getRotation().getDegrees())
		SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw().toDouble())
		SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch().toDouble())
		SmartDashboard.putNumber("Gyro Roll", gyro.getRoll().toDouble())
		SmartDashboard.putNumber("Tilt Magnitude", getTiltMagnitude())
		modules.zip(arrayOf("FL", "FR", "BL", "BR")) { mod, name ->
			SmartDashboard.putNumber(name + " Velocity", mod.driveE.getVelocity())
		}
		//SmartDashboard.putNumber("FL Current", modules[0].driveM.getOutputCurrent())
		getGravity().let {
			SmartDashboard.putNumber("Gravity X", it[0].toDouble())
			SmartDashboard.putNumber("Gravity Y", it[1].toDouble())
		}
	}


	class Module(driveCanId: Int, turnCanId: Int, val chassisAngularOffset: Double) {
		companion object {
			const val wheelDiameter = 0.0762 // meters
			const val neoFreeSpeedRpm = 5676.0

			const val wheelCircumference = wheelDiameter * Math.PI // meters

			const val drivePinionTeeth = 13.0

			const val driveReduction = (45.0 * 22) / (drivePinionTeeth * 15)

			const val drivePosFac = wheelCircumference / driveReduction // revolutions -> meters
			const val driveVelFac = drivePosFac / 60.0 // rpm -> m/s
			const val driveFreeSpeed = neoFreeSpeedRpm * driveVelFac // m/s

			const val turnPosFac = 2 * Math.PI // revolutions -> radians
			const val turnVelFac = turnPosFac / 60.0 // rpm -> radians per second

			const val turnPIDMinInput = 0.0 // radians
			const val turnPIDMaxInput = turnPosFac // radians
			
			// PID on velocity
			const val driveP = 0.1
			const val driveI = 0.0
			const val driveD = 0.0
			const val driveFF = 1 / driveFreeSpeed

			// PID on position
			// Why?
			const val turnP = 1.0
			const val turnI = 0.0
			const val turnD = 0.0
			const val turnFF = 0.0

			// Why?
			val driveIdle = IdleMode.kBrake
			val turnIdle = IdleMode.kBrake

			const val driveCurrentLim = 50
			const val turnCurrentLim = 20
		}

		val driveM = CANSparkMax(driveCanId, MotorType.kBrushless).apply {
			restoreFactoryDefaults()
			setIdleMode(driveIdle)
			setSmartCurrentLimit(driveCurrentLim)
			enableVoltageCompensation(12.0)
		}
		private val turnM = CANSparkMax(turnCanId, MotorType.kBrushless).apply {
			restoreFactoryDefaults()
			setIdleMode(turnIdle)
			setSmartCurrentLimit(turnCurrentLim)
			enableVoltageCompensation(12.0)
		}

		val driveE = driveM.getEncoder().apply {
			setPositionConversionFactor(drivePosFac)
			setVelocityConversionFactor(driveVelFac)
			setPosition(0.0)
		}
		private val turnE = turnM.getAbsoluteEncoder(Type.kDutyCycle).apply {
			setPositionConversionFactor(turnPosFac)
			setVelocityConversionFactor(turnVelFac)
			setInverted(true)
		}

		private val drivePID = driveM.getPIDController().apply {
			setFeedbackDevice(driveE)
			setP(driveP)
			setI(driveI)
			setD(driveD)
			setFF(driveFF)
			setOutputRange(-1.0, 1.0)
		}
		private val turnPID = turnM.getPIDController().apply {
			setFeedbackDevice(turnE)
			setPositionPIDWrappingEnabled(true)
			setPositionPIDWrappingMinInput(turnPIDMinInput)
			setPositionPIDWrappingMaxInput(turnPIDMaxInput)

			setP(turnP)
			setI(turnI)
			setD(turnD)
			setFF(turnFF)
			setOutputRange(-1.0, 1.0)
		}

		init {
			driveM.burnFlash()
			turnM.burnFlash()
		}

		fun getState() = SwerveModuleState(driveE.getVelocity(), Rotation2d(turnE.getPosition() - chassisAngularOffset))
		fun getPosition() = SwerveModulePosition(driveE.getPosition(), Rotation2d(turnE.getPosition() - chassisAngularOffset))

		fun setDesiredState(state: SwerveModuleState) {
			val corrected = SwerveModuleState(state.speedMetersPerSecond, state.angle.plus(Rotation2d(chassisAngularOffset)))
			val optimized = SwerveModuleState.optimize(corrected, Rotation2d(turnE.getPosition()))
			drivePID.setReference(optimized.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity)
			turnPID.setReference(optimized.angle.getRadians(), CANSparkMax.ControlType.kPosition)
		}

		fun resetEncoders() { driveE.setPosition(0.0) }
	}
}
