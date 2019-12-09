package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DoubleSolenoid
import frc.team4069.robot.Constants
import frc.team4069.saturn.lib.localization.DifferentialDriveLocalization
import frc.team4069.saturn.lib.mathematics.twodim.control.LTVUnicycleTracker
import frc.team4069.saturn.lib.mathematics.twodim.control.TrajectoryTrackerOutput
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import frc.team4069.saturn.lib.mathematics.units.meter
import frc.team4069.saturn.lib.motor.rev.SaturnMAX
import frc.team4069.saturn.lib.sensors.SaturnPigeon
import frc.team4069.saturn.lib.subsystem.DifferentialDriveModel
import frc.team4069.saturn.lib.subsystem.TankDriveSubsystem
import kotlin.properties.Delegates

object Drivetrain : TankDriveSubsystem() {

    const val kHighGearSensitivity = 0.6
    const val kLowGearSensitivity = 0.625

    const val kHighGearConversion = 1.0 / 8.93
    const val kLowGearConversion = 1.0 / 24.23

    var gear by Delegates.observable(Gear.High) { _, old, new ->
        if(old != new) {
            when(new) {
                Gear.High-> {
                    leftEncoder.canEncoder.positionConversionFactor = kHighGearConversion
                    leftEncoder.canEncoder.velocityConversionFactor = kHighGearConversion

                    rightEncoder.canEncoder.positionConversionFactor = kHighGearConversion
                    rightEncoder.canEncoder.velocityConversionFactor = kHighGearConversion

                    shifter.set(DoubleSolenoid.Value.kForward)
                }
                Gear.Low -> {
                    leftEncoder.canEncoder.positionConversionFactor = kLowGearConversion
                    leftEncoder.canEncoder.velocityConversionFactor = kLowGearConversion

                    rightEncoder.canEncoder.positionConversionFactor = kLowGearConversion
                    rightEncoder.canEncoder.velocityConversionFactor = kLowGearConversion

                    shifter.set(DoubleSolenoid.Value.kReverse)
                }
            }
        }
    }

    private val shifter = DoubleSolenoid(0, 7)

    override val leftMotor = SaturnMAX(1, CANSparkMaxLowLevel.MotorType.kBrushless, Constants.kLeftDrivetrainUnitModel)
    private val leftSlave = SaturnMAX(2, CANSparkMaxLowLevel.MotorType.kBrushless, Constants.kLeftDrivetrainUnitModel)
    val leftEncoder = leftMotor.encoder

    override val rightMotor = SaturnMAX(3, CANSparkMaxLowLevel.MotorType.kBrushless, Constants.kRightDrivetrainUnitModel)
    private val rightSlave = SaturnMAX(4, CANSparkMaxLowLevel.MotorType.kBrushless, Constants.kRightDrivetrainUnitModel)
    val rightEncoder = rightMotor.encoder

    private val gyroTalon = TalonSRX(5)
    override val gyro = SaturnPigeon(gyroTalon)

    override val driveModel = DifferentialDriveModel(2.3563.feet, Constants.DRIVETRAIN_KV, Constants.DRIVETRAIN_KA, Constants.DRIVETRAIN_KS)
    override val localization = DifferentialDriveLocalization(gyro, { leftEncoder.position }, { rightEncoder.position })
    override val trajectoryTracker = LTVUnicycleTracker(16.409255758939636,
        5.743092173917074,
        5.704580270256358,
        8.841822353363662) { velocity }

    val velocity
        get() = (leftEncoder.velocity + rightEncoder.velocity) / 2.0

    init {
        leftEncoder.canEncoder.positionConversionFactor = kHighGearConversion
        leftEncoder.canEncoder.velocityConversionFactor = kHighGearConversion
        leftEncoder.resetPosition(0.meter)
        rightEncoder.canEncoder.positionConversionFactor = kHighGearConversion
        rightEncoder.canEncoder.velocityConversionFactor = kHighGearConversion
        rightEncoder.resetPosition(0.meter)

        leftMotor.outputInverted = false
        rightMotor.outputInverted = true

        // Loop gains are minuscule because the controller still believes it's operating on RPMs
        // Since SaturnLib handles unit conversions rather than using what REV has built in
//        leftMotor.controller.p = 2E-3
//        leftMotor.controller.i = 5E-6
//        leftMotor.controller.d = 2.5E-3
//        rightMotor.controller.p = 2E-3
//        rightMotor.controller.i = 5E-6
//        rightMotor.controller.d = 2.5E-3

        leftMotor.controller.apply {
            p = 0.001
            i = 0.0
            d = 0.0
        }

        rightMotor.controller.apply {
            p = 0.001
            i = 0.0
            d = 0.0
        }

        rightSlave.follow(rightMotor)
        leftSlave.follow(leftMotor)

        leftMotor.canSparkMax.setSecondaryCurrentLimit(50.0)
        leftMotor.canSparkMax.setSecondaryCurrentLimit(50.0)
    }

    override fun setNeutral() {
        leftMotor.setDutyCycle(0.0)
        rightMotor.setDutyCycle(0.0)
    }

    override fun setOutput(output: TrajectoryTrackerOutput) {
        val demand = driveModel.getDemand(output)
        leftMotor.setVelocity(demand.leftSetpoint, arbitraryFeedForward = demand.leftArbFF)
        rightMotor.setVelocity(demand.rightSetpoint, arbitraryFeedForward = demand.rightArbFF)
    }

    enum class Gear {
        Low,
        High
    }
}