package frc.team4069.robot.subsystems

import frc.team4069.saturn.lib.commands.SaturnSubsystem
import io.github.oblarg.oblog.Loggable
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import frc.team4069.saturn.lib.subsystem.TankDriveSubsystem.Companion.kQuickStopAlpha
import frc.team4069.saturn.lib.subsystem.TankDriveSubsystem.Companion.kQuickStopThreshold
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.absoluteValue
import kotlin.math.max

object Drivetrain : SaturnSubsystem(), Loggable {
    private var quickStopAccumulator = 0.0

    @Log.ToString(name = "Current State", rowIndex = 0, columnIndex = 5)
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

    private val periodicIO = PeriodicIO()

    private val leftMotor = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val leftSlave = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val rightMotor = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val rightSlave = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)


    override fun lateInit() {
        rightSlave.follow(rightMotor)
        leftSlave.follow(leftMotor)
    }

    override fun setNeutral() {
        leftMotor.set(0.0)
        rightMotor.set(0.0)
    }

    override fun periodic() {
        periodicIO.leftCurrent = leftMotor.outputCurrent
        periodicIO.rightCurrent = rightMotor.outputCurrent
        periodicIO.leftVoltage = leftMotor.appliedOutput * leftMotor.busVoltage
        periodicIO.rightVoltage = rightMotor.appliedOutput * rightMotor.busVoltage


        when(wantedState) {
            State.Nothing -> {}
            State.OpenLoop -> {
                leftMotor.set(periodicIO.leftDemand)
                rightMotor.set(periodicIO.rightDemand)
            }
        }
        if(currentState != wantedState) currentState = wantedState
    }


    fun curvatureDrive(
        linearPercent: Double,
        curvaturePercent: Double,
        isQuickTurn: Boolean
    ) {
        val angularPower: Double
        val overPower: Boolean

        if (isQuickTurn) {
            if (linearPercent.absoluteValue < kQuickStopThreshold) {
                quickStopAccumulator = (1 - kQuickStopAlpha) * quickStopAccumulator +
                        kQuickStopAlpha * curvaturePercent.coerceIn(-1.0, 1.0) * 2.0
            }
            overPower = true
            angularPower = curvaturePercent
        } else {
            overPower = false
            angularPower = linearPercent.absoluteValue * curvaturePercent - quickStopAccumulator

            when {
                quickStopAccumulator > 1 -> quickStopAccumulator -= 1.0
                quickStopAccumulator < -1 -> quickStopAccumulator += 1.0
                else -> quickStopAccumulator = 0.0
            }
        }

        var leftMotorOutput = linearPercent + angularPower
        var rightMotorOutput = linearPercent - angularPower

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            when {
                leftMotorOutput > 1.0 -> {
                    rightMotorOutput -= leftMotorOutput - 1.0
                    leftMotorOutput = 1.0
                }
                rightMotorOutput > 1.0 -> {
                    leftMotorOutput -= rightMotorOutput - 1.0
                    rightMotorOutput = 1.0
                }
                leftMotorOutput < -1.0 -> {
                    rightMotorOutput -= leftMotorOutput + 1.0
                    leftMotorOutput = -1.0
                }
                rightMotorOutput < -1.0 -> {
                    leftMotorOutput -= rightMotorOutput + 1.0
                    rightMotorOutput = -1.0
                }
            }
        }

        // Normalize the wheel speeds
        val maxMagnitude = max(leftMotorOutput.absoluteValue, rightMotorOutput.absoluteValue)
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude
            rightMotorOutput /= maxMagnitude
        }

        tankDrive(leftMotorOutput, rightMotorOutput)
    }

    fun tankDrive(left: Double, right: Double) {
        wantedState = State.OpenLoop
        periodicIO.leftDemand = left
        periodicIO.rightDemand = right

    }

    enum class State {
        Nothing,
        OpenLoop
    }

    private class PeriodicIO : Loggable {

        @Log.VoltageView(name = "Left Voltage", width = 2, height = 1, rowIndex = 0, columnIndex = 0)
        var leftVoltage: Double = 0.0
        @Log.VoltageView(name = "Right Voltage", width = 2, height = 1, rowIndex = 0, columnIndex = 2)
        var rightVoltage: Double = 0.0

        @Log(name = "Left Current", width = 2, height = 1, rowIndex = 1, columnIndex = 0)
        var leftCurrent: Double = 0.0
        @Log(name = "Right Current", width = 2, height = 1, rowIndex = 1, columnIndex = 2)
        var rightCurrent: Double = 0.0

        var leftDemand: Double = 0.0
        var rightDemand: Double = 0.0
    }
}