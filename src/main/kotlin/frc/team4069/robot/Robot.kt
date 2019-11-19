package frc.team4069.robot

import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.hid.SaturnHID
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.Logger

object Robot : SaturnRobot() {
    private val loggableSubsystems = mutableListOf<Loggable>()
    private val controllers = mutableListOf<SaturnHID<*>>()

    override fun robotInit() {
        +Drivetrain
        +OI.driveController

        Logger.setCycleWarningsEnabled(false)
        Logger.configureLoggingAndConfig(this, false)
    }

    override fun robotPeriodic() {
        Logger.updateEntries()
        controllers.forEach(SaturnHID<*>::update)
    }

    override operator fun SaturnSubsystem.unaryPlus() {
        addToSubsystemHandler(this)
        if(this is Loggable) {
            loggableSubsystems += this
        }
    }

    operator fun SaturnHID<*>.unaryPlus() {
        controllers += this
    }
}

fun main() {
    Robot.start()
}