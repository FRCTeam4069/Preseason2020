package frc.team4069.robot

import edu.wpi.first.wpilibj.experimental.command.CommandScheduler
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.hid.SaturnHID
import io.github.oblarg.oblog.Loggable

object Robot : SaturnRobot() {
    private val loggableSubsystems = mutableListOf<Loggable>()
    private val controllers = mutableListOf<SaturnHID<*>>()

    override fun robotInit() {
        +Drivetrain
        +OI.driveController

    }

    override fun robotPeriodic() {
        controllers.forEach(SaturnHID<*>::update)
    }

    override fun teleopInit() {
        DriveCommand().schedule()
    }

    override fun autonomousInit() {
        VelocityPIDTest().schedule()
    }

    override fun disabledPeriodic() {
        CommandScheduler.getInstance().cancelAll()
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
