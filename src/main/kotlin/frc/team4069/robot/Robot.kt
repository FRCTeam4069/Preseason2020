package frc.team4069.robot

import edu.wpi.first.wpilibj.experimental.command.CommandScheduler
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.hid.SaturnHID

object Robot : SaturnRobot() {
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

    operator fun SaturnHID<*>.unaryPlus() {
        controllers += this
    }
}

fun main() {
    Robot.start()
}
