package frc.team4069.robot

import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import frc.team4069.saturn.lib.mathematics.units.conversions.feetPerSecond
import frc.team4069.saturn.lib.mathematics.units.velocity

class VelocityPIDTest : SaturnCommand(Drivetrain) {

    val target = 5.feet.velocity

    override fun initialize() {
        val value = target.value
        val arbFF = Constants.DRIVETRAIN_KV * value + Constants.DRIVETRAIN_KS

        Drivetrain.leftMotor.setVelocity(target, arbFF)
        Drivetrain.rightMotor.setVelocity(target, arbFF)
    }

    override fun execute() {
        println("LEFT ${(target - Drivetrain.leftEncoder.velocity).feetPerSecond}. RIGHT ${(target - Drivetrain.rightEncoder.velocity).feetPerSecond}")
    }
}
