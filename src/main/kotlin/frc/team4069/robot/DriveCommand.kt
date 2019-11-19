package frc.team4069.robot

import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.commands.SaturnCommand

class DriveCommand : SaturnCommand(Drivetrain) {

    override fun initialize() {
        Drivetrain.setNeutral()
    }

    override fun execute() {
        Drivetrain.curvatureDrive(OI.driveSpeed, OI.driveTurn, OI.driveSpeed == 0.0)
    }
}