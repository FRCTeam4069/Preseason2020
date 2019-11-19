package frc.team4069.robot

import edu.wpi.first.wpilibj.GenericHID
import frc.team4069.saturn.lib.hid.getTriggerAxis
import frc.team4069.saturn.lib.hid.getX
import frc.team4069.saturn.lib.hid.xboxController

object OI {
    val driveController = xboxController(0) {

    }

    val driveSpeed: Double
        get() {
            val fwd = driveController.getTriggerAxis(GenericHID.Hand.kRight)
            val rev = driveController.getTriggerAxis(GenericHID.Hand.kLeft)

            return fwd - rev
        }

    val driveTurn: Double
        get() = driveController.getX(GenericHID.Hand.kLeft)
}
