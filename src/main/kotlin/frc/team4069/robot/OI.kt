package frc.team4069.robot

import edu.wpi.first.wpilibj.GenericHID
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.hid.*

object OI {
    val driveController = xboxController(0) {
        button(kBumperLeft) {
            // Low gear while held
            changeOn {
                Drivetrain.gear = Drivetrain.Gear.Low
            }

            changeOff {
                Drivetrain.gear = Drivetrain.Gear.High
            }
        }
    }

    val driveSpeed: Double
        get() {
            val fwd = driveController.getTriggerAxis(GenericHID.Hand.kRight)
            val rev = driveController.getTriggerAxis(GenericHID.Hand.kLeft)

            return fwd - rev
        }

    val driveTurn: Double
        get() = driveController.getX(GenericHID.Hand.kLeft)

    val quickTurnOverride: Boolean
        get() = driveController.getRawButton(kBumperRight.value)
}
