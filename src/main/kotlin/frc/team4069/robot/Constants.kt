package frc.team4069.robot

import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU

object Constants {
    val kLeftDrivetrainUnitModel = NativeUnitLengthModel(1.STU, 3.534.inch)
    val kRightDrivetrainUnitModel = NativeUnitLengthModel(1.STU, 3.684.inch)

    const val DRIVETRAIN_KV = 1.998 // V/(m/s)
    const val DRIVETRAIN_KA = 0.76104 // 1/(m/s^2)
    const val DRIVETRAIN_KS = 0.127 // V
}