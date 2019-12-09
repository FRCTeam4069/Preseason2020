package frc.team4069.robot

import frc.team4069.saturn.lib.mathematics.units.Fraction
import frc.team4069.saturn.lib.mathematics.units.Meter
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import frc.team4069.saturn.lib.mathematics.units.derived.Acceleration
import frc.team4069.saturn.lib.mathematics.units.derived.Velocity
import frc.team4069.saturn.lib.mathematics.units.derived.Volt
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.mathematics.units.volt

typealias VelocityFeedforward = Fraction<Volt, Velocity<Meter>>
typealias AccelerationFeedforward = Fraction<Volt, Acceleration<Meter>>

object Constants {
    val kLeftDrivetrainUnitModel = NativeUnitLengthModel(1.STU, 3.534.inch)
    val kRightDrivetrainUnitModel = NativeUnitLengthModel(1.STU, 3.684.inch)

    val DRIVETRAIN_KV = SIUnit<VelocityFeedforward>(1.998)
    val DRIVETRAIN_KA = SIUnit<AccelerationFeedforward>(0.76104)
    val DRIVETRAIN_KS = 0.127.volt
}