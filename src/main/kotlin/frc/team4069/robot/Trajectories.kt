package frc.team4069.robot

import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.constraints.TimingConstraint
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.LinearAcceleration
import frc.team4069.saturn.lib.mathematics.units.conversions.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.conversions.feet

/**
 * Contains all pre-generated trajectories for use in autonomous
 * Also contains logic to generate trajectories on the fly to correct for deviations from expected behaviour in precision-critical maneuvers
 */
object Trajectories {
    private val kMaxVelocity = 2.feet.velocity
    private val kMaxAcceleration = 2.feet.acceleration
    private val kMaxCentripetalAcceleration = 4.feet.acceleration

    private val kConstraints = listOf(
            CentripetalAccelerationConstraint(kMaxCentripetalAcceleration)
    )

    val testTrajectory = waypoints(
        Pose2d(0.feet, 15.feet, 0.degree),
        Pose2d(10.84.feet, 10.189.feet, -15.degree)
    ).generateTrajectory(
        name = "Test Trajectory",
        reversed = false
    )

    private fun waypoints(vararg waypoints: Pose2d) = waypoints.toList()

    private fun List<Pose2d>.generateTrajectory(
            name: String,
            reversed: Boolean,
            startVelocity: SIUnit<LinearVelocity> = 0.meter.velocity,
            maxVelocity: SIUnit<LinearVelocity> = kMaxVelocity,
            maxAcceleration: SIUnit<LinearAcceleration> = kMaxAcceleration,
            constraints: List<TimingConstraint<Pose2dWithCurvature>> = kConstraints
    ): TimedTrajectory<Pose2dWithCurvature> {
        println("Generating $name")
        return DefaultTrajectoryGenerator.generateTrajectory(
                reversed = reversed,
                wayPoints = this,
                constraints = constraints,
                startVelocity = startVelocity,
                endVelocity = 0.meter.velocity,
                maxVelocity = maxVelocity,
                maxAcceleration = maxAcceleration
        )
    }
}