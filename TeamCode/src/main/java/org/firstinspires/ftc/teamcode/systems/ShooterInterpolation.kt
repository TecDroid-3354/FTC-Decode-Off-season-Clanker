package org.firstinspires.ftc.teamcode.systems

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.interpolation.InterpolatingDouble
import org.firstinspires.ftc.teamcode.subsystems.interpolation.InterpolatingTreeMap
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.Distance

// The following data class allows us to declare points that take a distance and return an angle
data class HoodPoint(
    val xDistanceToGoal: Distance,
    val yHoodAngle: Angle
)

data class ShooterPoint(
    val xDistanceToGoal: Distance,
    val yShooterRPM: AngularVelocity
)

class ShooterInterpolation(subsystem: String) {
    // This @maximumSize param represents the max number of allowed elements in the tree map
    // Once this number is surpassed, old data, i.e. the oldest data entries, is removed
    // to allow new entries to come in
    private var map: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> =
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(10)

    // The following points variable is a list containing all the points we want to interpolate between
    private val hoodPoints = listOf(
        // todo: !!!!! SET ALL DEGREES !!!!!!!!!
        HoodPoint(Distance.fromInches(25.7), Angle.fromRotations(0.27)),
        HoodPoint(Distance.fromInches(41.1), Angle.fromRotations(0.39)),
        HoodPoint(Distance.fromInches(67.4), Angle.fromRotations(0.53)),
        HoodPoint(Distance.fromInches(88.0), Angle.fromRotations(0.55)),
        HoodPoint(Distance.fromInches(116.0), Angle.fromRotations(0.52)),
        HoodPoint(Distance.fromInches(142.0), Angle.fromRotations(0.5)),
        HoodPoint(Distance.fromInches(167.0), Angle.fromRotations(0.65)),
        HoodPoint(Distance.fromInches(194.0), Angle.fromRotations(0.72)),
    )

    private val shooterPoints = listOf(
        ShooterPoint(Distance.fromInches(25.7), AngularVelocity.fromRpm(3100.0 - 450.0)),
        ShooterPoint(Distance.fromInches(41.1), AngularVelocity.fromRpm(3200.0 - 450.0)),
        ShooterPoint(Distance.fromInches(67.4), AngularVelocity.fromRpm(3350.0 - 450.0)),
        ShooterPoint(Distance.fromInches(88.0), AngularVelocity.fromRpm(3700.0 - 480.0)),
        ShooterPoint(Distance.fromInches(116.0), AngularVelocity.fromRpm(3940.0 - 550.0)),
        ShooterPoint(Distance.fromInches(142.0), AngularVelocity.fromRpm(4200.0 - 600.0)),
        ShooterPoint(Distance.fromInches(167.0), AngularVelocity.fromRpm(4500.0 - 750.0)),
        ShooterPoint(Distance.fromInches(194.0), AngularVelocity.fromRpm(4600.0 - 750.0)),
    )

    init {
        // todo: test
        when(subsystem) {
            "hood" -> {
                // Adding data to the tree map. It is necessary to use the InterpolatingDouble() since
                // it gives extra functionality to doubles by allowing them to actually interpolate
                for (point in hoodPoints) {
                    map.put(
                        InterpolatingDouble(point.xDistanceToGoal.inches),
                        InterpolatingDouble(point.yHoodAngle.rotations))
                }
            }

            "shooter" -> {
                // Adding data to the tree map. It is necessary to use the InterpolatingDouble() since
                // it gives extra functionality to doubles by allowing them to actually interpolate
                for (point in shooterPoints) {
                    map.put(
                        InterpolatingDouble(point.xDistanceToGoal.inches),
                        InterpolatingDouble(point.yShooterRPM.rpm)
                    )
                }
            }
        }
    }

    fun log(distanceToGoal: Distance, telemetry: Telemetry) {
        telemetry.addData("Distance to AprilTag (Interpolator)", distanceToGoal.inches)
    }

    fun getDesiredPoint(distanceToGoal: Distance): Double {
        return map.getInterpolated(InterpolatingDouble(distanceToGoal.inches)).value
    }
}