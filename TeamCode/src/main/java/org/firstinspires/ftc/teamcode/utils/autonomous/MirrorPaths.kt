package org.firstinspires.ftc.teamcode.utils.autonomous

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees

abstract class MirrorPaths(alliance: Alliance) {

    /** A boolean representing if the path should be mirrored */
    val mirror: Boolean = alliance == Alliance.RED

    /**
     * Mirrors a [Pose] if the set alliance is [Alliance.RED].
     * Returns the same pose otherwise.
     * @param pose the pose to mirror
     * @return the pose
     */
    protected fun mirror(pose: Pose): Pose {
        return if (mirror) pose.mirror() else pose
    }

    /**
     * Mirrors the heading specified. This method is intended to be used within the [com.pedropathing.paths.PathBuilder.setLinearHeadingInterpolation] method.
     * It asks the angle in degrees, so no conversions are needed
     * @param angle the angle to mirror if alliance is red.
     * @return angle in radians.
     */
    protected fun mirrorHeading(angle: Angle): Double {
        return if (mirror) normalizeDegrees(Angle.fromRadians(Math.PI).minus(angle).radians) else angle.radians
    }
}