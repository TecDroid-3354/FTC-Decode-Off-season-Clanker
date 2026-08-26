package org.firstinspires.ftc.teamcode.subsystems.mecanumDrive

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.PathChain
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.extensions.toPose2D

class Mecanum(
    private val follower: Follower,
    private val controller: GamepadEx,
    private val alliance: Alliance
): SubsystemBase() {

    /**
     * Runs in every loop. Follower and telemetry get updated
     */
    override fun periodic() {
        follower.update()
    }

    /**
     * Retrieves the [controller]'s axis readings and converts them into robot's velocity.
     * The axis get multiplied by each [MecanumConstants.Control] Multiplier and its respective alliance multiplier.
     * @return a [RunCommand] which set the [Follower]'s TeleOp drive to the [controller]'s axis.
     */
    fun driveFollowingDriverInput(): Command {
        return RunCommand({
            follower.setTeleOpDrive(
                controller.leftX * MecanumConstants.Control.FORWARD_VELOCITY_MULTIPLIER * alliance.multiplier,
                controller.leftY * MecanumConstants.Control.LATERAL_VELOCITY_MULTIPLIER * alliance.multiplier,
                controller.rightX * MecanumConstants.Control.TURN_VELOCITY_MULTIPLIER,
                false
            )
        }, this)
            .beforeStarting(InstantCommand({ follower.startTeleopDrive(MecanumConstants.Control.IS_BRAKE_MODE) }))
            .whenFinished  { follower.breakFollowing() }
    }

    /**
     * Gets the Follower's current position.
     * @return a [Pose2D] containing the robot's current position in the standard FTC Coordinates
     */
    fun getPose2D(): Pose2D {
        return follower.pose.toPose2D()
    }

    /**
     * Gets the [Follower]'s rotation component.
     * @return a [Rotation2d] as the robot's current heading in radians.
     */
    fun getRotation(): Rotation2d {
        return Rotation2d(getPose2D().getHeading(AngleUnit.RADIANS))
    }

    /**
     * Gets the current [Follower]'s velocity as a [Vector]
     * @return the current robot's velocity
     */
    fun getVelocity(): Vector {
        return follower.velocity
    }

    /**
     * Constructs a new [FollowPathCommand] based on the given parameters
     * @param path the desired path to follow
     * @param holdEnd if the path is required to hold its end at the end of the path
     * @param maxPower the maximum achievable power by the robot along the path
     * @return a new [FollowPathCommand] with the given parameters
     */
    fun followPathCMD(path: PathChain, holdEnd: Boolean, maxPower: Double): Command {
        return FollowPathCommand(follower, path, holdEnd, maxPower)
    }

    /**
     * Sets a new [Pose2D] to our robot's chassis.
     * @param pose a pose representing the new robot's [Pose2D]. Note that it must be in STANDARD FTC coordinates.
     */
    fun setPose(pose: Pose) {
        follower.pose = pose
    }
}