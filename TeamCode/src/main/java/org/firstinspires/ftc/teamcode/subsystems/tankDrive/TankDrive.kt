package org.firstinspires.ftc.teamcode.subsystems.tankDrive

import java.util.function.DoubleSupplier
import androidx.core.util.Supplier
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime

import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.controller.wpilibcontroller.RamseteController
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.DifferentialDriveKinematics
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.DifferentialDriveOdometry
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds
import com.seattlesolvers.solverslib.trajectory.Trajectory

import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.LinearVelocity
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import kotlin.math.abs

class TankDrive(
    private val hardwareMap: HardwareMap,
    private val rotationSupplier: Supplier<Angle>,
    startingPose: Pose2d
): SubsystemBase() {

    // ------ Motor Declaration ------ //
    lateinit var rightMotor: OpMotorEx
    lateinit var leftMotor: OpMotorEx

    // ------ Kinematics and Odometry Initialization ------ //
    private val kinematics = DifferentialDriveKinematics(TankDriveConstants.PhysicalCharacteristics.TRACK_WIDTH.meters)

    private val odometry = DifferentialDriveOdometry(
        Rotation2d(rotationSupplier.get().radians),
        startingPose
    )

    private var ramseteController: RamseteController =
        RamseteController(TankDriveConstants.Autonomous.b, TankDriveConstants.Autonomous.zeta)

    private var turnController  : PIDFController =
        PIDFController(TankDriveConstants.Autonomous.pidfCoefficients)

    /**
     * Called upon the creation of [TankDrive]
     */
    init {
        configureMotor()
    }

    /**
     * Called every loop
     */
    override fun periodic() {
        // Update odometry
        odometry.update(
            Rotation2d(rotationSupplier.get().radians),
            getEncoderDistance(leftMotor).meters,
            getEncoderDistance(rightMotor).meters
        )
        // Update Configurables
        updateConfigurables()
    }

    /**
     * Set a [ChassisSpeeds] to our chassis
     */
    private fun runVelocity(speeds: ChassisSpeeds) {
        // Chassis speeds to wheel speeds
        val wheelSpeeds = kinematics.toWheelSpeeds(speeds)

        val rightVelocity = LinearVelocity.fromMps(wheelSpeeds.rightMetersPerSecond)
        // Each side velocity
        val leftVelocity = LinearVelocity.fromMps(wheelSpeeds.leftMetersPerSecond)

        rightMotor.setVelocity(rightVelocity.toAngularVelocity(TankDriveConstants.PhysicalCharacteristics.WHEEL_RADIUS))
        // Set their respective velocities
        leftMotor.setVelocity(leftVelocity.toAngularVelocity(TankDriveConstants.PhysicalCharacteristics.WHEEL_RADIUS))
    }

    private fun runVelocity(leftVelocity: LinearVelocity, rightVelocity: LinearVelocity) {
        val speeds = kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(leftVelocity.mps, rightVelocity.mps))
        runVelocity(speeds)
    }

    /**
     * Run a [ChassisSpeeds] taking into account the max velocity
     * @param forward the forward velocity obtained from the joystick
     * @param turn the turn velocity obtained from the joystick
     * @return a [RunCommand]
     */
    fun driveCMD(forward: DoubleSupplier, turn: DoubleSupplier): Command {
        return RunCommand({
            // Taking into account the maximum chassis velocity
            val forwardVelocity = forward.asDouble * TankDriveConstants.PhysicalCharacteristics.LINEAR_VELOCITY.mps * TankDriveConstants.Control.FORWARD_MULTIPLIER
            val turnVelocity = turn.asDouble * TankDriveConstants.PhysicalCharacteristics.ANGULAR_VELOCITY.radPerSec * TankDriveConstants.Control.TURN_MULTIPLIER

            runVelocity(
                ChassisSpeeds(
                    forwardVelocity,
                    0.0,
                    turnVelocity
                )
            )
        })
            .addRequirements(this)
    }

    /**
     * Runs a desired [ChassisSpeeds] as a [RunCommand]
     * @param speeds the desired speeds to run
     * @return a [RunCommand] that runs the [ChassisSpeeds]
     */
    fun driveCMD(speeds: ChassisSpeeds): Command {
        return RunCommand({
            runVelocity(speeds)
        })
            .addRequirements(this)
    }

    /**
     * Common [Command] to follow any [Trajectory] with a [TankDrive] type of drivetrain.
     * It is preferred to follow only straight paths and use the [turnToAngle] command to rotate instead.
     * Creates a timer which will be reset each time the command is called.
     * Based on the elapsed time, a sample from the [Trajectory] will be passed into the [ramseteController]
     * and given to a drive velocity function.
     * @param trajectory the desired [Trajectory] to follow
     * @return a [RunCommand] which follows the trajectory
     */
    fun ramsetteCommand(trajectory: Trajectory): Command {
        // Creates a timer
        val timer: ElapsedTime = ElapsedTime()


        return RunCommand({
            val desiredState = trajectory.sample(timer.seconds())
            val desiredSpeeds = ramseteController.calculate(getPose(), desiredState)
            runVelocity(desiredSpeeds)
        })
            .addRequirements(this)
            .beforeStarting(InstantCommand({ timer.reset() }) )
            .interruptOn { timer.seconds() >= trajectory.totalTimeSeconds }
            .whenFinished { runVelocity(ChassisSpeeds()) }
    }

    /**
     * Requests the [TankDrive] to turn to a certain [Angle].
     * Receives an [Angle] and based on the current angle, it applies power through the [turnController].
     * Then, the desired output is torn into a velocity using the max [TankDriveConstants.PhysicalCharacteristics.ANGULAR_VELOCITY].
     * The [AngularVelocity] is converted into a [LinearVelocity] using the [TankDriveConstants.PhysicalCharacteristics.TRACK_WIDTH].
     * Ends when the angular error is less than 2.0 degrees.
     * @param angle the desired angle to turn to.
     * @return A [RunCommand] which turns the robot the desired [angle]
     */
    fun turnToAngle(angle: Angle): Command {
        return RunCommand({
            val power = turnController.calculate(angle.degrees, odometry.poseMeters.rotation.degrees)
            val velocity = power * TankDriveConstants.PhysicalCharacteristics.ANGULAR_VELOCITY.radPerSec
            val linearVelocity = AngularVelocity.fromRadPerSec(velocity)
                .toLinearVelocity(TankDriveConstants.PhysicalCharacteristics.TRACK_WIDTH)
            runVelocity(LinearVelocity(0.0).minus(linearVelocity), linearVelocity)
        })
            .addRequirements(this)
            .beforeStarting(InstantCommand({ turnController.reset() }))
            .interruptOn { abs(angle .degrees - odometry.poseMeters.rotation.degrees) < 2.0 }
            .whenFinished { runVelocity(ChassisSpeeds()) }
    }

    /**
     * Reset the [odometry] with a given [Pose2d]
     * @param pose the new pose
     */
    fun setPose(pose: Pose2d) {
        odometry.resetPosition(pose, Rotation2d(rotationSupplier.get().radians))
    }

    /**
     * Get the motor's encoder [Distance] useful to update odometry
     * @param motor the desired motor to get the distance traveled from.
     * @return the motor's traveled distance
     */
    private fun getEncoderDistance(motor: OpMotorEx): Distance {
        // Wheel's traveled angle
        val wheelTraveledAngle      : Angle     = motor.getPosition().get()
        // Angle to distance using wheel radius
        val wheelTraveledDistance   : Distance  = wheelTraveledAngle.toDistance(TankDriveConstants.PhysicalCharacteristics.WHEEL_RADIUS)
        // Transform to robot
        return  wheelTraveledDistance
    }

    /**
     * @return the robot's [Pose2d]
     */
    fun getPose(): Pose2d {
        return odometry.poseMeters
    }

    /**
     * @return the robot's current [ChassisSpeeds]
     */
    fun getRobotRelativeVelocity(): ChassisSpeeds {
        val leftVelocity = leftMotor.getVelocity().get().toLinearVelocity(TankDriveConstants.PhysicalCharacteristics.WHEEL_RADIUS)
        val rightVelocity = rightMotor.getVelocity().get().toLinearVelocity(TankDriveConstants.PhysicalCharacteristics.WHEEL_RADIUS)

        val speed = kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(leftVelocity.mps, rightVelocity.mps))

        return speed
    }

    /**
     * Update Panels Configurables
     */
    fun updateConfigurables() {
        rightMotor.applyModeConfiguration(
            MotorVelocityModeConfiguration()
                .withVelocityCoefficients(TankDriveConstants.Tunables.rightDriveCoefficients)
                .withFeedforwardCoefficients(TankDriveConstants.Tunables.rightDriveFeedForward)
        )

        leftMotor.applyModeConfiguration(
            MotorVelocityModeConfiguration()
                .withVelocityCoefficients(TankDriveConstants.Tunables.leftDriveCoefficients)
                .withFeedforwardCoefficients(TankDriveConstants.Tunables.rightDriveFeedForward)
        )

        ramseteController = RamseteController(TankDriveConstants.Autonomous.b, TankDriveConstants.Autonomous.zeta)
    }

    fun configureMotor() {
        rightMotor = OpMotorEx(hardwareMap, TankDriveConstants.Identification.RIGHT_MOTOR_ID)
        rightMotor.applyConfigurationAndResetEncoder(TankDriveConstants.Configuration.rightMotorConfiguration)
        leftMotor = OpMotorEx(hardwareMap, TankDriveConstants.Identification.LEFT_MOTOR_ID)
        leftMotor.applyConfigurationAndResetEncoder(TankDriveConstants.Configuration.leftMotorConfiguration)
    }
}