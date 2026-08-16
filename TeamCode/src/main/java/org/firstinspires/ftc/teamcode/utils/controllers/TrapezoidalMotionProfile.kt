package org.firstinspires.ftc.teamcode.utils.controllers

import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import com.seattlesolvers.solverslib.controller.PIDController
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.AngularAcceleration
import org.firstinspires.ftc.teamcode.utils.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration.MotorTrapezoidalModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

class TrapezoidalMotionProfile(val motor: OpMotorEx, val config: MotorTrapezoidalModeConfiguration) {

    // ------- Profile controller ------
    private var controller          : PIDController                     = PIDController(0.0, 0.0, 0.0)
    // ------- Useful variables -------
    private var startPose           : Angle                             = Angle.fromDegrees(0.0)
    private var targetPose          : Angle                             = Angle.fromDegrees(0.0)
    // ------- Profile timer -------
    private val timer               : ElapsedTime                       = ElapsedTime()
    // ------- Profile limits -------
    private var cruiseVelocity      : AngularVelocity                   = AngularVelocity(0.0)
    private var acceleration        : AngularAcceleration               = AngularAcceleration(0.0)

    // Initialization code
    init {
        configureProfile()
    }

    /**
     * Sets a new [PIDCoefficients] to our profile. It also sets the [targetPose] to 0.0 so the controller
     * can keep track of the system's state. Our profile's timer is also reset here.
     * @param pidCoefficients the new [PIDCoefficients] of the profile
     */
    fun setPIDCoefficients(pidCoefficients: PIDCoefficients) {
        targetPose = Angle(0.0)
        controller.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d)
        controller.reset()
        timer.reset()
    }

    fun setPositionTolerance(tolerance: Double) {
        controller.setTolerance(tolerance)
    }

    /**
     * Sets new cruise velocity and max acceleration to this profile. It will affect how the profile reacts to a setpoint.
     * Ensure your values are not too high neither too low.
     * @param newVelocity the new max velocity of the profile
     * @param newAcceleration the new max acceleration of the profile
     */
    fun setProfileLimits(newVelocity: AngularVelocity, newAcceleration: AngularAcceleration) {
        cruiseVelocity = newVelocity
        acceleration = newAcceleration
    }

    /**
     * Sets a new target [Angle] to the profile. If [updateProfile] method is called within a loop, then the motor
     * should try to reachc this setpoint
     */
    fun setTargetAngle(angle: Angle) {
        targetPose = angle
    }

    fun atSetPoint(): Boolean {
        return controller.atSetPoint()
    }

    /**
     * Main method of [TrapezoidalMotionProfile]. Should be called in every loop's iteration for correct functioning.
     */
    fun updateProfile() {
        // Retrieve the timer and profile error
        val time = timer.seconds()
        val error = (targetPose - startPose).rotations

        // Compute acceleration time and distance
        val tAcceleration = cruiseVelocity.rps / acceleration.rotPerSecPerSec
        val dAcceleration = 0.5 * acceleration.rotPerSecPerSec * tAcceleration * tAcceleration

        // Creating useful variables
        var setpoint = 0.0

        // Simplified trapezoidal logic
        if (abs(error) < dAcceleration * 2.0) {
            // Triangular profile if max velocity won't be reached
            val timePeak = sqrt(abs(error) / acceleration.rotPerSecPerSec)
            if (time < timePeak) {
                setpoint = startPose.rotations + 0.5 * acceleration.rotPerSecPerSec * time * time * sign(error)
            } else if (time < timePeak * 2.0) {
                val td = time - timePeak
                setpoint = startPose.rotations + (0.5 * acceleration.rotPerSecPerSec * timePeak * timePeak + acceleration.rotPerSecPerSec * timePeak * td - 0.5 * acceleration.rotPerSecPerSec * td * td) + sign(error)
            } else {
                setpoint = targetPose.rotations
            }
        } else {
            // Full Trapezoidal Profile
            val dCruise = abs(error) - 2.0 * dAcceleration
            val tCruise = dCruise / cruiseVelocity.rps

            if  (time < tAcceleration) {
                setpoint = startPose.rotations + 0.5 * acceleration.rotPerSecPerSec * time * time * sign(error)
            } else if (time < tAcceleration + tCruise) {
                setpoint = startPose.rotations + (dAcceleration * cruiseVelocity.rps * (time - tAcceleration)) * sign(error)
            } else if (time < 2 * tAcceleration + tCruise) {
                val td = time - tAcceleration - tCruise
                setpoint = startPose.rotations + (dAcceleration + dCruise + cruiseVelocity.rps * td - 0.5 * acceleration.rotPerSecPerSec * td * td) * sign(error)
            } else {
                setpoint = targetPose.rotations
            }
        }

        val currentPose = motor.getPosition().get().rotations
        val power = controller.calculate(currentPose, setpoint)
        motor.getMotorInstance().set(power)
    }

    // Configure profile's start position and reset the timer
    private fun configureProfile() {
        cruiseVelocity = config.cruiseVelocity
        acceleration = config.acceleration
        controller = PIDController(config.profileCoefficients.p, config.profileCoefficients.i, config.profileCoefficients.d)
        startPose = motor.getPosition().get()
        timer.reset()
    }
}