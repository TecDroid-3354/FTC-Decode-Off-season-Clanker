package org.firstinspires.ftc.teamcode.utils.devices

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.Voltage
import org.firstinspires.ftc.teamcode.utils.configurations.servoControlModeConfiguration.ServoContinuousRotationModeConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.servoControlModeConfiguration.ServoControlModeConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.servoControlModeConfiguration.ServoPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.servoControlModeConfiguration.ServoRunToPositionModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.deviceControlMode.ServoControlMode
import java.util.Optional
import kotlin.math.max
import kotlin.math.min

class OpServoEx(private val hardwareMap: HardwareMap, private val servoId: String) {

    // ------- Control Mode ------ //
    private var controlMode                     : ServoControlMode = ServoControlMode.UNKNOWN
    private var timesConfigured                 : Int              = 0
    // ------- Standard Servo Position Mode useful variables ------ //
    private lateinit var servo                  : ServoEx
    private var range                           : Angle = Angle(0.0)
    private val angleLUT                        : InterpLUT = InterpLUT()

    // ------- Servo Continuous Rotation Mode useful variables ------ //
    private lateinit var crServo                : CRServoEx
    // Max Continuous Rotation Power
    private var maxCRPower                      : Double    = 1.0

    // ------- Servo Run To Position Mode useful variables ------ //
    // Fourth Pin in an Axon Servo
    private lateinit var encoder                : AnalogInput
    // Encoder's empirical max voltage
    private var encoderMaxVoltage               : Voltage   = Voltage(0.0)
    // Absolute encoder offset
    private var encoderOffset                   : Angle     = Angle(0.0)
    // Max Run To Position Power
    private var maxRTPPower                     : Double    = 1.0
    // Keeps track of the total servo rotation without considering gear ratio
    private var totalRotation                   : Angle     = Angle.fromDegrees(0.0)
    // Keeps track of a previous angle in order to perform a subtraction in each iteration
    private var previousAngle                   : Angle     = Angle.fromDegrees(0.0)
    // Used for setting a target rotation
    private var targetRotation                  : Angle     = Angle.fromDegrees(0.0)
    // Keeps track of the number of full rotations in order to obtain the correct servo position through time
    private var fullRotations                   : Int       = 0
    private var gearRatio                       : Double    = 1.0
    // RTP Servo controller
    private var pidfController                  : PIDFController = PIDFController(PIDFCoefficients())

    private var positionLimits                  : ClosedRange<Angle> = Angle(0.0)..Angle(0.0)
    // ----- Commanding wrong control mode exception ----- //
    private val wrongControlModeCommandException: IllegalAccessError = IllegalAccessError(
        "Control mode not configured correctly, double check the Control Mode Configuration given"
    )

    init {
        registry.add(this)
    }

    /**
     * Sets an output considering the maximum power set in the servo's configuration
     * @param output the desired output from - 1 to 1
     */
    private fun setPower(output: Double) {
        val maxPower = when (controlMode) {
            ServoControlMode.CONTINUOUS_ROTATION -> maxCRPower
            ServoControlMode.RUN_TO_POSITION -> maxRTPPower
            else -> 0.0
        }

        val power = max(-maxPower, min(maxPower, output))
        crServo.set(power)
    }

    // TODO() Add comment
    fun setServoPosition(angle: Angle) {
        if (controlMode != ServoControlMode.POSITION) {
            throw wrongControlModeCommandException
        }

        val clampedAngle = angle.degrees.coerceIn(0.0, range.degrees)

        val servoPosition = angleLUT.get(clampedAngle)

        servo.set(servoPosition)
    }

    // TODO() Add comment
    fun setContinuousRotationOutput(output: Double) {
        if (controlMode != ServoControlMode.CONTINUOUS_ROTATION) {
            throw wrongControlModeCommandException
        }

        setPower(output)
    }

    // TODO() Add comment
    fun runToPosition(angle: Angle) {
        if (controlMode != ServoControlMode.RUN_TO_POSITION) {
            throw wrongControlModeCommandException
        }

        val clampedAngle = angle.rotations.coerceIn(positionLimits.start.rotations, positionLimits.endInclusive.rotations)
        val transformedAngle = clampedAngle * gearRatio
        targetRotation = Angle(transformedAngle)
    }

    // TODO() Add comment
    fun updateRunToPositionPIDF(coefficients: PIDFCoefficients, tolerance: Optional<Double>) {
        if (controlMode != ServoControlMode.RUN_TO_POSITION) {
            throw wrongControlModeCommandException
        }

        pidfController.setCoefficients(coefficients)
        if (tolerance.isPresent) {
            pidfController.setTolerance(tolerance.get())
        }
        pidfController.clearTotalError()
    }

    // TODO() Add comment
    fun stop() {
        crServo.set(0.0)
    }

    // TODO() Add comment
    private fun getServoAbsoluteAngle(): Angle {
        val currentAngle = Angle.fromDegrees((encoder.voltage / encoderMaxVoltage.volts) * 360)
        val transformedAngle = currentAngle.minus(encoderOffset)
        return Angle.fromDegrees(normalizeDegrees(transformedAngle.degrees))
    }

    // TODO() Add comment
    fun getAngle(): Angle {
        if (controlMode != ServoControlMode.RUN_TO_POSITION) {
            throw wrongControlModeCommandException
        }

        val transformedAngle = totalRotation.degrees / gearRatio
        val normalizedAngle = normalizeDegrees(transformedAngle)
        return Angle.fromDegrees(normalizedAngle)
    }

    // TODO() Add comment
    private fun updateServo() {
        if (controlMode == ServoControlMode.RUN_TO_POSITION) {
            // Retrieves the current servo angle
            val currentAngle = getServoAbsoluteAngle()
            // Calculates the difference between the current and past servo angle
            val angleDifference = currentAngle.minus(previousAngle)

            // Calculates whether the servo has achieved one rotation since last checked, when the angle difference is below or above half a rotation.
            // Then, updates full rotations
            if (angleDifference > Angle(0.5)) {
                fullRotations--
            } else if (angleDifference < Angle(-0.5)) {
                fullRotations++
            }

            // Calculates the total rotation considering the absolute angle and the achieved rotations
            totalRotation = currentAngle + Angle(fullRotations * 1.0)

            // Keeps track of the previous servo angle
            previousAngle = currentAngle

            // Calculates output
            val output = pidfController.calculate(totalRotation.degrees, targetRotation.degrees)
            // Power request
            setPower(output)
        }
    }

    // TODO() Add comment
    fun applyConfiguration(config: ServoControlModeConfiguration) {
        if (timesConfigured == 0) {
            controlMode = config.controlMode
            timesConfigured++
        } else if (controlMode != config.controlMode) {
            throw IllegalArgumentException("Servos cannot switch control modes while the program is running")
        }

        when (config) {
            is ServoPositionModeConfiguration -> {
                servo = ServoEx(hardwareMap, servoId)
                servo.inverted = config.inverted
                range = config.range
                angleLUT.add(0.0, 0.0)
                angleLUT.add(config.range.degrees, 1.0)
                angleLUT.createLUT()
                servo.set(0.0)
            }
            is ServoContinuousRotationModeConfiguration -> {
                crServo = CRServoEx(hardwareMap, servoId)
                crServo.setRunMode(CRServoEx.RunMode.RawPower)
                crServo.inverted = config.inverted
                maxCRPower = config.maxPower
                setPower(0.0)
            }
            is ServoRunToPositionModeConfiguration -> {
                crServo = CRServoEx(hardwareMap, servoId)
                crServo.setRunMode(CRServoEx.RunMode.RawPower)
                crServo.inverted = config.inverted
                encoder = hardwareMap.get(AnalogInput::class.java, config.absoluteId)
                encoderOffset = config.encoderOffset
                encoderMaxVoltage = config.absoluteMaxVoltage
                pidfController.setCoefficients(config.pidfCoefficients)
                pidfController.setTolerance(config.positionTolerance.rotations)
                positionLimits = config.positionLimits
                maxRTPPower = config.maxPower
                gearRatio = config.gearRatio
                previousAngle = getServoAbsoluteAngle()
                setPower(0.0)
            }
        }
    }

    /**
     * Companion objects belong to the class at top-level, meaning there's just one companion
     * per class creation, not class's instances.
     */
    companion object {
        private val registry = mutableListOf<OpServoEx>()

        /**
         * Calls [OpServoEx.updateServo] on every [OpServoEx] instance that currently exists.
         * Called from [TecDroidRobot] run method to ensure its call no matter what.
         */
        fun updateAll() {
            registry.forEach { it.updateServo() }
        }

        /**
         * Clears the registry of all tracked instances. MUST be called once in the
         * [TecDroidRobot]'s init block, before any [OpServoEx] (or subsystem that creates one)
         * is constructed.
         * Prevents that motors created on previous OpMode are updated in a new one.
         */
        fun clearRegistry() {
            registry.clear()
        }
    }
}