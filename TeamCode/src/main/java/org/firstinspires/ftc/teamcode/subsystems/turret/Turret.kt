package org.firstinspires.ftc.teamcode.subsystems.turret

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.geometry.Vector2d
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Supplier
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.utils.Angle


class Turret(val hw: HardwareMap, val telemetry: Telemetry, val rightRTPServoConfig: RTPServoConfig, val leftRTPServoConfig: RTPServoConfig): SubsystemBase() {

    lateinit var rightServo: RTPServo
    lateinit var leftServo: RTPServo
    lateinit var absoluteEncoder: AnalogInput

    var limits = Angle.fromDegrees(-141.0)..Angle.fromDegrees(120.0)

    init {
        servoConfig()
    }

    override fun periodic() {
        rightServo.periodic()
        leftServo.periodic()
    }

    fun stopTurret(): Command {
        return InstantCommand({
            rightServo.stop()
            leftServo.stop()
        })
    }

    private fun setTurretAngle(angle: Angle) {
        val clampedAngle = angle.rotations.coerceIn(limits.start.rotations, limits.endInclusive.rotations)
        rightServo.setTargetAngle(Angle(clampedAngle))
        leftServo.setTargetAngle(Angle(clampedAngle))
    }

    fun setTurretAngleCMD(angle: Supplier<Angle>): Command {
        return RunCommand({
            setTurretAngle(angle.get())
        })
            .addRequirements(this)
    }

    fun calculateTurretAngle(currentPose: Vector2d, targetPose: Vector2d, robotRotation: Angle): Angle {
        val targetAngle = targetPose.minus(currentPose).angle()
        val targetAngleAsAngle = Angle.fromRadians(targetAngle)
        val turretTarget = targetAngleAsAngle.minus(robotRotation)
        val normalizedAngle = AngleUnit.normalizeDegrees(turretTarget.degrees)
        return Angle.fromDegrees(normalizedAngle)
    }

    // Just need one encoder's reading
    fun getAbsoluteAngle(): Angle {
        return rightServo.getAngle()
    }

    fun servoConfig() {
        // Encoder initialization
        absoluteEncoder = hw.get(AnalogInput::class.java, "abs")

        /* SERVO INITIALIZATION */
        rightServo = RTPServo(hw, telemetry, rightRTPServoConfig, absoluteEncoder)
        rightServo.setPIDFTolerance(Angle.fromDegrees(0.5))


        leftServo = RTPServo(hw, telemetry, leftRTPServoConfig, absoluteEncoder)
        leftServo.setPIDFTolerance(Angle.fromDegrees(0.5))
    }
}