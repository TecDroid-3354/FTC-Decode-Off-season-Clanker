package org.firstinspires.ftc.teamcode.subsystems.turret

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Supplier
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.Angle


class Turret(val hw: HardwareMap, val telemetry: Telemetry, val rightRTPServoConfig: RTPServoConfig, val leftRTPServoConfig: RTPServoConfig): SubsystemBase() {

    lateinit var rightServo: RTPServo
    lateinit var leftServo: RTPServo
    lateinit var absoluteEncoder: AnalogInput

    var limits = Angle.fromDegrees(-141.0)..Angle.fromDegrees(120.0)

    var appliedPower = 0.0

    var pidController = PIDFController(PIDFCoefficients(0.021, 0.0, 0.0, 0.0))

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

    fun setTurretVoltage(power: Double) {
        if ((getAbsoluteAngle().degrees <= limits.start.degrees && power < 0.0) ||
            (getAbsoluteAngle().degrees >= limits.endInclusive.degrees && power > 0.0)) {
            rightServo.stop()
            leftServo.stop()
        } else {
            appliedPower = power
            rightServo.setPower(power)
            leftServo.setPower(power)
        }
    }

    fun alignToAprilTag(tx: Supplier<Angle>): Command {
        return RunCommand({
            val power = pidController.calculate(tx.get().degrees, 0.0)
            setTurretVoltage(power)
        })
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