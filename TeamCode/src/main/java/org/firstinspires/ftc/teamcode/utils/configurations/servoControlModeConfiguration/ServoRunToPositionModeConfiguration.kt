package org.firstinspires.ftc.teamcode.utils.configurations.servoControlModeConfiguration

import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Voltage
import org.firstinspires.ftc.teamcode.utils.devices.deviceControlMode.ServoControlMode

class ServoRunToPositionModeConfiguration: ServoControlModeConfiguration {

    override val controlMode: ServoControlMode
        get() = ServoControlMode.RUN_TO_POSITION

    var absoluteId          : String            = ""

    var absoluteMaxVoltage  : Voltage           = Voltage(3.3)

    var encoderOffset       : Angle             = Angle(0.0)

    var inverted            : Boolean           = false

    var maxPower            : Double            = 1.0

    var gearRatio           : Double            = 1.0

    var pidfCoefficients    : PIDFCoefficients  = PIDFCoefficients(0.005, 0.0, 0.0, 0.0)

    var positionLimits      : ClosedRange<Angle> = Angle(0.0)..Angle(0.1)

    var positionTolerance   : Angle             = Angle(0.2)

    fun withAbsoluteId(value: String)                   : ServoRunToPositionModeConfiguration {
        this.absoluteId = value
        return this
    }

    fun withAbsoluteMaxVoltage(value: Voltage)          : ServoRunToPositionModeConfiguration {
        this.absoluteMaxVoltage = value
        return this
    }

    fun withEncoderOffset(offset: Angle)                : ServoRunToPositionModeConfiguration {
        this.encoderOffset = offset
        return this
    }

    fun withInverted(value: Boolean)                    : ServoRunToPositionModeConfiguration {
        this.inverted = value
        return this
    }

    fun maxPower(value: Double)                         : ServoRunToPositionModeConfiguration {
        this.maxPower = value
        return this
    }

    fun withGearRatio(value: Double)                    : ServoRunToPositionModeConfiguration {
        this.gearRatio = value
        return this
    }

    fun withPIDFCoefficients(value: PIDFCoefficients)   : ServoRunToPositionModeConfiguration {
        this.pidfCoefficients = value
        return this
    }

    fun withPositionLimits(value: ClosedRange<Angle>): ServoRunToPositionModeConfiguration {
        this.positionLimits = value
        return this
    }

    fun withPositionTolerance(value: Angle): ServoRunToPositionModeConfiguration {
        this.positionTolerance = value
        return this
    }
}