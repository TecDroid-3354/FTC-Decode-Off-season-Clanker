package org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration

import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.devices.deviceControlMode.MotorControlMode

class MotorPositionModeConfiguration: MotorControlModeConfiguration {

    override val controlMode        : MotorControlMode
        get() = MotorControlMode.POSITION

    var positionCoefficients        : PIDFCoefficients                  = PIDFCoefficients()

    var positionLimits              : ClosedRange<Angle>                =
        Angle(Double.NEGATIVE_INFINITY)..Angle(Double.POSITIVE_INFINITY)

    var positionTolerance           : Double                            = 0.5

    fun withPIDFCoefficients(value: PIDFCoefficients)                   : MotorPositionModeConfiguration {
        this.positionCoefficients = value
        return this
    }

    fun withPositionLimits(value: ClosedRange<Angle>)       : MotorPositionModeConfiguration {
        this.positionLimits = value
        return this
    }

    fun withPositionTolerance(value: Double)                : MotorPositionModeConfiguration {
        this.positionTolerance = value
        return this
    }
}