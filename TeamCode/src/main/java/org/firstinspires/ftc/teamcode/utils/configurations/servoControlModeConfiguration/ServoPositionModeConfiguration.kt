package org.firstinspires.ftc.teamcode.utils.configurations.servoControlModeConfiguration

import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.devices.deviceControlMode.ServoControlMode

class ServoPositionModeConfiguration: ServoControlModeConfiguration {

    override val controlMode: ServoControlMode
        get() = ServoControlMode.POSITION

    var range                   : Angle                 = Angle(0.0)

    var inverted                : Boolean               = false

    fun withRange(value: Angle)   : ServoPositionModeConfiguration {
        this.range = value
        return this
    }

    fun withInverted(value: Boolean)            : ServoPositionModeConfiguration {
        this.inverted = value
        return this
    }
}