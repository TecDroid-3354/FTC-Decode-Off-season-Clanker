package org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration

import org.firstinspires.ftc.teamcode.utils.devices.deviceControlMode.MotorControlMode

class MotorPercentageModeConfiguration: MotorControlModeConfiguration {

    override val controlMode        : MotorControlMode
        get() = MotorControlMode.PERCENTAGE

    var maxPower                    : Double = 1.0

    fun withMaxPower(value: Double) : MotorPercentageModeConfiguration {
        this.maxPower = value.coerceIn(-1.0, 1.0)
        return this
    }
}