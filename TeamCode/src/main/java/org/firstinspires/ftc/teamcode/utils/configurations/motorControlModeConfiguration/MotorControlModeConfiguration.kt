package org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration

import org.firstinspires.ftc.teamcode.utils.devices.deviceControlMode.MotorControlMode

sealed interface MotorControlModeConfiguration {

    val controlMode: MotorControlMode
}