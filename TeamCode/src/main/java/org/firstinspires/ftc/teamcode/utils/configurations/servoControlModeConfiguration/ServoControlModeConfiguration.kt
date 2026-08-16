package org.firstinspires.ftc.teamcode.utils.configurations.servoControlModeConfiguration

import org.firstinspires.ftc.teamcode.utils.devices.deviceControlMode.ServoControlMode

sealed interface ServoControlModeConfiguration {

    val controlMode: ServoControlMode
}