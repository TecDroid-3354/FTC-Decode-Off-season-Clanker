package org.firstinspires.ftc.teamcode.utils.configurations

import org.firstinspires.ftc.teamcode.utils.configurations.servoControlModeConfiguration.ServoControlModeConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.servoControlModeConfiguration.ServoPositionModeConfiguration

class OpServoExConfiguration(
    var servoModeConfiguration: ServoControlModeConfiguration = ServoPositionModeConfiguration()
) {
    fun withServoModeConfiguration(value: ServoControlModeConfiguration): OpServoExConfiguration {
        this.servoModeConfiguration = value
        return this
    }
}