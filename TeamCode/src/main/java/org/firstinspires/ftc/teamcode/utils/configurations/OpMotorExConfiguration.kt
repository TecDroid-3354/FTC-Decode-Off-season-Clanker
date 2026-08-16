package org.firstinspires.ftc.teamcode.utils.configurations

import org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration.MotorControlModeConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration.MotorPercentageModeConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.genericConfigurations.GenericMotorConfiguration

class OpMotorExConfiguration(
    var genericMotorConfiguration: GenericMotorConfiguration = GenericMotorConfiguration(),
    var controlModeConfiguration: MotorControlModeConfiguration = MotorPercentageModeConfiguration()
) {

    fun withGenericMotorConfiguration(value: GenericMotorConfiguration): OpMotorExConfiguration {
        this.genericMotorConfiguration = value
        return this
    }

    fun withControlModeConfiguration(value: MotorControlModeConfiguration): OpMotorExConfiguration {
        this.controlModeConfiguration = value
        return this
    }
}