package org.firstinspires.ftc.teamcode.utils.extensions

import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.teamcode.utils.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.deviceControlMode.MotorControlMode

fun MotorEx.setVelocityCoefficients(pidCoefficients: PIDCoefficients) {
    this.setVeloCoefficients(
        pidCoefficients.p,
        pidCoefficients.i,
        pidCoefficients.d
    )
}

fun MotorEx.setVelocityFeedforward(feedforward: SimpleMotorFeedforward) {
    this.setFeedforwardCoefficients(
        feedforward.ks,
        feedforward.kv,
        feedforward.ka
    )
}

fun MotorEx.applyGenericConfiguration(config: GenericMotorConfiguration, mode: MotorControlMode) {
    this.inverted = config.inverted
    this.setRunMode(when (mode) {
        MotorControlMode.VELOCITY -> Motor.RunMode.VelocityControl
        else -> Motor.RunMode.RawPower
    })
    this.setCachingTolerance(config.cachingTolerance)
    this.setZeroPowerBehavior(config.zeroPowerBehavior)
}