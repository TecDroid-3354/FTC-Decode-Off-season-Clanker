package org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration

import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.AngularAcceleration
import org.firstinspires.ftc.teamcode.utils.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.devices.deviceControlMode.MotorControlMode

class MotorTrapezoidalModeConfiguration: MotorControlModeConfiguration {

    override val controlMode        : MotorControlMode
        get() = MotorControlMode.TRAPEZOIDAL

    var cruiseVelocity              : AngularVelocity       = AngularVelocity(0.0)

    var acceleration                : AngularAcceleration   = AngularAcceleration(0.0)

    var positionTolerance           : Double                = 0.5

    var profileLimits               : ClosedRange<Angle>    = Angle(Double.NEGATIVE_INFINITY)..
            Angle(Double.POSITIVE_INFINITY)

    var profileCoefficients         : PIDCoefficients       = PIDCoefficients()

    fun withCruiseVelocity(value: AngularVelocity)          : MotorTrapezoidalModeConfiguration {
        this.cruiseVelocity         = value
        return this
    }

    fun withAcceleration(value: AngularAcceleration)        : MotorTrapezoidalModeConfiguration {
        this.acceleration           = value
        return this
    }

    fun withPositionTolerance(value: Double)                : MotorTrapezoidalModeConfiguration {
        this.positionTolerance = value
        return this
    }

    fun withProfileLimits(value: ClosedRange<Angle>)        : MotorTrapezoidalModeConfiguration {
        this.profileLimits = value
        return this
    }

    fun withProfileCoefficients(value: PIDCoefficients)     : MotorTrapezoidalModeConfiguration {
        this.profileCoefficients    = value
        return this
    }
}