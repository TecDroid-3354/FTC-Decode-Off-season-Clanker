package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.utils.AngularVelocity

class ShooterConstants {

    object Identification {
        // This is the motor's Id, it needs to be called in the Driver Hub's configuration
        const val firstMotorId = "firstShooterMotor"
        const val secondMotorId = "secondShooterMotor"
    }

    object Configuration {

        // Whether the motor is inverted
        val direction = true
        // The motor's run mode, in this case as the shooter does not need any encoder position, is set to
        // run without encoder
        val runMode = DcMotor.RunMode.RUN_USING_ENCODER
        // The motor's behavior when is not given any output
        val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT

        const val ticksPerRotation = 28.0
    }

    // It will be used to give the motor the correct velocity to be set to
    @Configurable
    class PIDF {
        companion object {
            @JvmField
            // The PID controller used for the subsytem's motor
            var pidfCoefficients = PIDCoefficients(0.15, 0.0, 0.0)
            @JvmField
            var feedforward = SimpleMotorFeedforward(0.0, 1.18)
            //14.24
        }
    }

    @Configurable
    object Velocity {
        @JvmField
        var shooterDesiredVelocity = 0.0
    }
}