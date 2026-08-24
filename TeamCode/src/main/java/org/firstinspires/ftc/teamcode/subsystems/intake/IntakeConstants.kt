package org.firstinspires.ftc.teamcode.subsystems.intake

import com.seattlesolvers.solverslib.hardware.motors.Motor

class IntakeConstants {

    object Identification {
        // This is the right motor's Id, it needs to be called in the Driver Hub's configuration
        const val frontMotorId = "frontIntakeMotor"
        // This is the left motor's Id, it needs to be called in the Driver Hub's configuration
        const val backMotorId = "backIntakeMotor"
    }

    object Configuration {

        // The motor's behavior when is not given any output
        val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        // If the right motor is inverted
        val frontInverted = true
        // If the left motor is inverted
        val backInverted = false
    }
}