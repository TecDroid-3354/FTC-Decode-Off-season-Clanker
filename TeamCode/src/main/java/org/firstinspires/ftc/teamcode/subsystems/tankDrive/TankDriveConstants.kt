package org.firstinspires.ftc.teamcode.subsystems.tankDrive

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.utils.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.LinearVelocity
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration

object TankDriveConstants {

    object Identification {
        const val RIGHT_MOTOR_ID = "rightMotor"
        const val LEFT_MOTOR_ID = "leftMotor"
    }

    object PhysicalCharacteristics {
        const val GEAR_RATIO = 1.0
        const val COUNTS_PER_REV = 28.0
        val TRACK_WIDTH = Distance.fromMeters(0.0)
        val WHEEL_RADIUS = Distance.fromInches(0.0)
        val MAX_VELOCITY = AngularVelocity.fromRpm(6000.0 / GEAR_RATIO)
        val LINEAR_VELOCITY = LinearVelocity(MAX_VELOCITY.radPerSec * WHEEL_RADIUS.meters)
        val ANGULAR_VELOCITY = AngularVelocity.fromRadPerSec(LINEAR_VELOCITY.mps / WHEEL_RADIUS.meters)
    }

    @Configurable
    object Tunables {
        @JvmField
        var leftDriveCoefficients = PIDCoefficients(0.05, 0.0, 0.0)
        @JvmField
        var leftDriveFeedForward = SimpleMotorFeedforward(0.0, 0.1, 0.0)
        @JvmField
        var rightDriveCoefficients = PIDCoefficients(0.05, 0.0, 0.0)
        @JvmField
        var rightDriveFeedForward = SimpleMotorFeedforward(0.0, 0.1, 0.0)
        @JvmField
        var leftMotorInverted = true
        @JvmField
        var rightMotorInverted = false
    }

    object Configuration {
        private val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT

        private val rightMotorGenericConfiguration: GenericMotorConfiguration =
            GenericMotorConfiguration()
                .withInverted(Tunables.rightMotorInverted)
                .withGearRatio(PhysicalCharacteristics.GEAR_RATIO)
                .withZeroPowerBehavior(zeroPowerBehavior)
                .withTicksPerRev(PhysicalCharacteristics.COUNTS_PER_REV)

        private val leftMotorGenericConfiguration: GenericMotorConfiguration =
            rightMotorGenericConfiguration.withInverted(Tunables.leftMotorInverted)

        private val rightMotorModeConfiguration: MotorVelocityModeConfiguration =
            MotorVelocityModeConfiguration()
                .withVelocityCoefficients(Tunables.rightDriveCoefficients)
                .withFeedforwardCoefficients(Tunables.rightDriveFeedForward)

        private val leftMotorModeConfiguration: MotorVelocityModeConfiguration =
            MotorVelocityModeConfiguration()
                .withVelocityCoefficients(Tunables.leftDriveCoefficients)
                .withFeedforwardCoefficients(Tunables.leftDriveFeedForward)


        val rightMotorConfiguration: OpMotorExConfiguration =
            OpMotorExConfiguration()
                .withGenericMotorConfiguration(rightMotorGenericConfiguration)
                .withControlModeConfiguration(rightMotorModeConfiguration)

        val leftMotorConfiguration: OpMotorExConfiguration =
            OpMotorExConfiguration()
                .withGenericMotorConfiguration(leftMotorGenericConfiguration)
                .withControlModeConfiguration(leftMotorModeConfiguration)
    }

    object Control {
        //This value get multiplied t
        const val TURN_MULTIPLIER = 1.0
        const val FORWARD_MULTIPLIER = 1.0
    }

    @Configurable
    object Autonomous {
        @JvmField
        var b           : Double = 10.0
        @JvmField
        var zeta        : Double = 0.7
        @JvmField
        var pidfCoefficients: PIDFCoefficients = PIDFCoefficients(0.1, 0.0, 0.0, 0.0)
    }
}