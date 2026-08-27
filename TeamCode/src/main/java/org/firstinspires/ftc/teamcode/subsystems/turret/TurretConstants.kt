package org.firstinspires.ftc.teamcode.subsystems.turret

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Voltage

@Configurable
class TurretConstants {

    object Identification {

        object RightServo {

            val servoId = "rightServo"
            val absoluteId = "rightAbs"
        }

        object LeftServo {

            val servoId = "leftServo"
            val absoluteId = "leftAbs"
        }
    }

    object Configuration {

        val direction = RTPServo.Direction.FORWARD

        object AbsoluteEncoder {

            val maximumVoltage = Voltage.fromVolts(3.23)
            val offset = Angle.fromDegrees(-140.0 - 58)
        }
    }

    object PhysicalDescription {
        const val gearRatio = 1.0 / 5.5
    }

    companion object {
        @JvmField
        var turretControllerCoefficients = PIDFCoefficients(0.021, 0.0, 0.0, 0.0)
    }
}

val rightServoConfig = RTPServoConfig(
    TurretConstants.Identification.RightServo.servoId,
    TurretConstants.Identification.RightServo.absoluteId,
    TurretConstants.Configuration.AbsoluteEncoder.maximumVoltage,
    TurretConstants.Configuration.direction,
    TurretConstants.Configuration.AbsoluteEncoder.offset,
    1.0,
    TurretConstants.PhysicalDescription.gearRatio,
    TurretConstants.turretControllerCoefficients
)

val leftServoConfig = RTPServoConfig(
    TurretConstants.Identification.LeftServo.servoId,
    TurretConstants.Identification.LeftServo.absoluteId,
    TurretConstants.Configuration.AbsoluteEncoder.maximumVoltage,
    TurretConstants.Configuration.direction,
    TurretConstants.Configuration.AbsoluteEncoder.offset,
    1.0,
    TurretConstants.PhysicalDescription.gearRatio,
    TurretConstants.turretControllerCoefficients
)