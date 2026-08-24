package org.firstinspires.ftc.teamcode.subsystems.indexer.Slot

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.colorSensor.ColorSensorEx
import org.firstinspires.ftc.teamcode.subsystems.indexer.colorSensor.ColorSensorEx.DetectedColor

data class SlotConfig(
    val servoName: String,
    val isInverted: Boolean,
    val rightCSId: String,
    val leftCSId: String,
    val feedPosition: Double,
    val homePosition: Double,
    val archiveExtension: String
)

@Suppress("JoinDeclarationAndAssignment")
class Slot (val config: SlotConfig, hw: HardwareMap, telemetry: Telemetry) {

    // Declare the slot components
    private var servo: ServoEx
    var rightCS: ColorSensorEx
    var leftCS: ColorSensorEx

    // Initialization code //
    init {
        // Initialize the servo
        servo = ServoEx(hw, config.servoName)
        servo.inverted = config.isInverted

        // Initialize both coclor sensors
        rightCS = ColorSensorEx(
            hw.get(
                ColorSensor::class.java,
                config.rightCSId
            ),
            telemetry,
            config.archiveExtension
        )

        leftCS = ColorSensorEx(hw.get(
            ColorSensor::class.java,
            config.leftCSId),
            telemetry,
            config.archiveExtension)

        awakeServo().schedule()
    }


    /**
     * Compares the reading of both color sensors and if they are the same, it returns the [DetectedColor]
     * @return the [DetectedColor] of the [Slot]
     */
    private fun getDetectedColor(colorSensor: ColorSensorEx): DetectedColor {
        return when {
            colorSensor.hsv[0] in 200.0..300.0 -> DetectedColor.PURPLE
            colorSensor.hsv[1] in 0.5..0.70 -> DetectedColor.GREEN
            else -> DetectedColor.UNKNOWN
        }
    }

    fun getDetectedColor(): DetectedColor {
        return when {
            getDetectedColor(rightCS) == getDetectedColor(leftCS) -> getDetectedColor(leftCS)
            getDetectedColor(rightCS) != DetectedColor.UNKNOWN -> getDetectedColor(rightCS)
            getDetectedColor(leftCS) != DetectedColor.UNKNOWN -> getDetectedColor(leftCS)
            else -> DetectedColor.UNKNOWN
        }
    }

    /**
     * Sets a raw servo position based on the range it physically has
     */
    private fun setServoPosition(position: Double) {
        servo.set(position)
    }

    /**
     * Moves the servo to the feed position
     */
    fun feed() {
        setServoPosition(config.feedPosition)
    }

    /**
     * Moves the servo to the home position
     */
    fun home() {
        setServoPosition(config.homePosition)
    }

    /**
     * Executes a [SequentialCommandGroup] that literally awakens the servo for it to be ready for feeding
     * the shooter. This needs to be called in the initialization code so the servo works correctly
     */
    fun awakeServo(): Command {
        return SequentialCommandGroup(
            InstantCommand({ home() }),
            WaitCommand(500),
            InstantCommand({ setServoPosition(config.homePosition + 0.001)})
        )
    }
}