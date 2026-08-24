package org.firstinspires.ftc.teamcode.subsystems.shooter.hood

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.Angle

/**
 * Class intended to control the Hood Subsystem, which is mounted in the shooter.
 * @param hardwareMap contains the hardware devices of the robot. Configured through Driver Hub.
 * @param telemetry to print important data in real-time through Driver Hub.
 */
@Suppress("JoinDeclarationAndAssignment")
class Hood(val hardwareMap: HardwareMap, val telemetry: Telemetry): SubsystemBase() {
    private val servo: ServoEx
    var currentAngle: Angle

    // Initialization code //
    init {
        servo = ServoEx(hardwareMap, HoodConstants.Identification.hoodId)
        servoConfig()
        setHoodPosition(HoodConstants.Positions.minPosition + Angle.fromRotations(0.0001))

        // servo.position returns a value from 0.0 to 1.0, we take it as rotations.
        currentAngle = Angle.fromRotations(servo.servo.position)
    }

    // Code called every robot loop //
    override fun periodic() {
        // Telemetry to retrieve useful data
        currentAngle = Angle.fromRotations(servo.servo.position)
    }

    fun log() {
        telemetry.addData("HoodPositionRotations", currentAngle.rotations)
    }

    // Sets the desired angle to the servo (in radians) and updates the currentAngle variable //
    private fun setHoodPosition(position: Angle) {
        val clampedPosition = MathUtils.clamp(position.rotations, HoodConstants.Positions.minPosition.rotations,
            HoodConstants.Positions.maxPosition.rotations)
        servo.set(clampedPosition) // Per documentation, servo.set() requires radians
    }

    fun setHoodPositionCMD(position: Angle): Command {
        return InstantCommand({setHoodPosition(position)})
    }

    fun modifyCurrentPositionBy(factor: Angle) {
        setHoodPosition(Angle.fromRotations(servo.servo.position) + factor)
    }

    // Setup code //
    private fun servoConfig() {
        // Configures whether to invert the servo direction
        servo.inverted = HoodConstants.Configuration.hoodServoInverted
    }
}