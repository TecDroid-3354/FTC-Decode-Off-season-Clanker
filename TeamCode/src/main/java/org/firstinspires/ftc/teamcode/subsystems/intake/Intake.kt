package org.firstinspires.ftc.teamcode.subsystems.intake

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration.MotorPercentageModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx

class Intake(
    val hardwareMap: HardwareMap
) : SubsystemBase() {
    // Consider that right & left motors refer to the motors as seen from the turret
    lateinit var frontMotor: OpMotorEx
    lateinit var backMotor: OpMotorEx

    init {
        motorConfig()
    }

    /**
     * This is the method called in the [CMDOpMode]. It calls [enableIntake] twice and turns on both intakes
     */
    fun enableBothIntakes(): Command {
        return InstantCommand({
            frontMotor.setPower(1.0)
            backMotor.setPower(1.0)
        })
    }

    fun enableBothOuttakes(): Command {
        return InstantCommand({
            frontMotor.setPower(-1.0)
            backMotor.setPower(-1.0)
        })
    }

    /**
     * Quite literally stops both motors
     */
    fun stopBothIntakes(): Command {
        return InstantCommand({
            frontMotor.setPower(0.0)
            backMotor.setPower(0.0)
        })
    }

    // Setup code //

    // Configuring motors with the custom VelocityEx class
    private fun motorConfig() {
        frontMotor = OpMotorEx(hardwareMap, IntakeConstants.Identification.frontMotorId)
        frontMotor.applyConfigurationAndResetEncoder(
            OpMotorExConfiguration(
                GenericMotorConfiguration()
                    .withInverted(IntakeConstants.Configuration.frontInverted)
                    .withZeroPowerBehavior(IntakeConstants.Configuration.zeroPowerBehavior),
                MotorPercentageModeConfiguration()
            )
        )

        backMotor = OpMotorEx(hardwareMap, IntakeConstants.Identification.backMotorId)
        backMotor.applyConfigurationAndResetEncoder(
            OpMotorExConfiguration(
                GenericMotorConfiguration()
                    .withInverted(IntakeConstants.Configuration.backInverted)
                    .withZeroPowerBehavior(IntakeConstants.Configuration.zeroPowerBehavior),
                MotorPercentageModeConfiguration()
            )
        )
    }
}