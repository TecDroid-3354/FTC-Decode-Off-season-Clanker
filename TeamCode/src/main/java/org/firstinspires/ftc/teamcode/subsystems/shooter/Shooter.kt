package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.configurations.OpMotorExConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.genericConfigurations.GenericMotorConfiguration
import org.firstinspires.ftc.teamcode.utils.configurations.motorControlModeConfiguration.MotorVelocityModeConfiguration
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import java.io.FileReader

/**
 * This is the code for controlling the shooter wheels on our robot.
 *
 */
class Shooter(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry
): SubsystemBase() {

    // This is where the motor intended to control the shooter is declared
    lateinit var firstMotor: OpMotorEx
    lateinit var secondMotor: OpMotorEx

    private var flyWheelVelocity = AngularVelocity.fromRpm(0.0)

    // Initialization //

    // This is the code that will execute when the class is initialized
    init {
        motorConfiguration()
    }

    // Periodic method //
    override fun periodic() {
        //setPIDCoefficients()
        //flyWheelVelocity = AngularVelocity.fromRpm(ShooterConstants.Velocity.shooterDesiredVelocity)
        //setFlyWheelVelocity(flyWheelVelocity)
    }

    fun log(pTelemetry: TelemetryManager) {
        pTelemetry.addLine("// SHOOTER //")
        pTelemetry.addData("Shooter velocity", getVelocity().rpm)
        pTelemetry.addData("Target Velocity", flyWheelVelocity.rpm)
        pTelemetry.addData("Error", flyWheelVelocity.rpm - getVelocity().rpm)
    }
    /**
     * Sets the motor's velocity to a desired angular velocity
     */
    private fun setFlyWheelVelocity(velocity: AngularVelocity) {
        firstMotor.setVelocity(velocity)
        secondMotor.setVelocity(velocity)
    }

    fun setFlyWheelVelocityCMD(velocity: AngularVelocity): Command {
        return InstantCommand({
            setFlyWheelVelocity(velocity)
        })
    }

    fun stopCMD(): Command {
        return InstantCommand({
            firstMotor.stopMotor()
            secondMotor.stopMotor()
        })
    }

    private fun setPIDCoefficients() {
        firstMotor.applyModeConfiguration(
            MotorVelocityModeConfiguration()
                .withVelocityCoefficients(ShooterConstants.PIDF.pidfCoefficients)
                .withFeedforwardCoefficients(ShooterConstants.PIDF.feedforward)
        )
        secondMotor.applyModeConfiguration(
            MotorVelocityModeConfiguration()
                .withVelocityCoefficients(ShooterConstants.PIDF.pidfCoefficients)
                .withFeedforwardCoefficients(ShooterConstants.PIDF.feedforward)
        )
    }

    // Getters //

    fun getVelocity(): AngularVelocity {
        return firstMotor.getVelocity().get()
    }

    /**
     * Configures the motor with the given values in the constant sheet
     */
    fun motorConfiguration() {
        // The motor's configuration is grabbed from the constant's file
        firstMotor = OpMotorEx(hardwareMap, ShooterConstants.Identification.firstMotorId)
        firstMotor.applyConfigurationAndResetEncoder(
            OpMotorExConfiguration(
                GenericMotorConfiguration()
                    .withInverted(ShooterConstants.Configuration.direction)
                    .withZeroPowerBehavior(ShooterConstants.Configuration.zeroPowerBehavior)
                    .withTicksPerRev(ShooterConstants.Configuration.ticksPerRotation),
                MotorVelocityModeConfiguration()
                    .withVelocityCoefficients(ShooterConstants.PIDF.pidfCoefficients)
                    .withFeedforwardCoefficients(ShooterConstants.PIDF.feedforward)
            )
        )

        secondMotor = OpMotorEx(hardwareMap, ShooterConstants.Identification.secondMotorId)
        secondMotor.applyConfigurationAndResetEncoder(
            OpMotorExConfiguration(
                GenericMotorConfiguration()
                    .withInverted(ShooterConstants.Configuration.direction)
                    .withZeroPowerBehavior(ShooterConstants.Configuration.zeroPowerBehavior)
                    .withTicksPerRev(ShooterConstants.Configuration.ticksPerRotation),
                MotorVelocityModeConfiguration()
                    .withVelocityCoefficients(ShooterConstants.PIDF.pidfCoefficients)
                    .withFeedforwardCoefficients(ShooterConstants.PIDF.feedforward)
            )
        )
    }
}