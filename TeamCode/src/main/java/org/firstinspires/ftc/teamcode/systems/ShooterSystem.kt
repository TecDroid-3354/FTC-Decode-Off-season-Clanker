package org.firstinspires.ftc.teamcode.systems

import androidx.core.util.Supplier
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.Hood
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.HoodConstants
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.Distance
import java.util.function.BooleanSupplier
import kotlin.math.abs

@Suppress("JoinDeclarationAndAssignment")
class ShooterSystem(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    val distanceToAprilTagInches: Supplier<Double>,
    val isLLResultValid: BooleanSupplier
) {

    // Declaring subsystems
    val indexer: Indexer
    val shooter: Shooter
    val hood: Hood

    // Setting the interpolation & its supplier
    private val hoodInterpolator: ShooterInterpolation
    private val shooterInterpolator: ShooterInterpolation

    var shooterPoint = AngularVelocity.fromRpm(1000.0)
    var hoodPoint = Angle.fromRotations(HoodConstants.Positions.minPosition.rotations)

    init {
        // Starting interpolators
        hoodInterpolator = ShooterInterpolation("hood")
        shooterInterpolator = ShooterInterpolation("shooter")

        // Assigning subsystems
        indexer = Indexer(hardwareMap, telemetry)
        shooter = Shooter(hardwareMap, telemetry)
        hood = Hood(hardwareMap, telemetry)
    }

    fun periodic() {
        if (isLLResultValid.asBoolean) {
            shooterPoint = AngularVelocity.fromRpm(shooterInterpolator.getDesiredPoint(Distance.fromInches(distanceToAprilTagInches.get())))
            hoodPoint = Angle.fromRotations(hoodInterpolator.getDesiredPoint(Distance.fromInches(distanceToAprilTagInches.get())))
        }
    }

    // Command to shoot the Artifacts according to pattern
    fun shoot(motifPatterns: MotifPatterns, velocity: Supplier<AngularVelocity>, angle: Supplier<Angle>) : Command {
        val sequentialCMD = SequentialCommandGroup(
            shooter.setFlyWheelVelocityCMD(velocity.get()),
            hood.setHoodPositionCMD(angle.get()),
            WaitCommand(800),
            InstantCommand({ indexer.feedAllShooter().schedule() }),
            WaitCommand(3000),
            shooter.setFlyWheelVelocityCMD(AngularVelocity.fromRpm(1000.0))
        )

        return sequentialCMD
    }

    // Command to stop the shooter
    fun stopShooter(): Command {
        return shooter.stopCMD()
    }
}