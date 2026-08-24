package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.button.Trigger
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.drive.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret
import org.firstinspires.ftc.teamcode.subsystems.turret.leftServoConfig
import org.firstinspires.ftc.teamcode.subsystems.turret.rightServoConfig
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight
import org.firstinspires.ftc.teamcode.systems.ShooterSystem
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.extensions.leftBumper
import org.firstinspires.ftc.teamcode.utils.extensions.leftTrigger
import org.firstinspires.ftc.teamcode.utils.extensions.onFalse
import org.firstinspires.ftc.teamcode.utils.extensions.onTrue
import org.firstinspires.ftc.teamcode.utils.extensions.rightBumper
import org.firstinspires.ftc.teamcode.utils.extensions.rightTrigger
import org.firstinspires.ftc.teamcode.utils.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.PoseTracker
import org.firstinspires.ftc.teamcode.utils.extensions.start

class Robot(
    private val alliance: Alliance,
    private val hardwareMap: HardwareMap,
    private val controller: GamepadEx,
    private val telemetry: Telemetry
): TecDroidRobot(telemetry, hardwareMap) {

    /* Declare your Pedro Pathing's Follower here */
    private lateinit var follower: Follower
    /* Declare your subsystems here */
    lateinit var drive: Mecanum

    private lateinit var limelight: Limelight

    private lateinit var intake: Intake

    private lateinit var shooterSystem: ShooterSystem

    private lateinit var turret: Turret

    init {
        subsystemInitialization()
    }

    /* Initialize your subsystems and follower here */
    override fun subsystemInitialization() {
        // Follower initialization
        follower = Constants.createFollower(hardwareMap)
        // Subsystem initialization
        drive = Mecanum(follower, controller, alliance)

        limelight = Limelight(hardwareMap, telemetry) {
            Angle.fromDegrees(
                drive.getPose().getHeading(AngleUnit.DEGREES)
            )
        }

        intake = Intake(hardwareMap)

        shooterSystem = ShooterSystem(
            hardwareMap, telemetry,
            { limelight.getDistanceToGoal(intArrayOf(20, 24)).inches },
            { limelight.llResult?.isValid == true && limelight.llResult != null } )

        turret = Turret(hardwareMap, telemetry, rightServoConfig, leftServoConfig)
    }

    /* Runs indefinitely after the init button on the DS is pressed. Stops when play button is pressed */
    override fun initLoop() {}

    /* Initialize your teleop controller commands here */
    override fun initTeleOp() {
        // Chassis default command
        drive.defaultCommand = drive.driveFollowingDriverInput()
        drive.setPose(PoseTracker.pose)

        controller.start()
            .onTrue(InstantCommand({ drive.setPose(Pose()) }))

        controller.rightBumper()
            .onTrue(intake.enableBothIntakes())
            .onFalse(intake.stopBothIntakes())

        controller.leftBumper()
            .onTrue(intake.enableBothOuttakes())
            .onFalse(intake.stopBothIntakes())

        controller.rightTrigger()
            .onTrue(InstantCommand({
                shooterSystem.shoot(
                    limelight.getMotifPattern(),
                    shooterSystem.shooterPoint,
                    shooterSystem.hoodPoint
                ).schedule()
            }))

        controller.leftTrigger()
            .onTrue(
                shooterSystem.shoot(
                    limelight.getMotifPattern(),
                    AngularVelocity.fromRpm(3000.0),
                    Angle.fromRotations(0.7)
                )
            )

        Trigger { limelight.llResult?.isValid == true && limelight.llResult != null }
            .whileActiveContinuous(
                SequentialCommandGroup(
                    turret.alignToAprilTag { limelight.getFilteredTx(alliance) }
                )
            ).whenInactive(
                SequentialCommandGroup(
                    turret.stopTurret()
                )
            )

        // Build Commands:
        // controller.button().onTrue(Command)
    }

    /* Initialize your auto commands here, set chassis alliance and starting pose */
    override fun initAuto(startingPose: Pose) {
        drive.setPose(startingPose)

        Trigger { limelight.llResult?.isValid == true && limelight.llResult != null }
            .whileActiveContinuous(
                SequentialCommandGroup(
                    turret.alignToAprilTag { limelight.getFilteredTx(alliance) }
                )
            ).whenInactive(
                SequentialCommandGroup(
                    turret.stopTurret()
                )
            )
    }

    /* When the teleop ends, declare what to do */
    override fun onEnd() {
        PoseTracker.pose = drive.getPedroPose()
    }

    /* Print telemetry using the pTelemetry object on RobotConstants.Telemetry. It will be printed on both Panels and Driver Hub */
    override fun printTelemetry() {
        pTelemetry.addData("Robot Pose", drive.getPedroPose())
        pTelemetry.addData("turret angle", turret.getAbsoluteAngle().degrees)
        shooterSystem.periodic()
    }

    /* Common method to follow any path */
    override fun followPathCMD(path: PathChain, holdEnd: Boolean, maxPower: Double): Command {
        return drive.followPathCMD(path, holdEnd, maxPower)
    }

    fun getFollower(): Follower  {
        return follower
    }

    fun shootCMD(): Command {
        return InstantCommand({
            shooterSystem.shoot(
                limelight.getMotifPattern(),
                shooterSystem.shooterPoint,
                shooterSystem.hoodPoint
            ).schedule()
        })
    }

    fun isFull(): Boolean {
        return shooterSystem.indexer.isFull()
    }

    fun enableIntake() {
        intake.enableBothIntakes()
    }

    fun disableIntake() {
        intake.stopBothIntakes()
    }
}