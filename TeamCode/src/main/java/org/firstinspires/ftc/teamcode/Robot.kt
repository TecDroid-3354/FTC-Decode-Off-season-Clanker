package org.firstinspires.ftc.teamcode

import android.os.Binder
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathPoint
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.command.button.Trigger
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Vector2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.mecanumDrive.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret
import org.firstinspires.ftc.teamcode.subsystems.turret.leftServoConfig
import org.firstinspires.ftc.teamcode.subsystems.turret.rightServoConfig
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight
import org.firstinspires.ftc.teamcode.systems.ShooterSystem
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.AngularVelocity
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.autonomous.PoseTracker
import org.firstinspires.ftc.teamcode.utils.extensions.a
import org.firstinspires.ftc.teamcode.utils.extensions.b
import org.firstinspires.ftc.teamcode.utils.extensions.circle
import org.firstinspires.ftc.teamcode.utils.extensions.h
import org.firstinspires.ftc.teamcode.utils.extensions.leftBumper
import org.firstinspires.ftc.teamcode.utils.extensions.leftTrigger
import org.firstinspires.ftc.teamcode.utils.extensions.onFalse
import org.firstinspires.ftc.teamcode.utils.extensions.onTrue
import org.firstinspires.ftc.teamcode.utils.extensions.rightBumper
import org.firstinspires.ftc.teamcode.utils.extensions.rightTrigger
import org.firstinspires.ftc.teamcode.utils.extensions.start
import org.firstinspires.ftc.teamcode.utils.extensions.toPose
import org.firstinspires.ftc.teamcode.utils.extensions.triangle
import org.firstinspires.ftc.teamcode.utils.extensions.x
import org.firstinspires.ftc.teamcode.utils.extensions.y

class Robot(
    private val alliance: Alliance,
    private val hardwareMap: HardwareMap,
    private val controller: GamepadEx,
    private val telemetry: Telemetry
): TecDroidRobot(telemetry, hardwareMap) {

    /* Declare your Pedro Pathing's Follower here */
    private lateinit var follower: Follower
    /* Declare your subsystems here */
    private lateinit var drive: Mecanum

    private lateinit var limelight: Limelight

    private lateinit var intake: Intake

    private lateinit var shooterSystem: ShooterSystem

    private lateinit var turret: Turret

    private var turretTargetAngle: Angle = Angle(0.0)

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
            Angle.fromRadians(
                drive.getPose2D().getHeading(AngleUnit.RADIANS)
            )
        }

        intake = Intake(hardwareMap)

        shooterSystem = ShooterSystem(
            hardwareMap, telemetry,
            { drive.getDistanceTo(
                when (alliance) {
                Alliance.RED -> Pose(144.0, 144.0)
                Alliance.BLUE -> Pose(0.0, 144.4)
            }).inches
            },
            { limelight.llResult?.isValid == true && limelight.llResult != null } )

        turret = Turret(hardwareMap, telemetry, rightServoConfig, leftServoConfig)
    }

    /* Runs indefinitely after the init button on the DS is pressed. Stops when play button is pressed */
    override fun initLoop() {}

    /* Initialize your teleop controller commands here */
    override fun initTeleOp() {
        // Chassis default command
        drive.defaultCommand = drive.driveFollowingDriverInput()
        drive.setPose(PoseTracker.lastPose)

        turret.defaultCommand = turret.setTurretAngleCMD { turretTargetAngle }
        // Build Commands:
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
                    { shooterSystem.shooterPoint },
                    { shooterSystem.hoodPoint }
                ).schedule()
            }))

        controller.leftTrigger()
            .onTrue(
                shooterSystem.shoot(
                    limelight.getMotifPattern(),
                    { AngularVelocity.fromRpm(3000.0) },
                    { Angle.fromRotations(0.7) }
                )
            )

        controller.triangle()
            .onTrue(turret.setTurretAngleCMD({ Angle.fromDegrees(90.0) }))

        controller.circle()
            .onTrue(turret.setTurretAngleCMD({ Angle.fromDegrees(-90.0) }))
    }

    /* Initialize your auto commands here, set chassis alliance and starting pose */
    override fun initAuto(startingPose: Pose) {
        drive.setPose(startingPose)
        turret.defaultCommand = turret.setTurretAngleCMD { turretTargetAngle }
    }

    /* When the teleop ends, declare what to do */
    override fun onEnd() {
        PoseTracker.lastPose = follower.pose
    }

    /* Print telemetry using the pTelemetry object on RobotConstants.Telemetry. It will be printed on both Panels and Driver Hub */
    override fun printTelemetry() {
        shooterSystem.periodic()
        turretTargetAngle = turret.calculateTurretAngle(
            Vector2d(
                drive.getPose2D().x,
                drive.getPose2D().y
            ),
            when (alliance) {
                Alliance.BLUE -> Vector2d(-58.0, -58.0)
                Alliance.RED -> Vector2d(-58.0, 58.0)
            },
            Angle.fromRadians(drive.getPose2D().h)
        ).plus(Angle.fromDegrees(90.0))
        pTelemetry.addData("turret angle", turret.getAbsoluteAngle().degrees)
        pTelemetry.addData("Target turret angle", turretTargetAngle.degrees)
        pTelemetry.addData("Drive pose", drive.getPose2D())
        pTelemetry.addData("Follower pose", getFollower().pose)
//        pTelemetry.addData("Distance to target", drive.getDistanceTo(
//            when (alliance) {
//                Alliance.RED -> Pose(144.0, 144.0)
//                Alliance.BLUE -> Pose(0.0, 144.4)
//            }).inches)
        pTelemetry.addData("Shooter velocity rpm", shooterSystem.shooter.getVelocity().rpm)
    }

    /**
     * @return the Pedro's Follower
     */
    override fun getFollower(): Follower {
        return follower
    }

    /* Common method to follow any path */
    override fun followPathCMD(path: PathChain, holdEnd: Boolean, maxPower: Double): Command {
        return drive.followPathCMD(path, holdEnd, maxPower)
    }

    fun shootCMD(): Command {
        val timer: ElapsedTime = ElapsedTime()

        return shooterSystem.shoot(limelight.getMotifPattern(), { AngularVelocity(40.0) }) { Angle(0.8) }
            .beforeStarting(InstantCommand({ timer.reset() }) )
            .interruptOn { timer.seconds() > 5.0}
            .whenFinished { stopShooter() }
    }

    fun enableIntake(): Command {
        return intake.enableBothIntakes()
    }

    fun disableIntake(): Command {
        return intake.stopBothIntakes()
    }

    fun stopShooter(): Command {
        return shooterSystem.stopShooter()
    }

    fun isFull(): Boolean {
        return shooterSystem.indexer.isFull()
    }
}