package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathPoint
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.mecanumDrive.Mecanum
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.autonomous.PoseTracker
import org.firstinspires.ftc.teamcode.utils.extensions.toPose

class Robot(
    private val alliance: Alliance,
    private val hardwareMap: HardwareMap,
    private val controller: GamepadEx,
    telemetry: Telemetry
): TecDroidRobot(telemetry, hardwareMap) {

    /* Declare your Pedro Pathing's Follower here */
    private lateinit var follower: Follower
    /* Declare your subsystems here */
    private lateinit var drive: Mecanum

    init {
        subsystemInitialization()
    }

    /* Initialize your subsystems and follower here */
    override fun subsystemInitialization() {
        // Follower initialization
        follower = Constants.createFollower(hardwareMap)
        // Subsystem initialization
        drive = Mecanum(follower, controller, alliance)
    }

    /* Runs indefinitely after the init button on the DS is pressed. Stops when play button is pressed */
    override fun initLoop() {}

    /* Initialize your teleop controller commands here */
    override fun initTeleOp() {
        // Chassis default command
        drive.defaultCommand = drive.driveFollowingDriverInput()
        drive.setPose(PoseTracker.lastPose)
        // Build Commands:
        // controller.button().onTrue(Command)
    }

    /* Initialize your auto commands here, set chassis alliance and starting pose */
    override fun initAuto(startingPose: Pose) {
        drive.setPose(startingPose)
    }

    /* When the teleop ends, declare what to do */
    override fun onEnd() {
        PoseTracker.lastPose = drive.getPose2D().toPose()
    }

    /* Print telemetry using the pTelemetry object on RobotConstants.Telemetry. It will be printed on both Panels and Driver Hub */
    override fun printTelemetry() {
        pTelemetry.addData("Path Following", follower.currentPath.getHeadingGoal(PathPoint()))
        pTelemetry.addData("Robot Pose", follower.pose)
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
}