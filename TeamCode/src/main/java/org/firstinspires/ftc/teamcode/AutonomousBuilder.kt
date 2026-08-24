package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.command.button.Trigger
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.paths.Paths
import org.firstinspires.ftc.teamcode.utils.Alliance

open class AutonomousBuilder(private val alliance: Alliance): CommandOpMode() {

    // The LogiTech controller
    private lateinit var controller: GamepadEx
    // Robot's declaration
    private lateinit var robot: Robot


    private lateinit var paths: Paths
    // Autonomous command declaration
    private lateinit var autonomousCommand: SequentialCommandGroup

    // Executed after the init button is pressed.
    override fun initialize() {
        super.reset()
        controller = GamepadEx(gamepad1)
        robot = Robot(alliance, hardwareMap, controller, telemetry)
        paths = Paths(robot.getFollower(), alliance)
        robot.initAuto(paths.startingPose)

        autonomousCommand = SequentialCommandGroup(
            WaitCommand(4000),
            robot.followPathCMD(paths.Path1, false, 1.0),
            robot.followPathCMD(paths.Path2, true, 0.8),
            robot.shootCMD(),
            WaitUntilCommand{  robot.isFull().not() },
            robot.followPathCMD(paths.line3, true, 1.0)
        )

        // Schedule autonomous command
        autonomousCommand.schedule()
    }

    // Executed indefinitely after the init button is pressed
    override fun initialize_loop() {
        robot.initLoop()
    }

    // Executed after the play button is pressed
    override fun run() {
        robot.run()
        robot.pTelemetry.addData("Robot pose", robot.drive.getPose())
    }

    // Executed when the OpMode ends
    override fun end() {
        robot.onEnd()
    }
}