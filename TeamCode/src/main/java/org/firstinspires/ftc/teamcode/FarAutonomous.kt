package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.paths.FarPaths
import org.firstinspires.ftc.teamcode.utils.Alliance

open class FarAutonomous(private val alliance: Alliance): CommandOpMode() {

    // The LogiTech controller
    private lateinit var controller: GamepadEx
    // Robot's declaration
    private lateinit var robot: Robot
    // Declare an instance of your Paths class
    private lateinit var farPaths: FarPaths
    // Autonomous command declaration
    private lateinit var autonomousCommand: SequentialCommandGroup

    // Executed after the init button is pressed.
    override fun initialize() {
        super.reset()
        controller = GamepadEx(gamepad1)
        robot = Robot(alliance, hardwareMap, controller, telemetry)
        // Create an instance of your Paths class //
        farPaths = FarPaths(robot.getFollower(), alliance)

        // Set starting pose based on the autonomous to follow //
        robot.initAuto(farPaths.startingPose)

        // Chain commands inside the autonomous command //
        autonomousCommand = SequentialCommandGroup(
            robot.followPathCMD(farPaths.Path1, false, 1.0),//.alongWith(robot.enableIntake()),
            robot.followPathCMD(farPaths.Path3, true, 1.0),//.alongWith(robot.disableIntake()),
            robot.followPathCMD(farPaths.line3, false, 1.0),//.alongWith(robot.enableIntake()),
            robot.followPathCMD(farPaths.line4, true, 1.0),//.alongWith(robot.disableIntake()),
        )

        waitForStart()

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
    }

    // Executed when the OpMode ends
    override fun end() {
        robot.onEnd()
    }
}