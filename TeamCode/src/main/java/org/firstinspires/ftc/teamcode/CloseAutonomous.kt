package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.paths.FarPaths
import org.firstinspires.ftc.teamcode.utils.Alliance

open class CloseAutonomous(private val alliance: Alliance): CommandOpMode() {

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
        // TODO

        // Set starting pose based on the autonomous to follow //
        // TODO
        robot.initAuto(Pose())

        // Chain commands inside the autonomous command //
        // TODO
        autonomousCommand = SequentialCommandGroup(

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
    }

    // Executed when the OpMode ends
    override fun end() {
        robot.onEnd()
    }
}