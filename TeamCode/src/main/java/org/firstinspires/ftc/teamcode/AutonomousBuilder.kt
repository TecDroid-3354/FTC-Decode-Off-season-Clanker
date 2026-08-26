package org.firstinspires.ftc.teamcode

import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.paths.Curve
import org.firstinspires.ftc.teamcode.utils.Alliance

open class AutonomousBuilder(private val alliance: Alliance): CommandOpMode() {

    // The LogiTech controller
    private lateinit var controller: GamepadEx
    // Robot's declaration
    private lateinit var robot: Robot

    private lateinit var curve: Curve
    // Autonomous command declaration
    private lateinit var autonomousCommand: SequentialCommandGroup

    // Executed after the init button is pressed.
    override fun initialize() {
        super.reset()
        controller = GamepadEx(gamepad1)
        robot = Robot(alliance, hardwareMap, controller, telemetry)
        curve = Curve(robot.getFollower(), alliance)
        // TODO Set correct starting pose
        robot.initAuto(curve.startingPose)

        autonomousCommand = SequentialCommandGroup(
            robot.followPathCMD(curve.Path1, false, 1.0),
            InstantCommand({ telemetry.addLine("Command Executed") })
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