package org.firstinspires.ftc.teamcode

import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.utils.Alliance
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup

class Autonomous(private val alliance: Alliance): CommandOpMode() {

    lateinit var controller: GamepadEx
    lateinit var robot: Robot

    override fun initialize() {
        super.reset()
        controller = GamepadEx(gamepad1)
        robot = Robot(alliance, hardwareMap, controller, telemetry)
        robot.initAuto()

        val autonomousCommand: Command = SequentialCommandGroup(
            /* Write your commands here, don't forget to use parallels, race or sequential if needed */
            //FollowPathCommand(),
            SequentialCommandGroup(),
            ParallelCommandGroup(),
            InstantCommand(),
        )

        // Schedule the autonomous command.
        // Don't forget to enable the 30-second timer if running an actual autonomous
        autonomousCommand.schedule()
    }

    override fun initialize_loop() {
        robot.initLoop()
    }

    override fun run() {
        robot.run()
    }

    override fun end() {
        robot.onEnd()
    }
}