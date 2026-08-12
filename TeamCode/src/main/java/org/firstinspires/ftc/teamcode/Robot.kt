package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.hardware.motors.CRServo
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.TecDroidRobot
import org.firstinspires.ftc.teamcode.utils.extensions.a
import org.firstinspires.ftc.teamcode.utils.extensions.onFalse
import org.firstinspires.ftc.teamcode.utils.extensions.onTrue

class Robot(private val alliance: Alliance, private val hardwareMap: HardwareMap, private val controller: GamepadEx, telemetry: Telemetry): TecDroidRobot(telemetry, hardwareMap) {

    /* Declare your subsystems here */

    init {
        subsystemInitialization()
    }

    /* Initialize your subsystems here */
    override fun subsystemInitialization() {

    }

    /* Initialize your teleop controller commands here */
    override fun initTeleOp() {
        controller.a()
            .onTrue(InstantCommand({ intake.enable() }))
            .onFalse(InstantCommand({ intake.disable() }))
    }

    /* Initialize your auto commands here, set chassis alliance and starting pose */
    override fun initAuto() {}

    /* When the teleop ends, declare what to do */
    override fun onEnd() {}

    // Print telemetry using the pTelemetry object on RobotConstants.Telemetry. It will be printed on both Panels and Driver Hub.
    override fun printTelemetry() {}
}