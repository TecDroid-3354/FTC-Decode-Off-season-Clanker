package org.firstinspires.ftc.teamcode.utils

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.Robot
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.devices.OpMotorEx
import org.firstinspires.ftc.teamcode.utils.devices.OpServoEx

abstract class TecDroidRobot(private val telemetry: Telemetry, private val hardwareMap: HardwareMap): Robot() {

    val pTelemetry: TelemetryManager    = PanelsTelemetry.telemetry

    init {
        initBulkReadings()
        OpMotorEx.clearRegistry()
        OpServoEx.clearRegistry()
    }

    protected abstract fun subsystemInitialization()

    protected abstract fun printTelemetry()

    abstract fun initLoop()

    abstract fun initTeleOp()

    abstract fun initAuto(startingPose: Pose)

    private fun initBulkReadings() {
        super.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL)
    }

    override fun run() {
        CommandScheduler.getInstance().run()
        OpMotorEx.updateAll()
        OpServoEx.updateAll()
        printTelemetry()
        pTelemetry.update(telemetry)
    }

    abstract fun onEnd()

    abstract fun followPathCMD(path: PathChain, holdEnd: Boolean, maxPower: Double): Command

    abstract fun getFollower(): Follower
}