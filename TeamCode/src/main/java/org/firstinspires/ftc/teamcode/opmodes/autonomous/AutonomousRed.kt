package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.FarAutonomous
import org.firstinspires.ftc.teamcode.utils.Alliance

@Autonomous(group = "Auto", name = "Auto - Red")
class AutonomousRed: FarAutonomous(Alliance.RED)