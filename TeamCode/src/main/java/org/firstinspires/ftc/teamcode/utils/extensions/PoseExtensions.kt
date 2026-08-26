package org.firstinspires.ftc.teamcode.utils.extensions

import com.pedropathing.ftc.FTCCoordinates
import com.pedropathing.ftc.InvertedFTCCoordinates
import com.pedropathing.ftc.PoseConverter
import com.pedropathing.geometry.PedroCoordinates
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

fun Pose.toPose2D(): Pose2D {
    return PoseConverter.poseToPose2D(this, FTCCoordinates.INSTANCE)
}

fun Pose.toPose2DInverted(): Pose2D {
    return PoseConverter.poseToPose2D(this, InvertedFTCCoordinates.INSTANCE)
}

fun Pose2D.toPose(): Pose {
    return PoseConverter.pose2DToPose(this, PedroCoordinates.INSTANCE)
}