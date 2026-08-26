package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.autonomous.MirrorPaths;

public class Line extends MirrorPaths {
    public PathChain Path1;

    public Pose startingPose;

    public Line(Follower follower, Alliance alliance) {
        super(alliance);

        startingPose = mirror(new Pose(56.000, 36.000, Math.toRadians(90)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                mirror(new Pose(56.000, 36.000)),
                                mirror(new Pose(84.000, 36.000))
                        )
                ).setLinearHeadingInterpolation(mirrorHeading(Angle.fromDegrees(90)), mirrorHeading(Angle.fromDegrees(180)))
                .build();
    }
}

  