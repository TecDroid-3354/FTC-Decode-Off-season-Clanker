package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.autonomous.MirrorPaths;

public class Paths extends MirrorPaths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain line3;

    public Pose startingPose;

    public Paths(Follower follower, Alliance alliance) {

        super(alliance);

        startingPose = mirror(new Pose(63.243, 8.869, Math.toRadians(90)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(63.243, 8.869),

                                new Pose(62.954, 96.845)
                        )
                ).setConstantHeadingInterpolation(mirrorHeading(Angle.fromDegrees(90)))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                mirror(new Pose(62.954, 96.845)),

                                mirror(new Pose(58.199, 102.487))
                        )
                ).setLinearHeadingInterpolation(mirrorHeading(Angle.fromDegrees(90)), mirrorHeading(Angle.fromDegrees(45)))

                .build();

        line3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                mirror(new Pose(58.199, 102.487)),

                                mirror(new Pose(58.237, 133.280))
                        )
                ).setLinearHeadingInterpolation(mirrorHeading(Angle.fromDegrees(45)), mirrorHeading(Angle.fromDegrees(90)))

                .build();
    }
}
  