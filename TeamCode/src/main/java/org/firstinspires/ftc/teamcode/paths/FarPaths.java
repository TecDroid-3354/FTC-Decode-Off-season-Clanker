package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.autonomous.MirrorPaths;

public class FarPaths extends MirrorPaths {
    public PathChain Path1;
    public PathChain Path3;
    public PathChain line3;
    public PathChain line4;
    public Pose startingPose;

    public FarPaths(Follower follower, Alliance alliance) {
        super(alliance);

        startingPose = mirror(new Pose(56.869, 9.159, Math.toRadians(180)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                mirror(new Pose(56.869, 9.159)),
                                mirror(new Pose(64.388, 40.151)),
                                mirror(new Pose(12.000, 36.000))
                        )
                ).setConstantHeadingInterpolation(mirrorHeading(Angle.fromDegrees(180)))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                mirror(new Pose(11.670, 36.000)),
                                mirror(new Pose(18.112, 15.009)),
                                mirror(new Pose(55.865, 16.509))
                        )
                ).setConstantHeadingInterpolation(mirrorHeading(Angle.fromDegrees(180)))

                .build();

        line3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                mirror(new Pose(55.865, 16.509)),
                                mirror(new Pose(62.094, 65.711)),
                                mirror(new Pose(12.000, 59.976))
                        )
                ).setConstantHeadingInterpolation(mirrorHeading(Angle.fromDegrees(180)))

                .build();

        line4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                mirror(new Pose(10.720, 59.976)),

                                mirror(new Pose(55.920, 15.646))
                        )
                ).setConstantHeadingInterpolation(mirrorHeading(Angle.fromDegrees(180)))

                .build();
    }
}
  