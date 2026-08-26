package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.autonomous.MirrorPaths;

//public  class Curve extends MirrorPaths {
//    public PathChain Path1;
//    public Pose startingPose;
//
//    public Curve(Follower follower, Alliance alliance) {
//        super(alliance);
//
//        startingPose = mirror(new Pose(56.000, 9.200, Math.toRadians(90)));
//
//        Path1 = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                mirror(new Pose(56.000, 9.200)),
//                                mirror(new Pose(69.010, 77.276)),
//                                mirror(new Pose(82.076, 9.755))
//                        )
//                ).setLinearHeadingInterpolation(mirrorHeading(Angle.fromDegrees(90)), mirrorHeading(Angle.fromDegrees(180)))
//                .build();
//    }
//}

public  class Curve extends MirrorPaths {
    public PathChain Path1;
    public Pose startingPose;

    public Curve(Follower follower, Alliance alliance) {
        super(alliance);

        startingPose = mirror(new Pose(56.000, 9.200, Math.toRadians(90)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                mirror(new Pose(56.000, 9.200)),
                                mirror(new Pose(69.010, 77.276)),
                                mirror(new Pose(82.076, 9.200))
                        )
                ).setLinearHeadingInterpolation(mirrorHeading(Angle.fromDegrees(90)), mirrorHeading(Angle.fromDegrees(180)))
                .build();
    }
}
  