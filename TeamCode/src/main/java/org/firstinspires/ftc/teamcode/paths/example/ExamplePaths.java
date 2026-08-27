package org.firstinspires.ftc.teamcode.paths.example;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.autonomous.MirrorPaths;

/**
 * Template class in which the Pedro Paths will be placed.
 * Each pose should be mirrored to simplify the Path processing and to satisfy each alliance's coordinates.
 */
public class ExamplePaths extends MirrorPaths {

    public Pose startingPose;
    public ExamplePaths(Follower follower, Alliance alliance) {
        super(alliance);

        // TODO Each time you duplicate this class the starting pose should be correctly set.
        startingPose = null;
    }
}
