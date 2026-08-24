package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Distance
import java.util.function.Supplier
import kotlin.math.tan

class Limelight(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    val rotationSupplier: Supplier<Angle>
) : SubsystemBase() {

    private var limelight: Limelight3A? = null
    var llResult: LLResult? = null

    private var obeliskId = 0

    private var ty = 0.0
    private var tx = 0.0
    private var ta = 0.0

    private var distanceFromLimelightToGoalInches = Distance.fromInches(0.0)

    init {
        limelight = hardwareMap.get(
            Limelight3A::class.java,
            VisionConstants.LimelightIdentification.Id
        )

        // Retrieves pipeline
        limelight!!.pipelineSwitch(VisionConstants.LimelightConfiguration.PipelineIndex) // Gets the limelight pipeline
        // How many times per second the limelight receives data in seconds
        limelight!!.setPollRateHz(VisionConstants.LimelightConfiguration.PollRateHz)
        // Starts the limelight's readings
        limelight!!.start()
    }

    fun getTx(): Double = tx
    fun getTy(): Double = ty
    fun getTa(): Double = ta

    /**
     * Gets the distance from the limelight lenses to a filtered id and returns the distance in any unit desired
     *  @param filterArray the desired ids for the limelight to follow
     *  @return the distance from the filtered id to the limelight lenses
     */
    fun getDistanceToGoal(filterArray: IntArray): Distance {
        if (llResult!!.isValid && llResult != null) {
            val fiducialResult = llResult!!.fiducialResults

            for (detectedId in fiducialResult) {
                for (id in filterArray) {
                    if (detectedId.fiducialId == id) {
                        return distanceFromLimelightToGoalInches
                    }
                }
            }
        }

        return Distance.fromInches(0.0)
    }


    fun llResultIsValid(): Boolean {
        return llResult != null && llResult!!.isValid
    }
    fun getFilteredTx(alliance: Alliance): Angle {
        val filteredId = when (alliance) {
            Alliance.BLUE -> 20
            Alliance.RED -> 24
        }

        if (llResultIsValid()) {
            val fiducialResult = llResult!!.fiducialResults

            for (id in fiducialResult) {
                if (id.fiducialId == filteredId) {
                    return Angle.fromDegrees(getTx())
                }
            }
        }

        return Angle.fromDegrees(0.0)
    }

    /**
     * Gets the obelisk april tag id and relates it to a [MotifPatterns] depending on the actual pattern of the match
     * @return the current motif pattern
     */
    fun getMotifPattern(): MotifPatterns {
        return when (obeliskId) {
            21 -> MotifPatterns.GREEN_PURPLE_PURPLE
            22 -> MotifPatterns.PURPLE_GREEN_PURPLE
            23 -> MotifPatterns.PURPLE_PURPLE_GREEN
            else -> MotifPatterns.NO_PATTERN_DETECTED
        }
    }

    override fun periodic() {

        // Updating limelights' robot orientation with the Yaw
        limelight!!.updateRobotOrientation(rotationSupplier.get().degrees)

        // LLResult is like a container full of information about what Limelight sees
        llResult = limelight!!.getLatestResult()

        val fiducialResult = llResult!!.fiducialResults

        // Math to calculate distance was taken from documentation:
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance#using-area-to-estimate-distance
        // The condition verifies whether the LimeLight Result is a valid statement
        if (llResult != null && llResult!!.isValid()) {

            // Obelisk ID detection
            outerLoop@ for (detectedId in fiducialResult) {
                for (aprilTagId in VisionConstants.AprilTagsIdentification.ObeliskIds) {
                    if (detectedId.fiducialId == aprilTagId && getMotifPattern() == MotifPatterns.NO_PATTERN_DETECTED) {
                        obeliskId = detectedId.fiducialId
                        break@outerLoop
                    }
                }
            }


            val botPose = llResult!!.botpose_MT2
            // Offset to target in degrees (from crosshair)
            val targetOffsetAngle_Vertical = Angle.fromDegrees(llResult!!.getTy())
            // Needs to be in radians for tan() method
            val angleToGoalRadians =
                Math.toRadians(VisionConstants.LimelightPhysicalDescription.LLMountAngleFromHorizontal.degrees + targetOffsetAngle_Vertical.degrees)

            // Calculated distance from limelight lens to goal (in inches)
            distanceFromLimelightToGoalInches =
                (VisionConstants.AprilTagsPhysicalDescription.GoalHeightFromGround - VisionConstants.LimelightPhysicalDescription.LLHeightFromGroundToLens) / tan(
                    angleToGoalRadians
                )
            telemetry.addData(
                "Limelight TargetDistanceInches",
                distanceFromLimelightToGoalInches.inches
            )

            // We will first get a (MetaTag2) Pose3D. From here, we will extract its Tx, Ty & Ta components
            tx = llResult!!.tx
            ty = llResult!!.ty
            ta = llResult!!.ta

            /*
             * It is important to notice that the Full3D option should be enabled
             * */
        } else {
            tx = 0.0
            ty = 0.0
            ta = 0.0
        }
    }
}