package org.firstinspires.ftc.teamcode.utils.autonomous

import com.pedropathing.control.FilteredPIDFCoefficients
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.control.PredictiveBrakingCoefficients
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.ftc.localization.constants.OTOSConstants
import com.pedropathing.paths.PathConstraints
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.LinearVelocity
import org.firstinspires.ftc.teamcode.utils.Mass
import java.util.Optional

object PedroPathing {

    data class PIDFSwitchThresholds(
        var drivePIDFSwitch             : Optional<Double>,
        var translationalPIDFSwitch     : Optional<Double>,
        var headingPIDFSwitch           : Optional<Angle>
    )

    private object MotorDefaultNames {

        // Front right motor default name, declared in the Control HUB configuration
        const val FRONT_RIGHT_MOTOR_NAME: String            = "frontRight"

        // Back right motor default name, declared in the Control HUB configuration
        const val BACK_RIGHT_MOTOR_NAME : String            = "backRight"

        // Front left motor default name, declared in the Control HUB configuration
        const val FRONT_LEFT_MOTOR_NAME : String            = "frontLeft"

        // Back left motor default name, declared in the Control HUB configuration
        const val BACK_LEFT_MOTOR_NAME  : String            = "backLeft"
    }

    // Follower Constants
    private val followerDefaultConstants: FollowerConstants = FollowerConstants()
    // Mecanum Constants
    private val mecanumDefaultConstants : MecanumConstants = MecanumConstants()
    // OTOS Localizer Constants
    private val otosDefaultConstants    : OTOSConstants = OTOSConstants()
    // Values directly obtained per documentation. Path constraints
    private val pathDefaultConstraints  : PathConstraints = PathConstraints(0.99, 100.0, 1.0, 1.0)

    /* INITIALIZATION CODE */
    init {
        // Initializing defaults

        with(followerDefaultConstants) {
            mass                        (Mass.fromKilograms(10.0).kilograms)
            isUseSecondaryHeadingPIDF                       = false
            isUseSecondaryDrivePIDF                         = false
            isUseSecondaryTranslationalPIDF                 = false
        }

        with(mecanumDefaultConstants) {
            // ------------ Our motor's name --------------- //
            rightFrontMotorName         (MotorDefaultNames.FRONT_RIGHT_MOTOR_NAME)
            rightRearMotorName          (MotorDefaultNames.BACK_RIGHT_MOTOR_NAME)
            leftFrontMotorName          (MotorDefaultNames.FRONT_LEFT_MOTOR_NAME)
            leftRearMotorName           (MotorDefaultNames.BACK_LEFT_MOTOR_NAME)

            // ---------- Our motor's direction ---------- //

            // By default, wheels on the other side will travel in the opposite direction
            leftFrontMotorDirection     (DcMotorSimple.Direction.FORWARD)
            leftRearMotorDirection      (DcMotorSimple.Direction.FORWARD)
            // By default, wheels on one side will travel in a negative or positive direction depending on your mounting
            rightFrontMotorDirection    (DcMotorSimple.Direction.REVERSE)
            rightRearMotorDirection     (DcMotorSimple.Direction.REVERSE)

            // ---------- Our motor's max power ---------- //
            maxPower(1.0)
        }

        with(otosDefaultConstants) {
            hardwareMapName             ("otos")
            // The default angular return units by standard
            angleUnit                   (AngleUnit.RADIANS)
            // The default distance return units by standard
            linearUnit                  (DistanceUnit.INCH)
        }
    }

    /**
     * Creates a new [FollowerConstants] with the requested optional parameters. If not present, the default values
     * will be grabbed by the new configuration.
     * Default [Mass] is 10 kilograms. This value MUST be changed according to your robot's mass.
     * @param mass your robot's mass. It is used to compensate for centripetal force.
     * @param headingPIDFCoefficients the robot's primary heading PIDF Coefficients.
     * You can adjust the PIDF value LIVE when accessing Panels.
     * @param secondaryHeadingPIDFCoefficients the robot's secondary heading PIDF Coefficients.
     * You can adjust the PIDF value live when accessing to Panel.
     * @param forwardZeroPowerAcceleration The deceleration rate of the robot over the x-axis (forward) in inches per second squared. THIS VALUE IS REQUIRED TO BE NEGATIVE.
     * @param lateralZeroPowerAcceleration The deceleration rate of the robot over the y-axis (lateral) in inches per second squared. THIS VALUE IS REQUIRED TO BE NEGATIVE.
     * @param translationalPIDFCoefficients the robot's primary translational PIDF Coefficients.
     * You can adjust the PIDF value LIVE when accessing Panels
     * @param secondaryTranslationalPIDFCoefficients the robot´'s secondary translational PIDF Coefficients.
     * You can adjust the PIDF value live when accessing to Panels.
     * @param drivePIDFCoefficients the robot's primary drive PIDF Coefficients.
     * You can adjust the PIDF value LIVE when accessing Panels
     * @param secondaryDrivePIDFCoefficients the robot's secondary drive PIDF Coefficients.
     * You can adjust the PIDF value live when accessing to Panels.
     * @param centripetalScaling the centripetal force correction enables the robot to accurately follow curved paths.
     * For this value to be tuned, your robot's [Mass] is needed by the [FollowerConstants].
     * You can use Panels to visualize the robot in Panels Field View
     * @param movementPIDFSwitchValues these values represent the limit when the secondary PIDF controller in each type of movement
     * (turning, translational and drive) starts operating. It may be used when the secondary PIDFs are enabled.
     * @return a new [FollowerConstants] containing the specified parameters.
     * @see followerDefaultConstants
     */
    fun createFollowerConstantsWithPIDF(mass: Optional<Mass>,
                                        headingPIDFCoefficients: Optional<PIDFCoefficients>, secondaryHeadingPIDFCoefficients: Optional<PIDFCoefficients>,
                                        forwardZeroPowerAcceleration: Optional<Double>, lateralZeroPowerAcceleration: Optional<Double>,
                                        translationalPIDFCoefficients: Optional<PIDFCoefficients>, secondaryTranslationalPIDFCoefficients: Optional<PIDFCoefficients>,
                                        drivePIDFCoefficients: Optional<PIDFCoefficients>, secondaryDrivePIDFCoefficients: Optional<PIDFCoefficients>,
                                        centripetalScaling: Optional<Double>, movementPIDFSwitchValues: Optional<PIDFSwitchThresholds>
    ): FollowerConstants {
        // Creating the new FollowerConstants(). It will hold the new configurations
        val newFollowerConstants        : FollowerConstants = followerDefaultConstants

        // If present, mass will be updated to a new value. Adjust this according to your robot's weight
        if (mass.isPresent) {
            newFollowerConstants.mass(mass.get().kilograms.coerceIn(Double.MIN_VALUE, Double.MAX_VALUE))
        }

        // If present, heading PIDF coefficients will be applied.
        if (headingPIDFCoefficients.isPresent) {
            newFollowerConstants.headingPIDFCoefficients(headingPIDFCoefficients.get())
        } else { newFollowerConstants.headingPIDFCoefficients(PIDFCoefficients(0.01, 0.0, 0.0, 0.0)) }

        // If present, secondary heading PIDF Coefficients will be applied along with the enabled flag.
        if (secondaryHeadingPIDFCoefficients.isPresent) {
            newFollowerConstants.isUseSecondaryHeadingPIDF  = true
            newFollowerConstants.secondaryHeadingPIDFCoefficients(secondaryHeadingPIDFCoefficients.get())
        } else { newFollowerConstants.isUseSecondaryHeadingPIDF = false }

        // If present, forward zero power acceleration will be set
        if (forwardZeroPowerAcceleration.isPresent) {
            newFollowerConstants.forwardZeroPowerAcceleration(forwardZeroPowerAcceleration.get()
                .coerceIn(Double.NEGATIVE_INFINITY, Double.MIN_VALUE.unaryMinus()))
        }

        // If present, lateral zero power acceleration will be set
        if (lateralZeroPowerAcceleration.isPresent) {
            newFollowerConstants.lateralZeroPowerAcceleration(lateralZeroPowerAcceleration.get()
                .coerceIn(Double.NEGATIVE_INFINITY, Double.MIN_VALUE.unaryMinus()))
        }

        // If present, translational PIDF Coefficients will be applied to the robot's lateral movement.
        if (translationalPIDFCoefficients.isPresent) {
            newFollowerConstants.translationalPIDFCoefficients(translationalPIDFCoefficients.get())
        } else { newFollowerConstants.translationalPIDFCoefficients(
            PIDFCoefficients(
                0.01,
                0.0,
                0.0,
                0.0
            )
        ) }

        // If present, secondary translational PIDF Coefficients will be applied along with the enabled flag.
        if (secondaryTranslationalPIDFCoefficients.isPresent) {
            newFollowerConstants.isUseSecondaryTranslationalPIDF = true
            newFollowerConstants.secondaryTranslationalPIDFCoefficients(secondaryTranslationalPIDFCoefficients.get())
        } else { newFollowerConstants.isUseSecondaryTranslationalPIDF = false }

        // If present, drive PIDF Coefficients will be used to create a FilteredPIDFCoefficients object with a default t value of 0.6.
        if (drivePIDFCoefficients.isPresent) {
            newFollowerConstants.drivePIDFCoefficients(
                FilteredPIDFCoefficients(
                    drivePIDFCoefficients.get().P,
                    drivePIDFCoefficients.get().I,
                    drivePIDFCoefficients.get().D,
                    0.55,
                    drivePIDFCoefficients.get().F
                )
            )
        }

        // If present, the secondary drive PIDFCoefficients() object will be used to create a FilteredPIDFCoefficients() with a default t value.
        if (secondaryDrivePIDFCoefficients.isPresent) {
            newFollowerConstants.isUseSecondaryDrivePIDF    = true
            newFollowerConstants.secondaryDrivePIDFCoefficients(
                FilteredPIDFCoefficients(
                    secondaryDrivePIDFCoefficients.get().P,
                    secondaryDrivePIDFCoefficients.get().I,
                    secondaryDrivePIDFCoefficients.get().D,
                    0.55,
                    secondaryDrivePIDFCoefficients.get().F
                )
            )
        } else { newFollowerConstants.isUseSecondaryDrivePIDF = false }

        // If present, the centripetal scaling value will be set to the follower's constants. The default value is 0.0005.
        if (centripetalScaling.isPresent) {
            newFollowerConstants.centripetalScaling(centripetalScaling.get())
        }

        // If present, the limit for the secondary PIDF Controllers will be overridden with the new ones passed.
        if (movementPIDFSwitchValues.isPresent) {
            val pidfSwitchValues = movementPIDFSwitchValues.get()

            if (pidfSwitchValues.headingPIDFSwitch.isPresent) {
                newFollowerConstants.headingPIDFSwitch = pidfSwitchValues.headingPIDFSwitch.get().radians
            }
            if (pidfSwitchValues.translationalPIDFSwitch.isPresent) {
                newFollowerConstants.translationalPIDFSwitch = pidfSwitchValues.translationalPIDFSwitch.get()
            }
            if (pidfSwitchValues.drivePIDFSwitch.isPresent) {
                newFollowerConstants.drivePIDFSwitch = pidfSwitchValues.drivePIDFSwitch.get()
            }
        }

        // Return the new FollowerConstants() object.
        return newFollowerConstants
    }

    /**
     * Creates a new [FollowerConstants] tailored to configure the [com.pedropathing.follower.Follower]
     * to use predictive braking instead of the traditional translational and drive PIDFs.
     * @param mass your robot's mass. It is used to compensate for centripetal force.
     * @param headingPIDFCoefficients the robot's primary heading PIDF Coefficients.
     * You can adjust the PIDF value LIVE when accessing Panels.
     * @param secondaryHeadingPIDFCoefficients the robot's secondary heading PIDF Coefficients.
     * You can adjust the PIDF value live when accessing to Panel.
     * @param secondaryHeadingPIDFSwitchThreshold the value in which the secondary heading PIDF will start action.
     * Default value is [Math.PI] / 20.0
     * @param predictiveBrakingCoefficients these are the coefficients the [com.pedropathing.control.PredictiveBrakingController] will use
     * in order to determine how the robot should move. Under automatic, run the predictive braking tuner.
     * You can change values live when accessing Panels.
     * @return a new [FollowerConstants] containing the necessary settings to use Predictive Braking as the main controller
     * during Autonomous period.
     */
    fun createFollowerConstantsWithPredictiveBraking(mass: Optional<Mass>,
                                                     headingPIDFCoefficients: Optional<PIDFCoefficients>, secondaryHeadingPIDFCoefficients: Optional<PIDFCoefficients>,
                                                     secondaryHeadingPIDFSwitchThreshold: Optional<Angle>, predictiveBrakingCoefficients: Optional<PredictiveBrakingCoefficients>
    ): FollowerConstants {
        val newFollowerConstants        : FollowerConstants = FollowerConstants()

        // Turn off centripetal scaling as predictive braking takes account curves.
        newFollowerConstants.centripetalScaling(0.0)
        newFollowerConstants.usePredictiveBraking = true

        // If present, mass will be updated to a new value. Adjust this according to your robot's weight
        if (mass.isPresent) {
            newFollowerConstants.mass(mass.get().kilograms.coerceIn(Double.MIN_VALUE, Double.MAX_VALUE))
        }

        // If present, heading PIDF coefficients will be applied.
        if (headingPIDFCoefficients.isPresent) {
            newFollowerConstants.headingPIDFCoefficients(headingPIDFCoefficients.get())
        } else { newFollowerConstants.headingPIDFCoefficients(PIDFCoefficients(0.01, 0.0, 0.0, 0.0)) }

        // If present, secondary heading PIDF Coefficients will be applied along with the enabled flag.
        if (secondaryHeadingPIDFCoefficients.isPresent) {
            newFollowerConstants.isUseSecondaryHeadingPIDF  = true
            newFollowerConstants.secondaryHeadingPIDFCoefficients(secondaryHeadingPIDFCoefficients.get())

            if (secondaryHeadingPIDFSwitchThreshold.isPresent) {
                newFollowerConstants.headingPIDFSwitch = secondaryHeadingPIDFSwitchThreshold.get().radians
            }
        } else { newFollowerConstants.isUseSecondaryHeadingPIDF = false }

        if (predictiveBrakingCoefficients.isPresent) {
            newFollowerConstants.predictiveBrakingCoefficients(predictiveBrakingCoefficients.get())
        } else { newFollowerConstants.predictiveBrakingCoefficients(PredictiveBrakingCoefficients(0.1, 0.04, 0.0016)) }

        return newFollowerConstants
    }

    /**
     * Creates a new [MecanumConstants] with the requested optional parameters. If not present, the parameter will not be
     * set and the default values used by the [MecanumConstants] will be then used.
     * @param rightSideDirection the desired motor's [DcMotorSimple.Direction] on the right side of the mecanum.
     * @param leftSideDirection the desired motor's [DcMotorSimple.Direction] on the left side of the mecanum.
     * @param maxPower the maximum applied power to the mecanum motors
     * @param xVelocity the average maximum velocity over the x-axis of the robot (Forward and reverse) in inches per second.
     * @param yVelocity the average maximum velocity over the y-axis of the robot (Left and right) in inches per second.
     * @return a new [MecanumConstants] containing the specified parameters. If not it will simply return the default values.
     * @see [mecanumDefaultConstants]
     */
    fun createMecanumConstants(maxPower: Optional<Double>, rightSideDirection: Optional<DcMotorSimple.Direction>, leftSideDirection: Optional<DcMotorSimple.Direction>,
                               xVelocity: Optional<LinearVelocity>, yVelocity: Optional<LinearVelocity>
    ): MecanumConstants {
        // Creating a new MecanumConstants(). It will hold the new configurations
        val newMecanumConstants         : MecanumConstants = mecanumDefaultConstants

        // If present, the right side of the mecanum will spin at the specified direction
        if (rightSideDirection.isPresent) {
            newMecanumConstants.rightFrontMotorDirection(rightSideDirection.get())
            newMecanumConstants.rightRearMotorDirection(rightSideDirection.get())
        }

        // If present, the left side of the mecanum will spin at the specified direction
        if (leftSideDirection.isPresent) {
            newMecanumConstants.leftFrontMotorDirection(leftSideDirection.get())
            newMecanumConstants.leftRearMotorDirection(leftSideDirection.get())
        }

        // If present, a new maximum achievable power will be set to the mecanum
        if (maxPower.isPresent) {
            newMecanumConstants.maxPower(maxPower.get())
        }

        // If present, a new xVelocity will be passed to the configuration.
        // This value must be obtained by running the Forward Velocity Test
        if (xVelocity.isPresent) {
            newMecanumConstants.xVelocity(xVelocity.get().inps)
        }

        // If present, a new yVelocity will be passed to the configuration.
        // This value must be obtained by running the Lateral Velocity Test
        if (yVelocity.isPresent) {
            newMecanumConstants.yVelocity(yVelocity.get().inps)
        }

        // Return the new MecanumConstants()
        return newMecanumConstants
    }

    /**
     * Creates a new [OTOSConstants] based on the requested [offset], [linearScalar] and [angularScalar].
     * Default values are included so new ones are not necessary to be given.
     * @param offset a [com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D] representing the sensor offset from the robot's center.
     * X and Y values must be given in inches, while the theta value in radians.
     * @param linearScalar a scale factor for the sensor x and y given values.
     * @param angularScalar a scale factor for the sensor's integrated IMU readings.
     * @return a new [OTOSConstants] holding the specified parameters. If not, it will simply return the default values
     * @see otosDefaultConstants
     */
    fun createOTOSLocalizerConstants(offset: Optional<SparkFunOTOS.Pose2D>, linearScalar: Optional<Double>,
                                     angularScalar: Optional<Double>
    ): OTOSConstants {
        // Creating a new OtosConstants() object. It will hold the new configurations
        val otosNewConstants            : OTOSConstants = otosDefaultConstants

        // If present, setting the new offset value.
        // If not present, an empty Pose2D() will be passed.
        if (offset.isPresent) {
            otosNewConstants.offset(offset.get())
        } else { otosNewConstants.offset(SparkFunOTOS.Pose2D()) }

        // If present, it will clamp the linear scalar value between its boundaries to avoid misunderstandings
        // when setting a bigger value.
        // Bigger values will be ignored by the OTOS Sensor
        if (linearScalar.isPresent) {
            otosNewConstants.linearScalar(linearScalar.get()
                .coerceIn(SparkFunOTOS.MIN_SCALAR + Double.MIN_VALUE, SparkFunOTOS.MAX_SCALAR - Double.MIN_VALUE))
        }

        // If present, it will clamp the angular scalar value between its boundaries to avoid misunderstandings
        // when setting a bigger value.
        // Bigger values will be ignored by the OTOS Sensor
        if (angularScalar.isPresent) {
            otosNewConstants.angularScalar(angularScalar.get()
                .coerceIn(SparkFunOTOS.MIN_SCALAR + Double.MIN_VALUE, SparkFunOTOS.MAX_SCALAR - Double.MIN_VALUE))
        }

        // Return the new OTOSConstants()
        return otosNewConstants
    }

    /**
     * Creates a new [PathConstraints] with the specified braking parameters. If not specified, default values will be
     * given as parameters.
     * @param brakingStrength this controls how strong the braking is. A higher value means stronger braking, a lower value means gentler braking.
     * @param brakingStart this controls how early along the path the robot will start braking.
     * The higher the value the earlier the braking starts. The lower the value the later the braking starts.
     * @return a new [PathConstraints] containing the specified configurations
     * @see pathDefaultConstraints
     */
    fun createPathConstraints(brakingStrength: Optional<Double>, brakingStart: Optional<Double>): PathConstraints {
        // Creating the new PathConstraints(). It will hold the new configurations
        val newPathConstraints          : PathConstraints = pathDefaultConstraints

        // If present, braking strength will be set. If not, it will be set to 1.0
        if (brakingStrength.isPresent) {
            newPathConstraints.brakingStrength = brakingStrength.get()
        } else { newPathConstraints.brakingStrength = 1.0 }

        // If present, braking start will be set. If not, it will be set to 1.0
        if (brakingStart.isPresent) {
            newPathConstraints.brakingStart = brakingStart.get()
        } else { newPathConstraints.brakingStart = 1.0 }

        // Return the new PathConstraints()
        return newPathConstraints
    }
}