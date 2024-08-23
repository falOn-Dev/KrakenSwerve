package frc.robot.subsystems.swerve

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2
import frc.robot.subsystems.swerve.gyro.GyroIOSim
import frc.robot.subsystems.swerve.module.SwerveModule
import org.littletonrobotics.junction.Logger
import org.photonvision.EstimatedRobotPose
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * Drivetrain subsystem using Falon's custom swerve code
 *
 * @constructor Instantiates a swerve subsystem using CTRE's constants class
 * @param drivetrainConstants Drivetrain parameters like Pigeon ID and CAN bus name
 * @param moduleConstants Array of module constants containing things like motor IDs, inversions, and offsets
 *
 * @author Falon C.
 */
class Drivetrain(
    private val drivetrainConstants: SwerveDrivetrainConstants,
    vararg val moduleConstants: SwerveModuleConstants,
) : SubsystemBase() {

    /**
     * Gyro IO for interacting with a gyroscope, automatically initializes between Real, Sim, and Replay (blank interface)
     */
    private val gyro: GyroIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> GyroIOPigeon2(drivetrainConstants)
        Constants.RobotConstants.Mode.SIM -> GyroIOSim(this::robotRelativeSpeeds)
        Constants.RobotConstants.Mode.REPLAY -> object : GyroIO {}
    }

    /**
     * Gyro inputs for reading values out of the gyro IO interface
     */
    val gyroInputs: GyroIO.GyroInputs = GyroIO.GyroInputs()

    /**
     * Array of [SwerveModule], used internally for interacting with modules
     * Stored in the order FL, FR, BL, BR
     */
    private val modules: Array<SwerveModule> = moduleConstants.map { SwerveModule(it) }.toTypedArray()

    /**
     * Array of [SwerveModuleState] used for storing desired states, these are then logged for tuning purposes
     */
    private val desiredStates: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }

    /**
     * Array of [SwerveModuleState] used for storing measured states from drivebase's modules, these are then logged for tuning purposes
     */
    private val measuredStates: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }

    /**
     * Array of [Translation2d] used for storing the physical positions of modules, used for kinematics
     */
    private val moduleTranslations: Array<Translation2d> = Array(4) { Translation2d() }

    /**
     * Array of [SwerveModulePosition] used for storing the distance traveled of each module, this is for odometry
     */
    private val modulePositions: Array<SwerveModulePosition> = Array(4) { SwerveModulePosition() }

    /**
     * Robot relative speeds of the robot, used for logging and gyro simulation
     */
    val robotRelativeSpeeds: ChassisSpeeds
        get() = ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(*measuredStates), gyroInputs.yaw.unaryMinus())

    /**
     * Field relative speeds of the robot, used for logging and gyro simulation
     */
    val fieldRelativeSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(*measuredStates)

    /**
     * Kinematics object used for calculating module states from chassis speeds and vice versa
     */
    private val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(*getModuleTranslations())

    /**
     * Pose estimator used for calculating the robot's position on the field
     */
    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        gyroInputs.yaw,
        getModulePositions(),
        Pose2d(),
    )

    /**
     * Current pose of the robot on the field
     */
    val pose: Pose2d
        get() = poseEstimator.estimatedPosition

    /**
     * Method for getting the module translations from the module constants
     *
     * @return Array of [Translation2d] containing the module translations
     */
    private fun getModuleTranslations(): Array<Translation2d> {
        modules.forEachIndexed { index, module ->
            moduleTranslations[index] = Translation2d(module.config.LocationX, module.config.LocationY)
        }

        return moduleTranslations
    }

    /**
     * Method for getting the module positions from the module constants
     *
     * @return Array of [SwerveModulePosition] containing the module positions
     */
    private fun getModulePositions(): Array<SwerveModulePosition> {
        modules.forEachIndexed { index, module ->
            modulePositions[index] = module.modulePosition
        }

        return modulePositions
    }

    /**
     * Method for resetting the heading of the robot
     */
    fun resetHeading() {
        gyro.setYaw(0.0)
    }

    /**
     * Method for setting the heading of the robot
     *
     * @param heading New heading of the robot
     */
    fun applyChassisSpeeds(speeds: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)

        val swerveModuleStates = kinematics.toSwerveModuleStates(discreteSpeeds)

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, TunerConstants.kSpeedAt12VoltsMps)

        modules.forEachIndexed { index, module ->
            desiredStates[index] = swerveModuleStates[index]
            module.apply(swerveModuleStates[index])
        }
    }

    /**
     * Method for adding a vision measurement to the pose estimator
     * @see [EstimatedRobotPose]
     *
     * @param pose Vision measurement to add to the pose estimator
     */
    fun addVisionMeasurement(pose: EstimatedRobotPose) {
        poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds)
    }

    /**
     * Factory for creating a command to drive the robot
     * This command will create chassis speeds from the inputs and apply them to the robot
     *
     * @param forwards Supplier for the forwards speed of the robot
     * @param strafe Supplier for the strafe speed of the robot
     * @param rotation Supplier for the rotation speed of the robot
     * @param isFieldOriented Supplier for whether the robot is field oriented or not
     *
     * @return Command for driving the robot
     */
    fun driveCommand(
        forwards: DoubleSupplier,
        strafe: DoubleSupplier,
        rotation: DoubleSupplier,
        isFieldOriented: BooleanSupplier,
    ): Command? {
        return this.run {
            val speeds = if (isFieldOriented.asBoolean) {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwards.asDouble * 1.5,
                    strafe.asDouble * 1.5,
                    rotation.asDouble * (Math.PI),
                    gyroInputs.yaw,
                )
            } else {
                ChassisSpeeds(
                    forwards.asDouble * 1.5,
                    strafe.asDouble * 1.5,
                    rotation.asDouble * (Math.PI),
                )
            }

            applyChassisSpeeds(speeds)
        }
    }

    /**
     * Method for resetting the odometry of the robot
     *
     * @param pose New pose of the robot
     */
    fun resetOdometry(pose: Pose2d) {
        poseEstimator.resetPosition(gyroInputs.yaw, getModulePositions(), pose)
    }

    /**
     * Periodic method, runs every loop
     *
     * This method updates the gyro inputs, module inputs, and pose estimator
     */
    override fun periodic() {
        gyro.updateInputs(gyroInputs)
        Logger.processInputs("swerve/gyro", gyroInputs)
        modules.forEachIndexed { index, it ->
            it.updateInputs()
            Logger.processInputs("swerve/module[${index + 1}]", it.inputs)
        }
        poseEstimator.update(gyroInputs.yaw, getModulePositions())

        Logger.recordOutput("swerve/pose", poseEstimator.estimatedPosition)
        modules.forEachIndexed { index, swerveModule ->
            measuredStates[index] = swerveModule.state
        }
        Logger.recordOutput("swerve/measuredState", *measuredStates)
        Logger.recordOutput("swerve/desiredState", *desiredStates)
    }
}
