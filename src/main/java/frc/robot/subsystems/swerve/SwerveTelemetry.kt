package frc.robot.subsystems.swerve

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class SwerveTelemetry {
    val field: Field2d = Field2d()

    val currentStatesPublisher: StructArrayPublisher<SwerveModuleState> =
        NetworkTableInstance.getDefault()
            .getTable("swerve")
            .getStructArrayTopic("currentStates", SwerveModuleState.struct)
            .publish()

    val desiredStatesPublisher: StructArrayPublisher<SwerveModuleState> =
        NetworkTableInstance.getDefault()
            .getTable("swerve")
            .getStructArrayTopic("desiredStates", SwerveModuleState.struct)
            .publish()

    fun telemetrize(state: SwerveDriveState) {
        field.robotPose = state.Pose

        SmartDashboard.putData("swerve/Field", field)
        currentStatesPublisher.set(state.ModuleStates)
        desiredStatesPublisher.set(state.ModuleTargets)
    }
}
