package frc.robot

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RobotBase

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object Constants {
    object SwerveConstants {
        const val CANBusName = "swerve"
    }

    object VisionConstants {
        val robotToCam: Transform3d = Transform3d(
            Translation3d(
                Units.inchesToMeters(13.967415),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(5.070727 - 1.7),
            ),
            Rotation3d(0.0, Units.degreesToRadians(20.0), 0.0),
        )

        val aprilTagField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()

        val singleTagStdDevs: Matrix<N3, N1> = VecBuilder.fill(4.0, 4.0, 8.0)
    }

    object OperatorConstants {
        enum class ControllerType {
            FLIGHTSTICK,
            GAMEPAD,
        }

        const val DRIVER_CONTROLLER_PORT = 0

        val DRIVER_CONTROLLER_TYPE: ControllerType = ControllerType.FLIGHTSTICK
    }

    object RobotConstants {
        enum class Mode {
            REAL,
            SIM,
            REPLAY,
        }
        private val simMode: Mode = Mode.SIM
        val mode: Mode = if (RobotBase.isReal()) Mode.REAL else simMode
    }
}
