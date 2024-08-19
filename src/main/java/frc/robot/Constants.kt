package frc.robot

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

    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 0
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
