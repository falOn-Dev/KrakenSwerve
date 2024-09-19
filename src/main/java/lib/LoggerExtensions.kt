package lib

import com.choreo.lib.ChoreoTrajectory
import edu.wpi.first.math.geometry.Pose2d
import org.littletonrobotics.junction.Logger

fun Logger.recordOutput(key: String, value: ChoreoTrajectory) {
    Logger.recordOutput(key, Pose2d.struct, *value.poses)
}
