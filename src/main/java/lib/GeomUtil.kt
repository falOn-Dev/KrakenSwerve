package lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.abs

fun Pose2d.near(other: Pose2d, epsilon: Double = 1E-9): Boolean {
    return this.translation.near(other.translation, epsilon) && this.rotation.near(other.rotation, epsilon)
}

fun Rotation2d.near(other: Rotation2d, epsilon: Double = 1E-9): Boolean {
    return this.cos.near(other.cos, epsilon) && this.sin.near(other.sin, epsilon)
}

fun Translation2d.near(other: Translation2d, epsilon: Double = 1E-9): Boolean {
    return this.x.near(other.x, epsilon) && this.y.near(other.y, epsilon)
}

fun Double.near(other: Double, epsilon: Double = 1E-9): Boolean {
    return abs(this - other) < epsilon
}