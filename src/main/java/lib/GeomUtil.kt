package lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
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

fun Pose2d.flip() = Pose2d(this.translation.flip(), this.rotation.flip())

fun Translation2d.flip() = Translation2d(16.54 - this.x, this.y)

fun Rotation2d.flip(): Rotation2d = this.rotateBy(Rotation2d.fromDegrees(180.0))

fun Pose3d.flip() = Pose3d(this.translation.flip(), this.rotation.flip())

fun Translation3d.flip() = Translation3d(16.54 - this.x, this.y, this.z)

fun Rotation3d.flip(): Rotation3d = this.rotateBy(Rotation3d(0.0, 0.0, Math.PI))

fun Gamepiece.flip() = Gamepiece(this.first.flip(), this.second)
