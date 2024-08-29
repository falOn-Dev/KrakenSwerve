package frc.robot.commands.swerve

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.Drivetrain
import lib.MotionCommand

class SimpleMoveCommand(private val command: MotionCommand, private val swerve: Drivetrain) : Command() {
    private var startPose = swerve.pose

    private val direction = Direction.valueOf(command.direction.uppercase())
    private val target = if(direction == Direction.CLOCKWISE || direction == Direction.COUNTERCLOCKWISE) command.value else Units.inchesToMeters(command.value)

    override fun execute() {
        when (direction) {
            Direction.NORTH -> swerve.applyChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0.2, 0.0, 0.0, swerve.gyroInputs.yaw))
            Direction.SOUTH -> swerve.applyChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(-0.2, 0.0, 0.0, swerve.gyroInputs.yaw))
            Direction.WEST -> swerve.applyChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.2, 0.0, swerve.gyroInputs.yaw))
            Direction.EAST -> swerve.applyChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, -0.2, 0.0, swerve.gyroInputs.yaw))
            Direction.CLOCKWISE -> swerve.applyChassisSpeeds(ChassisSpeeds(0.0, 0.0, 1.0))
            Direction.COUNTERCLOCKWISE -> swerve.applyChassisSpeeds(ChassisSpeeds(0.0, 0.0, -1.0))
        }

        when(direction){
            Direction.NORTH -> println("${swerve.pose.translation.x} >= ${startPose.translation.x + target}")
            Direction.SOUTH -> println("${swerve.pose.translation.x} <= ${startPose.translation.x - target}")
            Direction.WEST -> println("${swerve.pose.translation.y} >= ${startPose.translation.y + target}")
            Direction.EAST -> println("${swerve.pose.translation.y} <= ${startPose.translation.y - target}")
            Direction.CLOCKWISE -> println("${swerve.pose.rotation.degrees} >= ${startPose.rotation.degrees + target}")
            Direction.COUNTERCLOCKWISE -> println("${swerve.pose.rotation.degrees} <= ${startPose.rotation.degrees - target}")
        }
    }

    override fun isFinished(): Boolean {
        when(direction){
            Direction.NORTH -> return swerve.pose.translation.x >= startPose.translation.x + target
            Direction.SOUTH -> return swerve.pose.translation.x <= startPose.translation.x - target
            Direction.WEST -> return swerve.pose.translation.y >= startPose.translation.y + target
            Direction.EAST -> return swerve.pose.translation.y <= startPose.translation.y - target
            Direction.CLOCKWISE -> return swerve.pose.rotation.degrees >= startPose.rotation.degrees + target
            Direction.COUNTERCLOCKWISE -> return swerve.pose.rotation.degrees <= startPose.rotation.degrees - target
        }
    }


    enum class Direction {
        NORTH,
        SOUTH,
        WEST,
        EAST,
        CLOCKWISE,
        COUNTERCLOCKWISE
    }
}