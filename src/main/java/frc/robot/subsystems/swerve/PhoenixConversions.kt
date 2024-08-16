package frc.robot.subsystems.swerve


/**
 * @param falconRotations Falcon rotations
 * @param gearRatio gear ratio between Falcon and mechanism
 * @return degrees of rotation of mechanism
 */
fun falconRotationsToMechanismDegrees(falconRotations: Double, gearRatio: Double): Double {
    return falconRotations * 360.0 / gearRatio
}

/**
 * @param degrees Degrees of rotation of mechanism
 * @param gearRatio gear ratio between Falcon and mechanism
 * @return Falcon rotations
 */
fun degreesToFalconRotations(degrees: Double, gearRatio: Double): Double {
    return (degrees / 360.0) * gearRatio
}

/**
 * @param rps Falcon rotations per second
 * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
 * @return RPM of mechanism
 */
fun falconRPSToMechanismRPM(falconRPS: Double, gearRatio: Double): Double {
    val motorRPM = falconRPS * 60.0
    return motorRPM / gearRatio
}

/**
 * @param RPM RPM of mechanism
 * @param gearRatio Gear ratio between Falcon and mechanism (set to 1 for Falcon RPS)
 * @return Falcon rotations per second
 */
fun rpmToFalconRPS(rpm: Double, gearRatio: Double): Double {
    val motorRPM = rpm * gearRatio
    return motorRPM / 60.0
}

/**
 * @param falconRotations Falcon rotations
 * @param circumference circumference of wheel
 * @param gearRatio gear ratio between Falcon and mechanism
 * @return linear distance traveled by wheel in meters
 */
fun falconRotationsToMechanismMeters(
    falconRotations: Double, circumference: Double, gearRatio: Double
): Double {
    val wheelRotations = falconRotations / gearRatio
    return (wheelRotations * circumference)
}

/**
 * @param falconRPS Falcon rotations per second
 * @param circumference circumference of wheel
 * @param gearRatio gear ratio between Falcon and mechanism
 * @return mechanism linear velocity in meters per second
 */
fun falconRPSToMechanismMPS(
    falconRPS: Double, circumference: Double, gearRatio: Double
): Double {
    val wheelRPM = falconRPSToMechanismRPM(falconRPS, gearRatio)
    return (wheelRPM * circumference) / 60
}

/**
 * @param velocity velocity in meters per second
 * @param circumference circumference of wheel
 * @param gearRatio gear ratio between Falcon and mechanism
 * @return Falcon rotations per second
 */
fun mpsToFalconRPS(velocity: Double, circumference: Double, gearRatio: Double): Double {
    val wheelRPM = ((velocity * 60) / circumference)
    return rpmToFalconRPS(wheelRPM, gearRatio)
}