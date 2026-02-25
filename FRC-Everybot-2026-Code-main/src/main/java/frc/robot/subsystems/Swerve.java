package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private final double TRACK_WIDTH_INCHES = 24.5;
    private final double WHEEL_BASE_INCHES = 23;

    private final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);
    private final double WHEEL_BASE_METERS = Units.inchesToMeters(WHEEL_BASE_INCHES);

    private final SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / -2),
            new Translation2d(WHEEL_BASE_METERS / -2, TRACK_WIDTH_METERS / 2),
            new Translation2d(WHEEL_BASE_METERS / -2, TRACK_WIDTH_METERS / -2));
    private final double STEERING_MOTOR_REDUCTION = 9424.0 / 203.0;
    private final double DRIVE_MOTOR_PINION_TEETH = 13;
    private final double DRIVE_MOTOR_REDUCTION = (22.0 / DRIVE_MOTOR_PINION_TEETH) * (45.0 / 15.0);
    private final double DRIVE_WHEEL_DIAMETER_INCHES = 3;
    private final double DRIVE_WHEEL_CIRCUMFERANCE = Math.PI * DRIVE_WHEEL_DIAMETER_INCHES;
    private final double DRIVE_WHEEL_CIRCUMFERANCE_METERS = Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERANCE);
    private final double MAX_VELOCITY_METERS_PER_SEC = 4.46 * 0.9;

    public Swerve() {

    }

    public Command driveJoystick(DoubleSupplier forwardpercent, DoubleSupplier sidetoside, DoubleSupplier rotation) {
        return this.run(() -> {

        }).withName("Swerve.driveJoystick");
    }

}
