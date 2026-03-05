package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import pabeles.concurrency.IntOperatorTask.Max;

public class SwerveModule {

    private static final SparkMaxConfig STEERING_CONFIG = new SparkMaxConfig();
    private static final SparkMaxConfig DRIVING_CONFIG = new SparkMaxConfig();
    private static final double STEERING_MOTOR_REDUCTION = 9424.0 / 203.0;
    private static final double DRIVE_MOTOR_PINION_TEETH = 13;
    private static final double DRIVE_MOTOR_REDUCTION = 1/((22.0 / DRIVE_MOTOR_PINION_TEETH) * (45.0 / 15.0));
    private static final double DRIVE_WHEEL_DIAMETER_INCHES = 3;
    private static final double DRIVE_WHEEL_CIRCUMFERANCE = Math.PI * DRIVE_WHEEL_DIAMETER_INCHES;
    private static final double DRIVE_WHEEL_CIRCUMFERANCE_METERS = Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERANCE);
    private static final double MAX_VELOCITY_METERS_PER_SEC = 4.46 * 0.9;
    static {
        STEERING_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        STEERING_CONFIG.absoluteEncoder.inverted(true).positionConversionFactor(2 * Math.PI)
                .velocityConversionFactor(2 * Math.PI / 60).apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
        STEERING_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1, 0, 0)
                .positionWrappingEnabled(true).positionWrappingInputRange(0, 2 * Math.PI);
        DRIVING_CONFIG.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
        DRIVING_CONFIG.encoder.positionConversionFactor(DRIVE_MOTOR_REDUCTION * DRIVE_WHEEL_CIRCUMFERANCE_METERS)
                .velocityConversionFactor(DRIVE_MOTOR_REDUCTION * DRIVE_WHEEL_CIRCUMFERANCE_METERS / 60);
        DRIVING_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04, 0, 0).feedForward
                .kV(12 / MAX_VELOCITY_METERS_PER_SEC);
                

    }
    private SparkMax steering;
    private SparkMax driving;
    private SwerveModuleState desired = new SwerveModuleState();
    private double offset;

    public SwerveModule(int steeringID, int drivingID, double offset) {
        steering = new SparkMax(steeringID, MotorType.kBrushless);
        driving = new SparkMax(drivingID, MotorType.kBrushless);
        steering.configure(STEERING_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driving.configure(DRIVING_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        desired.angle = new Rotation2d(steering.getAbsoluteEncoder().getPosition());
        driving.getEncoder().setPosition(0);
        this.offset = offset;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driving.getEncoder().getVelocity(),
                new Rotation2d(steering.getAbsoluteEncoder().getPosition() - offset));

    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driving.getEncoder().getPosition(),
                new Rotation2d(steering.getAbsoluteEncoder().getPosition() - offset));
    }

    public void SetDesiredState(SwerveModuleState desiSwerveModuleState) {
        SwerveModuleState rotated = new SwerveModuleState(desiSwerveModuleState.speedMetersPerSecond,
                desiSwerveModuleState.angle.plus(new Rotation2d(offset)));
        rotated.optimize(new Rotation2d(steering.getAbsoluteEncoder().getPosition()));
        steering.getClosedLoopController().setSetpoint(rotated.angle.getRadians(), ControlType.kPosition);
        driving.getClosedLoopController().setSetpoint(rotated.speedMetersPerSecond, ControlType.kVelocity);
        this.desired = desiSwerveModuleState;

    }

}
