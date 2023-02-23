package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.hardware.Device;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;

class SwerveModule implements Sendable {
    
    // TODO: Calculate this value
    private static final double
        METERS_PER_SEC_TO_DRIVE_VOLTS = 1;
    
    private static final SimpleMotorFeedforward STEER_FEEDFORWARD = new SimpleMotorFeedforward(0.14, 1);
    private static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.2, 1);
    
    public static double getMaxDriveSpeedMetersPerSec () {
        return RobotController.getBatteryVoltage() / METERS_PER_SEC_TO_DRIVE_VOLTS;
    }
    
    private static Device<CANSparkMax> initializeMotor (String robotRelativeModulePosition, boolean isSteer) {

        String motorType = "DRIVE";
        if (isSteer) motorType = "STEER";

        Device<CANSparkMax> motor = new Device<>(
            "CAN.MOTOR_CONTROLLER.SWERVE." + robotRelativeModulePosition + "." + motorType + "_MOTOR", 
            id -> {
                CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);
                sparkMax.setIdleMode(IdleMode.kBrake);
                return sparkMax;
            }, 
            sparkMax -> {
                sparkMax.stopMotor();
                sparkMax.close();
            });
        
        return motor;
    }

    private static Device<ResettableEncoder> initializeEncoder (String robotRelativeModulePosition) {
        Device<ResettableEncoder> steerEncoder = new Device<>(
            "CAN.MOTOR_ENCODER.SWERVE."+robotRelativeModulePosition+".CANCODER", 
            id -> {
                ResettableEncoder encoder = new ResettableEncoder(id);
                encoder.setInverted(false);
                return encoder;
            }, 
            null);
        
        return steerEncoder;
    }
    
    private final Device<CANSparkMax> driveMotor, steerMotor;
    private final Device<ResettableEncoder> steerEncoder;
    
    private double
        DS_steerOutputVoltage = 0,
        DS_driveOutputVoltage = 0,
        DS_unoptimizedDesiredRotation = 0,
        DS_desiredRotation = 0,
        DS_desiredDriveSpeed = 0;
    
    private boolean DS_driveEnabled = true;
    
    /**
     * 1 unit input for this PID controller is a full 360 deg rotation.
     * 1 unit output for this PID controller is one volt applied to the steer motor.
     */
    private final RotationalPID steerPID;
    
    public SwerveModule (String robotRelativeModulePosition) {
        steerPID = new RotationalPID(6/90., 0, 0, 6);      
        driveMotor = initializeMotor(robotRelativeModulePosition, false);
        steerMotor = initializeMotor(robotRelativeModulePosition, true);
        steerEncoder = initializeEncoder(robotRelativeModulePosition);
    }
    
    /**
     * Update this module's motor controllers to try to set the module to the desired speed and angle.
     * This method must be called periodically.
     * @param desiredState The desired {@link SwerveModuleState}.
     */
    public void update (SwerveModuleState desiredState) {
        DS_unoptimizedDesiredRotation = desiredState.angle.getDegrees();
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, getRotation());
        updateDriveMotor(optimizedDesiredState.speedMetersPerSecond);
        
        if (desiredState.speedMetersPerSecond != 0) {
            updateSteerMotor(optimizedDesiredState.angle);
        } else {
            steerMotor.get().setVoltage(0);
        }
    }
    
    public void xMode (boolean turnDir) {
        SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(45 + (turnDir ? 90 : 0)));
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getRotation());
        updateDriveMotor(0);
        updateSteerMotor(optimizedState.angle);
    }
    
    private void updateDriveMotor (double desiredSpeedMetersPerSec) {
        DS_desiredDriveSpeed = desiredSpeedMetersPerSec;

        double voltsOutput = DRIVE_FEEDFORWARD.calculate(METERS_PER_SEC_TO_DRIVE_VOLTS * desiredSpeedMetersPerSec);
        DS_driveOutputVoltage = voltsOutput;

        driveMotor.get().setVoltage(DS_driveEnabled ? voltsOutput : 0);
    }
    
    private void updateSteerMotor (Rotation2d desiredRotation) {
        DS_desiredRotation = desiredRotation.getDegrees();

        double voltsOutput = STEER_FEEDFORWARD.calculate(steerPID.calculate(getRotation(), desiredRotation));
        DS_steerOutputVoltage = voltsOutput;

        steerMotor.get().setVoltage(DS_driveEnabled ? voltsOutput : 0);
    }
    
    /**
     * Stop all motor controllers assigned to this {@link SwerveModule}.
     */
    public void stop () {
        driveMotor.get().stopMotor();
        DS_desiredDriveSpeed = 0;
        DS_driveOutputVoltage = 0;
        steerMotor.get().stopMotor();
        DS_steerOutputVoltage = 0;
    }
    
    /**
     * Zero the steer encoder and save its offset to the encoder.
     */
    public void zeroSteerEncoder () {
        steerEncoder.get().zeroRotation();
    }
    
    public Rotation2d getRotation () {
        return steerEncoder.get().getRotation();
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("unop-desiredRotation", () -> DS_unoptimizedDesiredRotation, null);
        builder.addDoubleProperty("desiredRotation", () -> DS_desiredRotation, null);
        builder.addDoubleProperty("desiredDriveSpeed", () -> DS_desiredDriveSpeed, null);
        builder.addDoubleProperty("currentRotation", () -> steerEncoder.get().getRotation().getDegrees(), null);
        builder.addDoubleProperty("steerOutputVoltage", () -> DS_steerOutputVoltage, null);
        builder.addDoubleProperty("driveOutputVoltage", () -> DS_driveOutputVoltage, null);

        builder.addBooleanProperty("Enabled Drive", () -> DS_driveEnabled, e -> DS_driveEnabled = e);
    }
    
}
