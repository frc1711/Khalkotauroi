// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import claw.CLAWRobot;
import claw.hardware.Device;
import claw.math.InputTransform;
import claw.math.Transform;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDMap;
import frc.robot.LiveCommandTester;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
    
    /**
     * An offset in the claw's relative encoder's reading from the homing spot (fully collapsed) to the maximum
     * extension of the claw (fully released). This offset helps to prevent the claw from extending too far and
     * breaking itself.
     */
    private static final double CLAW_MAX_REACH_OFFSET = 22.3333;
    
    /**
     * An offset in the claw's relative encoder's reading from the homing spot (fully collapsed) to the minimum
     * extension of the claw (almost fully grabbing). This offset helps to prevent the claw from contracting too
     * much and breaking itself.
     */
    private static final double CLAW_MIN_REACH_OFFSET = 1;
    
    // TODO: Add javadocs
    private static final double GRAB_OUTPUT_CURRENT = 6;
    
    private static final double
        CLAW_MOVE_VOLTAGE = 4,
        CLAW_HOMING_VOLTAGE = 1.8;
    
    private static Arm armInstance;
    
    public static Arm getInstance () {
        if (armInstance == null) {
            armInstance = new Arm(
                new CANSparkMax(15, MotorType.kBrushless), 
                new CANSparkMax(14, MotorType.kBrushless), 
                new DigitalInput(IDMap.ARM_LIMIT_SWITCH)
            );
        }
        return armInstance;
    }
    
    private final CANSparkMax armMotor, clawMotor;
    
    private final Device<DutyCycle> armEncoder = new Device<>(
        "DIO.ENCODER.ARM.ARM_ENCODER",
        id -> new DutyCycle(new DigitalInput(id)),
        DutyCycle::close
    );
    
    private final Debouncer clawGrabDebouncer = new Debouncer(.8, DebounceType.kRising);
    private double clawEncoderOffset = 0;
    
    private boolean clawHasBeenHomed = false;
    private boolean isHoldingObject = false;
    
    public Arm(CANSparkMax armMotor, CANSparkMax clawMotor, DigitalInput armLimitSwitch) {
        this.armMotor = armMotor;
        this.clawMotor = clawMotor;
        
        XboxController controller = new XboxController(1);
        Transform transform = InputTransform.getInputTransform(
            InputTransform.SQUARE_CURVE,
            0.2
        );
        
        LiveCommandTester tester = new LiveCommandTester(
            "Use controller 1. Y button enables arm control, move the arm using the left joystick. " +
            "Use the A button to enable the claw homing sequence. Use left and right bumpers to " +
            "release and grab using the claw.",
            liveValues -> {
                if (controller.getAButton() && !hasClawBeenHomed()) {
                    
                    // Run homing sequence
                    runClawHomingSequence();
                    
                } else if (hasClawBeenHomed() && controller.getRightBumper()) {
                    
                    // Grab claw
                    operateClaw(ClawMovement.GRAB);
                    
                } else if (hasClawBeenHomed() && controller.getLeftBumper()) {
                    
                    // Release claw
                    operateClaw(ClawMovement.RELEASE);
                    
                } else {
                    
                    // No claw operations
                    operateClaw(ClawMovement.NONE);
                    
                }
                
                liveValues.setField("Claw can release", clawCanExtend());
                liveValues.setField("Claw can grab", clawCanContract());
                liveValues.setField("Output current", clawMotor.getAppliedOutput());
                liveValues.setField("Encoder reading", clawMotor.getEncoder().getPosition());
                
                if (controller.getYButton()) {
                    
                    double armVoltage = 7 * transform.apply(controller.getLeftY());
                    liveValues.setField("Arm voltage", armVoltage);
                    setArmVoltage(armVoltage);
                    
                } else {
                    liveValues.setField("Arm voltage", 0.0);
                    stopArm();
                }
            },
            this::stop,
            this
        );
        
        CLAWRobot.getExtensibleCommandInterpreter().addCommandProcessor(
            tester.toCommandProcessor("armtest")
        );
        
        RobotContainer.putConfigSendable("Arm Subsystem", this);
    }
    
    public void setArmVoltage(double input) {
        armMotor.setVoltage(input);
    }
    
    public void stopArm() {
        armMotor.setVoltage(0);
    }
    
    private final Debouncer homingSequenceDebouncer = new Debouncer(0.045, DebounceType.kRising);
    public void runClawHomingSequence () {
        if (homingSequenceDebouncer.calculate(clawMotor.getOutputCurrent() > GRAB_OUTPUT_CURRENT)) {
            clawEncoderOffset = getClawEncoder();
            clawMotor.setVoltage(0);
            clawHasBeenHomed = true;
        } else {
            clawMotor.setVoltage(CLAW_HOMING_VOLTAGE);
        }
    }
    
    public boolean hasClawBeenHomed () {
        return clawHasBeenHomed;
    }
    
    private double getClawEncoder () {
        return -clawMotor.getEncoder().getPosition();
    }
    
    private boolean clawCanContract () {
        return getClawEncoder() - clawEncoderOffset > CLAW_MIN_REACH_OFFSET;
    }
    
    private boolean clawCanExtend () {
        return getClawEncoder() - clawEncoderOffset < CLAW_MAX_REACH_OFFSET;
    }
    
    public void operateClaw (ClawMovement move) {
        switch (move) {
            case NONE:
                clawMotor.stopMotor();
                break;
            case GRAB:
                if (isHoldingObject || !clawCanContract()) {
                    clawMotor.stopMotor();
                } else {
                    clawMotor.setVoltage(CLAW_MOVE_VOLTAGE);
                    isHoldingObject = clawGrabDebouncer.calculate(clawMotor.getOutputCurrent() > GRAB_OUTPUT_CURRENT);
                }
                break;
            case RELEASE:
                if (!clawCanExtend()) clawMotor.stopMotor();
                else clawMotor.setVoltage(-CLAW_MOVE_VOLTAGE);
                isHoldingObject = false;
                break;
        }
    }
    
    public enum ClawMovement {
        NONE,
        GRAB,
        RELEASE;
    }
    
    public void stop() {
        stopArm();
        operateClaw(ClawMovement.NONE);
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {
        builder.addDoubleProperty("Claw output current", clawMotor::getOutputCurrent, null);
        builder.addDoubleProperty("Arm position", () -> armEncoder.get().getOutput(), null);
    }
    
}