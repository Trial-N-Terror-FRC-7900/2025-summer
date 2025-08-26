package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private SparkMaxConfig motorConfig;
    private SparkMax m_intakeMotor;
    private SparkClosedLoopController m_intakepidcontroller;
    private RelativeEncoder intakeEncoder;

    public Intake() {
        motorConfig = new SparkMaxConfig();

        m_intakeMotor = new SparkMax(IntakeConstants.IntakeMotorID, MotorType.kBrushless);
        m_intakepidcontroller = m_intakeMotor.getClosedLoopController();
        intakeEncoder = m_intakeMotor.getEncoder();
        
        motorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(3)
            .i(0)
            .d(0)
            .outputRange(-0.5, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        motorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .maxVelocity(1000)
            .maxAcceleration(1000)
            .allowedClosedLoopError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .maxVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);
        
        m_intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command run(){
        return this.run(() -> {
            m_intakepidcontroller.setReference(
                IntakeConstants.motorSpeed, 
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command stop(){
        return this.run(() -> {
            m_intakeMotor.set(0);
        });
    }
}
