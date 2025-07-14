package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig; 

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;


public class Shoulder extends SubsystemBase {

    private SparkMaxConfig motorConfig;
    private SparkMax m_armMotor1;
    private SparkMax m_armMotor2;
    private SparkClosedLoopController m_pidcontroller;
    private AbsoluteEncoder m_encoder;


    public Shoulder() {
        motorConfig = new SparkMaxConfig();

        m_armMotor1 = new SparkMax(ShoulderConstants.armMortar1CanID, MotorType.kBrushless);

        m_armMotor2 = new SparkMax(ShoulderConstants.armMortar1CanID, MotorType.kBrushless);

        m_encoder = motor.getencoder();

        motorConfig.m_encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.4)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
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

        m_armMotor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_armMotor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
    
    }
    public Command IntakeTransfer(){
        return this.runOnce(() -> m_pidcontroller.setReference(ShoulderConstants.armDown, SparkMax.ControlType.kPosition));
    }
    public Command armAmp(){
        //call to specific rotation first
        //m_armMotor1.set(0);
        return this.run(() -> m_pidcontroller.setReference(ShoulderConstants.armAmp, SparkMax.ControlType.kPosition));
      }
    
    public Command armSpeaker() {
        //m_armMotor1.set(1*flip);
        //m_armMotor2.set(-1*flip);
        //m_armMotor1.get();
        return this.runOnce(() -> m_pidcontroller.setReference(ShoulderConstants.armSpeaker, SparkMax.ControlType.kPosition));
    }
}
