package frc.robot.subsystems;
//hello am robot
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
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
    private SparkMaxConfig followerMotorConfig;
    private AbsoluteEncoderConfig absEncoderConfig;
    private SparkMax m_armMotor1;
    private SparkMax m_armMotor2;
    private SparkClosedLoopController m_pidcontroller;
    private AbsoluteEncoder m_encoder;

    private RelativeEncoder encoder;

    public Shoulder() {
        motorConfig = new SparkMaxConfig();
        followerMotorConfig = new SparkMaxConfig();
        absEncoderConfig = new AbsoluteEncoderConfig();

        m_armMotor1 = new SparkMax(ShoulderConstants.armMotor1CanID, MotorType.kBrushless);

        m_armMotor2 = new SparkMax(ShoulderConstants.armMotor2CanID, MotorType.kBrushless);
        m_pidcontroller = m_armMotor2.getClosedLoopController();
        m_encoder = m_armMotor2.getAbsoluteEncoder();
        encoder = m_armMotor2.getEncoder();

        //LEADER CONFIG
        //motorConfig.inverted(true);

        motorConfig.smartCurrentLimit(20);

        motorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
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

        absEncoderConfig.zeroOffset(0.98);
        motorConfig.apply(absEncoderConfig);

        m_armMotor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //FOLLOWER CONFIG

        followerMotorConfig.follow(ShoulderConstants.armMotor2CanID, true);
        m_armMotor1.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }

    public Command IntakeTransfer(){
        return this.runOnce(() -> {
            m_pidcontroller.setReference(
                ShoulderConstants.armDown, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
            //m_armMotor2.set(0);
        });
    }

    public Command armAmp(){
        //call to specific rotation first
        //m_armMotor1.set(0);
        return this.run(() -> {
            m_pidcontroller.setReference(
                ShoulderConstants.armAmp, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
        });
    }
    
    public Command armSpeaker() {
        return this.run(() -> {
            m_pidcontroller.setReference(
                ShoulderConstants.armSpeaker, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public void periodic () {

        SmartDashboard.putNumber("encoder position", m_encoder.getPosition());
    }
}
