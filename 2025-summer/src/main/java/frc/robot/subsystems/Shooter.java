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
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private SparkMaxConfig motorConfig;
    private SparkMax m_upperMotor;
    private SparkMax m_lowerMotor;
    private SparkMax m_indexMotor;
    private SparkClosedLoopController m_upperpidcontroller;
    private SparkClosedLoopController m_lowerpidcontroller;
    private SparkClosedLoopController m_indexpidcontroller;
    private RelativeEncoder upperEncoder;
    private RelativeEncoder lowerEncoder;
    private RelativeEncoder indexEncoder;


    public Shooter() {
        motorConfig = new SparkMaxConfig();

        m_upperMotor = new SparkMax(ShooterConstants.UpperMotorID, MotorType.kBrushless);
        m_upperpidcontroller = m_upperMotor.getClosedLoopController();
        upperEncoder = m_upperMotor.getEncoder();

        m_lowerMotor = new SparkMax(ShooterConstants.LowerMotorID, MotorType.kBrushless);
        m_lowerpidcontroller = m_lowerMotor.getClosedLoopController();
        lowerEncoder = m_lowerMotor.getEncoder();

        m_indexMotor = new SparkMax(ShooterConstants.IndexMotorID, MotorType.kBrushless);
        m_indexpidcontroller = m_indexMotor.getClosedLoopController();
        indexEncoder = m_indexMotor.getEncoder();

        motorConfig.smartCurrentLimit(20);

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
        
        m_upperMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_lowerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_indexMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command shoot(){
        return this.run(() -> {
            m_upperpidcontroller.setReference(
                ShooterConstants.motorSpeed, 
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
            );
            m_lowerpidcontroller.setReference(
                ShooterConstants.motorSpeed, 
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command intake(){
        return this.run(() -> {
            m_upperpidcontroller.setReference(
                -ShooterConstants.indexSpeed, 
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
            );
            m_lowerpidcontroller.setReference(
                -ShooterConstants.indexSpeed, 
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
            );
            m_indexpidcontroller.setReference(
                -ShooterConstants.indexSpeed, 
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command stop(){
        return this.run(() -> {
            m_upperMotor.set(0);
            m_lowerMotor.set(0);
        });
    }

    public Command feed(){
        return this.run(() -> {
            //switch from automatic note feeding to manual
            m_indexpidcontroller.setReference(
                ShooterConstants.indexSpeed, 
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
            );
        });
    }

    public Command indexStop(){
        return this.run(() -> {
            m_indexMotor.set(0);
        });
    }

    public void periodic () {

        SmartDashboard.setDefaultNumber("Shooter Wheels Speed:", ShooterConstants.motorSpeed);
    }
}