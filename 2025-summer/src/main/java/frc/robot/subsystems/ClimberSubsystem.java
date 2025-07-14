package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;


public class ClimberSubsystem extends SubsystemBase {
    private SparkMax m_climberMotorL;
    private SparkMax m_climberMotorR;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    public ClimberSubsystem() {
        m_climberMotorL = new SparkMax(17, MotorType.kBrushless);
        closedLoopController = m_climberMotorL.getClosedLoopController();
        encoder = m_climberMotorL.getEncoder();

        m_climberMotorR = new SparkMax(18, MotorType.kBrushless);
        closedLoopController = m_climberMotorR.getClosedLoopController();
        encoder = m_climberMotorR.getEncoder();
    
        /*
         * Create a new SPARK MAX configuration object. This will store the
         * configuration parameters for the SPARK MAX that we will set below.
         */
        motorConfig = new SparkMaxConfig();
    
        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        motorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

            /*
            * Apply the configuration to the SPARK MAX.
            *
            * kResetSafeParameters is used to get the SPARK MAX to a known state. This
            * is useful in case the SPARK MAX is replaced.
            *
            * kPersistParameters is used to ensure the configuration is not lost when
            * the SPARK MAX loses power. This is useful for power cycles that may occur
            * mid-operation.
            */
            m_climberMotorL.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            m_climberMotorR.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Initialize dashboard values
            SmartDashboard.setDefaultNumber("Target Position", 0);
            SmartDashboard.setDefaultNumber("Target Velocity", 0);
            SmartDashboard.setDefaultBoolean("Control Mode", false);
            SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }
}
