package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

public class AgitatorSys extends SubsystemBase {
    
    private final SparkMax agitatorMtr;
    private final RelativeEncoder agitatorEnc;
    private final SparkClosedLoopController agitatorController;

    public AgitatorSys () {

        agitatorMtr = new SparkMax(CANDevices.agitatorMtrId, MotorType.kBrushless);
        agitatorEnc = agitatorMtr.getEncoder();
        agitatorController = agitatorMtr.getClosedLoopController();

        SparkMaxConfig agitatorConfig = new SparkMaxConfig();
        agitatorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        agitatorConfig.encoder
            .positionConversionFactor(25)
            .velocityConversionFactor(25);

    }

    public void setAgitatorRPM() {
        agitatorMtr.set(0.5);
    }

    public void getAgitatorRPM() {
        agitatorEnc.getVelocity();
    }

    public void stop() {
        agitatorMtr.set(0);
    }

}
