package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

public class TestSys extends SubsystemBase {

    public static SparkMax m_testMtr = new SparkMax(CANDevices.m_testMtrId, MotorType.kBrushless);

    private boolean Run = false;

    public boolean Stop() {
        return Run = false;
    }

    public boolean Start() {
        return Run = true;
    }

    public boolean Run() {
        return Run = true;
    }
    
    public TestSys() {
    }

    @Override
    public void periodic() {

        if(Run == true){
            m_testMtr.set(0.5);
            Run = false;
        }
        else{
            m_testMtr.set(0);
        }


    }

}
