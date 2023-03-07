package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class Arm extends SubsystemBase implements Loggable{
  
public final CANSparkMax m_armMotor;
private final AbsoluteEncoder m_Encoder;

  public Arm(){
    m_armMotor = new CANSparkMax(5, MotorType.kBrushless);
    m_Encoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);

    //set some soft limit to prevent full extension (illegal)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkIfStalled();
  }

  @Log(name = "Pivot Encoder Counts", tabName = "Arm")
  public double getAbsoluteEncoderCounts(){
    return m_Encoder.getPosition();
  }

  @Log(name = "Check if Stalled")
  public boolean checkIfStalled(){
    //Check pulse per 10 ms. If the rate of pulse immediately drops, then we are stalled.
    boolean isStalled = (m_armMotor.getOutputCurrent() < Constants.voltageDropThreshold);
    if(isStalled){
      m_armMotor.setSmartCurrentLimit(0, 0); //TBD
    }
    return isStalled;
  }

  @Log.Graph(name = "Arm Motor Pulse")
  public double getMotorPulse(){
    return m_armMotor.getOutputCurrent();
  }
  

}

}