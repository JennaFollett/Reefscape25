package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Arm extends SubsystemBase {
    SparkFlex m_armSpark = new SparkFlex(CANIds.kTestarmMotor, MotorType.kBrushless);
    SparkClosedLoopController m_controller = m_armSpark.getClosedLoopController();
    private double targetposition = 0;
    public static final double Floorarm = -38.6;
    public static final double Spitarm = -10;
public Arm() {

SparkFlexConfig config = new SparkFlexConfig();

// Set PID gains
config.closedLoop
    .p(0.1)
    .i(0)
    .d(0)
    .outputRange(-0.3, 0.3);

m_armSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
}
public Command ArmtopositionCommand(double Position){
    return this.runOnce(() -> this.ArmtoPosition(Position));

}
private void ArmtoPosition(double Position){
    m_controller.setReference(Position, ControlType.kPosition);
    targetposition = Position;
}

public void Nudge(double Nudgevalue){
targetposition += Nudgevalue;
targetposition = MathUtil.clamp(targetposition,-38.6,7.5);
m_controller.setReference(targetposition, ControlType.kPosition);
}


}


