package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmIOSparkMax implements ArmIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final AbsoluteEncoder absoluteEncoder;

    public ArmIOSparkMax() {
        leftMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightMotor = new CANSparkMax(0, MotorType.kBrushless);
        absoluteEncoder = leftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    }

    @Override
    public void setArmVoltage(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armVelocityRPM = absoluteEncoder.getVelocity();
        inputs.armAngle = Rotation2d.fromRotations(absoluteEncoder.getPosition());
        inputs.armCurrentAmps = (leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent()) / 2.0;
        inputs.armAppliedVolts =
                (leftMotor.getAppliedOutput() * leftMotor.getBusVoltage()
                                + rightMotor.getAppliedOutput() * rightMotor.getBusVoltage())
                        / 2.0;
    }
}
