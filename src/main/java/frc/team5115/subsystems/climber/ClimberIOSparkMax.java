package frc.team5115.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.team5115.Constants;

public class ClimberIOSparkMax implements ClimberIO{

    private static final double kStatic=0;
    private static final double kSpring=0;
    private static final double kGravity=0;
    private static final double kVelocity=0;

    public final CANSparkMax leftClimb;
    public final CANSparkMax rightClimb;
    public final RelativeEncoder leftClimbEncoder;
    public final RelativeEncoder rightClimbEncoder;
    public final DigitalInput leftSensor;
    public final DigitalInput rightSensor;


    public ClimberIOSparkMax(){
        leftClimb = new CANSparkMax(Constants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
        //left climber is inverted!
        leftClimb.setInverted(true);
        leftClimb.setIdleMode(IdleMode.kBrake);
        leftClimb.setSmartCurrentLimit(45);

        rightClimb = new CANSparkMax(Constants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        rightClimb.setInverted(true);
        rightClimb.setIdleMode(IdleMode.kBrake);
        rightClimb.setSmartCurrentLimit(45);

        leftClimbEncoder = leftClimb.getEncoder();
        leftClimbEncoder.setPositionConversionFactor(1.0/16.0);

        rightClimbEncoder = rightClimb.getEncoder();
        rightClimbEncoder.setPositionConversionFactor(1.0/16.0);

        leftSensor = new DigitalInput(Constants.CLIMB_LEFT_SENSOR_ID);
        rightSensor = new DigitalInput(Constants.CLIMB_RIGHT_SENSOR_ID);
    }

    @Override
    public void setClimberSpeed(double desiredVelocity){
        double leftVoltage = kStatic * Math.signum(desiredVelocity) + kVelocity * desiredVelocity + kSpring * /*getRotations()*/ + kGravity * Math.cos(/*getRotations()*/1);
        /*leftClimb.setVoltage(voltage);
        rightClimb.setVoltage(voltage);*/
    }

    @Override
    public double[] getRotations(){
        //returns left, right
        double[] rots = {leftClimbEncoder.getPosition(), rightClimbEncoder.getPosition()};
        return rots;
    }

}
