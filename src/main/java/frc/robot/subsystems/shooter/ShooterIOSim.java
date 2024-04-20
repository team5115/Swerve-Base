package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final FlywheelSim leftSim;
    private final FlywheelSim rightSim;
    private final FlywheelSim auxSim;
    private final DCMotorSim intakeSim;
    private final DCMotorSim amperSim;

    private double leftAppliedVolts;
    private double rightAppliedVolts;
    private double auxAppliedVolts;
    private double intakeAppliedVolts;
    private double amperAppliedVolts;

    public ShooterIOSim() {
        // TODO determine gearings and moments of inertia for the shooter and intake motors

        leftSim = new FlywheelSim(DCMotor.getNEO(1), 0, 0);
        rightSim = new FlywheelSim(DCMotor.getNEO(1), 0, 0);
        auxSim = new FlywheelSim(DCMotor.getNEO(1), 0, 0);

        intakeSim = new DCMotorSim(DCMotor.getNEO(1), 0, 0);

        // TODO determine traits for the snowblower motor
        DCMotor snowblowerPlant = new DCMotor(0, 0, 0, 0, 0, 0);
        amperSim = new DCMotorSim(snowblowerPlant, 0, 0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftSim.update(LOOP_PERIOD_SECS);
        rightSim.update(LOOP_PERIOD_SECS);
        auxSim.update(LOOP_PERIOD_SECS);
        intakeSim.update(LOOP_PERIOD_SECS);
        amperSim.update(LOOP_PERIOD_SECS);

        inputs.leftVelocityRPM = leftSim.getAngularVelocityRPM();
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());

        inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());

        inputs.auxVelocityRPM = auxSim.getAngularVelocityRPM();
        inputs.auxAppliedVolts = auxAppliedVolts;
        inputs.auxCurrentAmps = Math.abs(auxSim.getCurrentDrawAmps());

        inputs.intakeVelocityRPM = intakeSim.getAngularVelocityRPM();
        inputs.intakeAppliedVolts = intakeAppliedVolts;
        inputs.intakeCurrentAmps = Math.abs(intakeSim.getCurrentDrawAmps());

        inputs.amperPosition = Rotation2d.fromRadians(amperSim.getAngularPositionRad());
        inputs.amperAppliedVolts = amperAppliedVolts;
        inputs.amperCurrentAmps = Math.abs(amperSim.getCurrentDrawAmps());

        inputs.noteDetected = false;
    }

    @Override
    public void setAmperPercent(double percent) {
        // TODO Auto-generated method stub
        ShooterIO.super.setAmperPercent(percent);
    }

    @Override
    public void setAuxBrakeMode(boolean enable) {
        // TODO Auto-generated method stub
        ShooterIO.super.setAuxBrakeMode(enable);
    }

    @Override
    public void setAuxPercent(double percent) {
        // TODO Auto-generated method stub
        ShooterIO.super.setAuxPercent(percent);
    }

    @Override
    public void setAuxVoltage(double volts) {
        // TODO Auto-generated method stub
        ShooterIO.super.setAuxVoltage(volts);
    }

    @Override
    public void setIntakeBrakeMode(boolean enable) {
        // TODO Auto-generated method stub
        ShooterIO.super.setIntakeBrakeMode(enable);
    }

    @Override
    public void setIntakePercent(double percent) {
        // TODO Auto-generated method stub
        ShooterIO.super.setIntakePercent(percent);
    }

    @Override
    public void setLeftPercent(double percent) {
        // TODO Auto-generated method stub
        ShooterIO.super.setLeftPercent(percent);
    }

    @Override
    public void setLeftVoltage(double volts) {
        // TODO Auto-generated method stub
        ShooterIO.super.setLeftVoltage(volts);
    }

    @Override
    public void setRightPercent(double percent) {
        // TODO Auto-generated method stub
        ShooterIO.super.setRightPercent(percent);
    }

    @Override
    public void setRightVoltage(double volts) {
        // TODO Auto-generated method stub
        ShooterIO.super.setRightVoltage(volts);
    }

    @Override
    public void setSidesBrakeMode(boolean enable) {
        // TODO Auto-generated method stub
        ShooterIO.super.setSidesBrakeMode(enable);
    }
}
