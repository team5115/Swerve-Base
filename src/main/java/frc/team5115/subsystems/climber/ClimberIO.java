package frc.team5115.subsystems.climber;

public interface ClimberIO {

    public static class ClimberIOInputs{

    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setClimberSpeed(double desiredVelocity) {}

    public default double[] getRotations(){double[] d={1,0}; return d;}
}
