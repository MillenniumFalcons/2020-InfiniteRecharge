package frc.team3647Subsystems;

/**
 * basically 254's code 
 */
public interface PeriodicSubsystem {

    public void init();
    public void end();

    public void readPeriodicInputs();
    public void writePeriodicOutputs();
    
    public abstract String getName();
}