package frc.team3647Subsystems;

/**
 * basically 254's code 
 */
public interface Subsystem {

    public void init();
    public void end();

    public void readPeriodicInputs();
    public void writePeriodicOutputs();
    
    public void periodic(double timestamp);

    public abstract String getName();
}