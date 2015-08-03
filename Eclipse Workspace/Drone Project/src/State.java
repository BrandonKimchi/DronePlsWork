
public class State {
	
	private double value;
	private long time;
	
	/**
	 * Constructor that takes value for the state
	 * @param val The value of the system at that state
	 * @param time The time at which the state was
	 */
	public State(double val, long time){
		this.value = val;
		this.time = time;
	}
	
	/**
	 * Constructor for a State which copies data from another state
	 * @param state The state to duplicate
	 */
	public State(State state){
		this.value = state.getVal();
		this.time = state.getTime();
	}
	
	/**
	 * Gives the time of the state
	 * @return double value for time
	 */
	public long getTime(){
		return time;
	}
	
	/**
	 * Gives the value of that state
	 * @return double value of state
	 */
	public double getVal(){
		return value;
	}
	
	/**
	 * Sets the value of the state to the values of another state
	 * @param state The state to copy values from
	 */
	public void set(State state){
		this.value = state.getVal();
		this.time = state.getTime();
	}

}
