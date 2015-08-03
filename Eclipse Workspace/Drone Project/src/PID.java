
public class PID {
	
	double kp, ki, kd;
	State state = new State(0, 0);
	double goal;
	double intVal; //Stores the value of the integral term
	double output;
	
	
	/**
	 * Constructor for the PID system
	 * @param goal The setvalue, or the desired result value of the system
	 * @param Kp The gain value for the proportional term
	 * @param Ki The gain value for the integral term
	 * @param Kd The gain value for the derivative term
	 */
	public PID (double goal, double Kp, double Ki, double Kd){
		this.kp = Kp;
		this.ki = Ki;
		this.kd = Kd;
		this.goal = goal;
		this.intVal = 0.0d;
		this.output = 0.0;
	}
	
	public void update(State currState){	
		State past = new State(state);
		state.set(currState);

		double out = 0.0d;
		
		//Proportional reaction to the difference in state
		out += kp*(goal-state.getVal());
		
		
		//Derivative reaction to the difference in state
		if(state.getTime() != past.getTime()){
			out -= kd* ( (state.getVal() - past.getVal()) / (state.getTime() - past.getTime()) );
		}
		
		//Integral reaction to the difference in state
		intVal += (past.getVal() * (state.getTime() - past.getTime())) + //area of the rectangle formed
				( state.getVal() - past.getVal() *  (state.getTime() - past.getTime() / 2)); 
				//Area of the triangle formed by the change, and negative if the value goes down, so will subtract from the area
		out+= ki * intVal; //intVal constantly changes by summing all the integral errors over time
		
		this.output = out;
	}
	
	public double getOutput(){
		return this.output;
	}
	
}
