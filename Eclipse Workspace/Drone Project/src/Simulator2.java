import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


public class Simulator2 {
	
	/* This is an area for configuring the changeable variables for the system */
	
		/* >> Motor and Propeller Characteristics */
			//Maximum speed at which the rotors can spin, limiting maximum system response
			static int maxRPM = 0;
		
		
		/* >> Simulator configuration and values */
			//Rate at which the controller runs, in Hz
			static int controlFrequency = 10;
			//Length of time for which the simulator should run in milliseconds
			static long simLength = 5000;
	
	/* End of config area */
	
	//Other variables
	static long milliseconds = 0; //Keeps track of time as an integer value to avoid round-off error that comes with floating points.
	
	static int runController = 1000/controlFrequency;
	static double state = 0.0; //the state of the system (ie, the angle of the drone)
	
	public static void main (String[] args) throws IOException{
		
		//Create a file for writing the states to, as well as a writer to write the states.
		int num = 0;
		while(new File("./output"+num+".csv").exists()){
			num++;
		}
		FileWriter writer = new FileWriter("./output"+num+".csv");
		//Setup the column headers for the output csv file.
		writer.write("Time" + "," + "Value" + "\n");
		
		//Instantiates the control system, and runs it.
		
		
		//runs the simulation for the specified number of milliseconds
		for(long time = 0; time < simLength; time++){
			
		}
	}

}
