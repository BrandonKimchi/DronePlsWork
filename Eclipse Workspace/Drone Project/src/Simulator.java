import java.io.*;

public class Simulator {
	
	public static void main(String[] args) throws IOException{
		
		//Filewriter thingies
		int num = 0;
		while(new File("./output"+num+".csv").exists()){
			num++;
		}
		FileWriter writer = new FileWriter("./output"+num+".csv");
		writer.write("Time" + "," + "Value" + "\n");
		
		PID pid = new PID(5.0, 10, 3, 3);
		double time = 0.0;
		
		double rate = 0.001;
		double processRate = 0.1;
		
		double value = 0.0;
		double changeRate = 0.0;
		while(time < 1){
			System.out.println("Time: " + time + "\tValue: " + value);
			writer.write(time + "," + (double)Math.round(value*100)/100.0 + "\n");
			if( ((int) (time*100 % 10)) % 10 == 0){
				//pid.update(new State(value, time));
				double pidOutput = pid.getOutput();
				changeRate = pidOutput;
System.out.println("Correction System has run.\nNew correction rate is: " + changeRate);
			}
			
			time += rate;
			value += changeRate*rate;
			int temp = (int) Math.round(time*1000);
			time = temp/1000.0;
		}
		
		writer.flush();
		writer.close();
	}

}
