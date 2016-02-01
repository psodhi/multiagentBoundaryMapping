/*
Generic PID controller
Author: Achal Arvind
Date: 1st Nov 2012 
*/

#define Kp 1.0				//Proportionality constant	
#define Ki 0.0				//Integral constant
#define Kd 0.0				// Derivative constant

//Define the min and max values of the command.
#define DEFAULT_MIN -1.0
#define DEFAULT_MAX 1.0

class PIDcontroller{
	private:
		float kp,kd,ki;
		float command;
	public:
		//Constructor that allows Kp, Ki, Kd to be updated
		PIDcontroller(float p, float i, float d)
		{
			kp=p;
			ki=i;
			kd=d;	
		}
		
		//Default constructor initialises controller to default Kp, Ki, Kd values (results in a proportianal controller)
		PIDcontroller()
		{
			kp=Kp;
			ki=Ki;
			kd=Kd;
		}	
		
		//Function to update PID constants
		void update_constants(float p, float i, float d)
		{
			kp=p;
			ki=i;
			kd=d;	
		}
		
		//Function to calulate PID command
		float calculate_command(double error, double *errorsum, double *errorprevious)
		{
			command=0;
			(*errorsum)+=error;				//Upate sum of errors
			command+=(kp*error);				//Proportional component
			command+=(kd*(error-(*errorprevious)));		//Derivative component
			command+=(ki*(*errorsum));				//Integral component
			(*errorprevious)=error;				//Change old error to current error
			return command;					//Send calculated command
		}
};
