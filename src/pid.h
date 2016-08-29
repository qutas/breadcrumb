class pid{

	public:
		pid(double dt, double Kp = 1.0, double Ki = 0.0, double Kd = 0.0, double min_output=-1.0, double max_output=1.0);

		void reset();

		double step(double setpoint, double measured_value);

		void setKp(double p);
		void setKi(double i);
		void setKd(double d);

		void setOutputMin(double min);
		void setOutputMax(double max);

	public:
		double Kd;
		double Kp;
		double Ki;

		double min_output;
		double max_output;

	private:
		double integral;
		double previous_error;

		double dt;
};
