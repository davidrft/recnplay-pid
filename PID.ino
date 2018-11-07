class PID {
public:
    unsigned long last_time;
    double input, output, setpoint;
    double error_sum, last_error;
    double _kp, _ki, _kd;

    void compute()
    {
        unsigned long now = millis()
        double time_diff = (double)(now - last_time);

        double error = setpoint - input;
        error_sum += (error * time_diff);
        double diff_error = (error - last_error);

        output = (_kp * error) + (_ki * error_sum) + (_kd * diff_error);

        last_error = error;
        last_time = now;
    }

    void set_tunings(double kp, double, ki, double kd) 
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }
}