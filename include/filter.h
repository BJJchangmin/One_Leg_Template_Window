#ifndef __FILTER_H__
#define __FILTER_H__

double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * pi * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq)
{
    //double cutoff_freq = 100;
    double time_const = 1 / (2 * pi * cutoff_freq);
    double output = 0;

    output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

#endif // !__FILTER_H__
