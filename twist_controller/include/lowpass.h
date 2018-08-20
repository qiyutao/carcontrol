#pragma once

class LowPassFilter
{
  private:
    double a, b, last_val;
    bool ready;

  public:
    LowPassFilter();
    void set(double tau, double ts);

    double get();

    double filt(double val);
};