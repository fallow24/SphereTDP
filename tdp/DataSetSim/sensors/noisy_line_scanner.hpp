#ifndef NOISY_LINE_SCANNER_H
#define NOISY_LINE_SCANNER_H

#include "line_scanner.hpp"

/* A noisy line scanner with an arbitrary FOV */
class Noisy_Line_Scanner: public Line_Scanner
{

public:
    Noisy_Line_Scanner(pair<double, double> range,
                      pair<double, double> fov,
                      pair<double, double> resolution,
                      double range_noise_mean,
                      double range_noise_std):
        Line_Scanner(range, fov, resolution)
    {
        this->range_distribution = normal_distribution<double>(range_noise_mean, range_noise_std);
    }

    ~Noisy_Line_Scanner(){};

    /* Returns a random value sampled from a random distribution
     * that will be added on all range measurements */
    virtual double get_range_noise();
private:
    // Mersenne twister PRNG, initialized with seed from previous
    // random device instance
    mt19937 randomness_source;
    normal_distribution<double> range_distribution;
};

#endif //NOISY_LINE_SCANNER_H