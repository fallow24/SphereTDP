#include "noisy_line_scanner.hpp"

double Noisy_Line_Scanner::get_range_noise()
{
    // This randomly sampled value is a percentage of the measured range
    return this->range_distribution(this->randomness_source);
}


