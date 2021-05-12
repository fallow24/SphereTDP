#ifndef LIVOX_MID_40_H
#define LIVOX_MID_40_H

#include "noisy_line_scanner.hpp"

/* A Noisy Livox-Mid-40 scanner with the "flower"
 * scanning pattern. */
class LivoxMid40: public Noisy_Line_Scanner
{
private:
  double azimuth_offset;
  double internal_time = 0;
public:
  LivoxMid40(pair<double, double> range,
            pair<double, double> fov,
            pair<double, double> resolution,
            double range_noise_mean,
            double range_noise_std,
            double azimuth_offset = 0):
       Noisy_Line_Scanner(range, fov, resolution,
                         range_noise_mean, range_noise_std)
  {
    if(abs(get<0>(fov) - get<1>(fov)) > 0.1)
    {
      cerr << "ERROR: Livox Mid 40 scanner can only provide a "
           << "circular field of view. I.e. FOV_ELEVATION == FOV_AZIMUTH. "
           << "Given was Elev = " << get<0>(fov) << " Azi = " << get<1>(fov)
           <<  endl;
      exit(1);
    }
    if(get<0>(fov) >= 180.0 || get<1>(fov) >= 180.0)
    {
      cerr << "ERROR: Cannot choose fov greater or equal to 180.0 [deg] for Livox "
           << "Mid 40 scanner." << endl;
      exit(1);
    }

    this->azimuth_offset = azimuth_offset;
  }

  virtual void get_sensing_pattern(vector<pair<double, double> > &pattern,
                                   double dt);
};

#endif //LIVOX_MID_40_H