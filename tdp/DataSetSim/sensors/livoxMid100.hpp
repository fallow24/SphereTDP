#ifndef LIVOX_MID_100_H
#define LIVOX_MID_100_H 

#include "livoxMid40.hpp"

/* A noisy Livox-Mid-100 scanner. Consits of 3 Livox-Mid-40 scanners
 * that are offset by azimuth to each other (but overlap slightly) to
 * provide a wider azimuth FOV.  */
class LivoxMid100: public Noisy_Line_Scanner
{
private:
  const double AZIMUTH_OVERLAP; // [deg]
  LivoxMid40 scan1, scan2, scan3;

public:
  LivoxMid100(pair<double, double> range,
            pair<double, double> fov,
            pair<double, double> resolution,
            double range_noise_mean,
            double range_noise_std):
       Noisy_Line_Scanner(range, fov, resolution,
                          range_noise_mean, range_noise_std),
       /* We assume a constant overlap angle and
        * calculate the offset and FOV from that
        */
       AZIMUTH_OVERLAP(8.4),
       scan1(range,
             // Calculate the mid40 fov
             make_pair(get<0>(fov), (get<1>(fov)+2*AZIMUTH_OVERLAP)*0.333333),
             resolution,
             range_noise_mean, range_noise_std,
             // Calculate the azimuth offset
             - deg_to_rad((get<1>(fov) + 2*AZIMUTH_OVERLAP)*0.333333 - AZIMUTH_OVERLAP)),
       scan2(range,
             make_pair(get<0>(fov), (get<1>(fov)+2*AZIMUTH_OVERLAP)*0.333333),
             resolution,
             range_noise_mean, range_noise_std,
             0),
       scan3(range,
             make_pair(get<0>(fov), (get<1>(fov)+2*AZIMUTH_OVERLAP)*0.333333),
             resolution,
             range_noise_mean, range_noise_std,
             deg_to_rad((get<1>(fov) + 2*AZIMUTH_OVERLAP)*0.333333 -AZIMUTH_OVERLAP))
  {}

  virtual void get_sensing_pattern(vector<pair<double, double> > &pattern,
                                   double dt);
};

#endif //LIVOX_MID_100_H