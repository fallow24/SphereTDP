#include <iostream>
#include <vector>

#include "line_scanner.hpp"

using namespace std;

/* This is an ideal sensor. Hence no noise */
double Line_Scanner::get_range_noise()
{
  return 0;
}

/* Assuming an ideal line scanner that goes over the entire
 * FOV with the resolution that's specified */
void Line_Scanner::get_sensing_pattern(vector<pair<double, double> > &pattern,
                                          double dt)
{
  double elev_max = get<0>(this->get_fov())*0.5,
         elev_res = get<0>(this->get_resolution());

  double azi_max = get<1>(this->get_fov())*0.5,
         azi_res = get<1>(this->get_resolution());

  for(double elev = - get<0>(this->get_fov())*0.5; elev <= elev_max; elev += elev_res)
  {
      for(double azi = - get<1>(this->get_fov())*0.5 ; azi <= azi_max; azi += azi_res)
      {
          pattern.push_back(make_pair(azi, elev));
      }
  }
}

