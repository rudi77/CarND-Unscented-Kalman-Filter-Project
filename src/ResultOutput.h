#ifndef __RESULT_OUTPUT_H__
#define __RESULT_OUTPUT_H__

#include <string>
#include <sstream>

using namespace std;

// This represents one line in the mearuement output file
class ResultOutput
{
public:
  double p_x;     // estimated x
  double p_y;     // estimated y
  double v1;      // estimated velocity x
  double v2;      // estimated velocity y
  double m_px;    // measured x
  double m_py;    // measured y
  double x_gt;    // ground truth x
  double y_gt;    // ground truth y
  double vx_gt;   // ground truth velocity x
  double vy_gt;   // ground truth velocity y
  double rmse_x;  // RSME of x
  double rmse_y;  // RSME of y
  double rmse_vx; // RSME of vx
  double rmse_vy; // RSME of vy

  string toString() const
  {
    stringstream ss;
    ss << p_x
      << "\t" << p_y
      << "\t" << v1
      << "\t" << v2
      << "\t" << m_px
      << "\t" << m_py
      << "\t" << x_gt
      << "\t" << y_gt
      << "\t" << vx_gt
      << "\t" << vy_gt
      << "\t" << rmse_x
      << "\t" << rmse_y
      << "\t" << rmse_vx
      << "\t" << rmse_vy
      << endl;

    return ss.str();
  }
};


#endif
