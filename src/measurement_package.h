#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

  Eigen::VectorXd currentMeasurement()
  {
    switch (sensor_type_)
    {
    case LASER:
    {
      auto measurements = Eigen::VectorXd(2);
      measurements << raw_measurements_[0], raw_measurements_[1];
      return measurements;
    }
    case RADAR:
    {
      auto px = cos(raw_measurements_[1]) * raw_measurements_[0];
      auto py = sin(raw_measurements_[1]) * raw_measurements_[0];
      auto measurements = Eigen::VectorXd(2);
      measurements << px, py;
      return measurements;
    }
    }

    throw std::exception();
  }

};

#endif /* MEASUREMENT_PACKAGE_H_ */
