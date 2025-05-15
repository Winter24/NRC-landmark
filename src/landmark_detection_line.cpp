#include <memory>
#include <vector>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using std::placeholders::_1;

// Gaussian‐fit estimate peak index (O(K))
static double estimate_peak_index(const std::vector<double>& I) {
  int K = (int)I.size();
  std::vector<double> y(K);
  for (int i = 0; i < K; ++i)
    y[i] = I[i] > 1e-3 ? std::log(I[i]) : std::log(1e-3);

  double S0=0, S1=0, S2=0, S3=0, S4=0;
  for (int i = 0; i < K; ++i) {
    double ii = i, yi = y[i];
    S0 += yi; S1 += ii*yi; S2 += ii*ii*yi;
    S3 += ii*ii*ii*yi; S4 += ii*ii*ii*ii*yi;
  }

  // build augmented matrix
  double M[3][4] = {
    { S4, S3, S2,  S2 },
    { S3, S2, S1,  S1 },
    { S2, S1,  K,  S0 }
  };
  // Gaussian elimination
  for (int i = 0; i < 3; ++i) {
    double piv = M[i][i];
    if (std::fabs(piv) < 1e-8) continue;
    for (int j = i; j < 4; ++j) M[i][j] /= piv;
    for (int k = i+1; k < 3; ++k) {
      double f = M[k][i];
      for (int j = i; j < 4; ++j) M[k][j] -= f * M[i][j];
    }
  }
  double sol[3];
  for (int i = 2; i >= 0; --i) {
    double v = M[i][3];
    for (int j = i+1; j < 3; ++j) v -= M[i][j] * sol[j];
    sol[i] = (std::fabs(M[i][i])>1e-8) ? v / M[i][i] : 0.0;
  }
  double A = sol[0], B = sol[1];
  if (A >= 0) return K/2.0;
  double idx = -B / (2*A);
  return std::clamp(idx, 0.0, double(K-1));
}

class ReflectorDetector : public rclcpp::Node {
public:
  ReflectorDetector()
  : Node("landmark_detection")
  {
    // Parameters
    declare_parameter<double>("min_intensity",        50.0);
    declare_parameter<double>("angle_cluster_thresh", 0.05);
    declare_parameter<double>("range_min",             0.2);
    declare_parameter<double>("range_max",             5.0);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ReflectorDetector::scanCb, this, _1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/reflector_poses", 10);
  }

private:
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Read params
    double mint = get_parameter("min_intensity").as_double();
    double ath  = get_parameter("angle_cluster_thresh").as_double();
    double rmin = get_parameter("range_min").as_double();
    double rmax = get_parameter("range_max").as_double();

    // 1) Radius + intensity filter
    std::vector<std::pair<double,double>> pts;
    std::vector<double> intens, angles;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i], I = msg->intensities[i];
      if (r>=rmin && r<=rmax && I>=mint) {
        double th = msg->angle_min + i * msg->angle_increment;
        pts.emplace_back(r*std::cos(th), r*std::sin(th));
        intens.push_back(I);
        angles.push_back(th);
      }
    }
    if (pts.empty()) return;

    // 2) Cluster by angle
    std::vector<std::vector<int>> clusters;
    std::vector<int> cur;
    for (size_t i = 0; i < angles.size(); ++i) {
      if (cur.empty() ||
          std::fabs(angles[i] - angles[i-1]) < ath)
      {
        cur.push_back(i);
      } else {
        clusters.push_back(cur);
        cur.clear();
        cur.push_back(i);
      }
    }
    if (!cur.empty()) clusters.push_back(cur);

    // 3) Build PoseArray
    geometry_msgs::msg::PoseArray pa;
    pa.header = msg->header;

    for (auto &c : clusters) {
      int K = c.size();
      if (K < 3) continue;

      // Build vectors
      std::vector<double> Is(K), Ths(K);
      std::vector<std::pair<double,double>> Cs(K);
      for (int i = 0; i < K; ++i) {
        int idx = c[i];
        Is[i]  = intens[idx];
        Ths[i] = angles[idx];
        Cs[i]  = pts[idx];
      }

      // 4) Gaussian-fit on log(I)
      double b = estimate_peak_index(Is);
      int ib = std::clamp(int(std::round(b)), 0, K-1);

      // Center estimate
      double x0 = Cs[ib].first;
      double y0 = Cs[ib].second;

      geometry_msgs::msg::Pose p;
      p.position.x = x0;
      p.position.y = y0;
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      pa.poses.push_back(p);
    }

    // 5) Publish PoseArray
    pose_pub_->publish(pa);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReflectorDetector>());
  rclcpp::shutdown();
  return 0;
}
