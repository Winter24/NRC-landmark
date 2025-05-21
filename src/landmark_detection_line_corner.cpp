#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

struct Point2D {
  double x, y;
};

// Euclid distance
double pointDist(const Point2D &p1, const Point2D &p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

// distance from point p to d(a,b)
double pointToLineDistance(const Point2D &p, const Point2D &a, const Point2D &b) {
  double num = std::abs((b.y - a.y)*p.x - (b.x - a.x)*p.y + b.x*a.y - b.y*a.x);
  double den = std::hypot(b.x - a.x, b.y - a.y);
  return den > 1e-6 ? num/den : std::numeric_limits<double>::max();
}

// intersect (a1,b1) & (a2,b2)
Point2D intersect(const Point2D &a1, const Point2D &b1,
                  const Point2D &a2, const Point2D &b2) {
  double A1 = b1.y - a1.y, B1 = a1.x - b1.x, C1 = A1*a1.x + B1*a1.y;
  double A2 = b2.y - a2.y, B2 = a2.x - b2.x, C2 = A2*a2.x + B2*a2.y;
  double det = A1*B2 - A2*B1;
  if (std::abs(det) < 1e-6) {
    return { (a1.x + b2.x)*0.5, (a1.y + b2.y)*0.5 };
  }
  return { (B2*C1 - B1*C2)/det, (A1*C2 - A2*C1)/det };
}

class LandmarkDetector : public rclcpp::Node {
public:
  LandmarkDetector()
  : Node("landmark_detector")
  {
    // parameter
    declare_parameter("min_intensity",       200.0);
    declare_parameter("range_min",           0.2);
    declare_parameter("range_max",           20.0);
    declare_parameter("angle_cluster_thresh",0.01);
    declare_parameter("dist_cluster_thresh", 0.05);

    declare_parameter("ransac_thresh",       0.01);
    declare_parameter("min_cluster_size",    3);
    declare_parameter("line_ransac_iters",   100);

    declare_parameter("corner_angle_tol",    0.5);
    declare_parameter("max_cluster_centroid_dist", 2.0);

    scan_sub_   = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LandmarkDetector::scan_callback, this, _1));

    line_pub_   = create_publisher<geometry_msgs::msg::PoseArray>("/line_landmarks", 10);
    corner_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/corner_landmarks", 10);
    vis_pub_    = create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_markers", 10);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double min_int   = get_parameter("min_intensity").as_double();
    double r_min     = get_parameter("range_min").as_double();
    double r_max     = get_parameter("range_max").as_double();
    double ang_th    = get_parameter("angle_cluster_thresh").as_double();
    double dist_th   = get_parameter("dist_cluster_thresh").as_double();
    double ransac_th = get_parameter("ransac_thresh").as_double();
    int    min_size  = get_parameter("min_cluster_size").as_int();
    int    line_iters= get_parameter("line_ransac_iters").as_int();
    double corner_ang_tol     = get_parameter("corner_angle_tol").as_double();
    double max_centroid_dist  = get_parameter("max_cluster_centroid_dist").as_double();

    // Filtering point
    std::vector<Point2D> pts;
    std::vector<double>  angs;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      double I = msg->intensities[i];
      if (!std::isfinite(r) || r<r_min || r>r_max || I<min_int) continue;
      double a = msg->angle_min + i*msg->angle_increment;
      pts.push_back({r*std::cos(a), r*std::sin(a)});
      angs.push_back(a);
    }

    // Clustering
    std::vector<std::vector<Point2D>> clusters;
    std::vector<Point2D> curr;
    for (size_t i = 0; i < pts.size(); ++i) {
      if (i>0 &&
          std::abs(angs[i]-angs[i-1])<ang_th &&
          pointDist(pts[i],pts[i-1])<dist_th)
      {
        curr.push_back(pts[i]);
      } else {
        if (curr.size() >= (size_t)min_size) clusters.push_back(curr);
        curr.clear(); curr.push_back(pts[i]);
      }
    }
    if (curr.size() >= (size_t)min_size) clusters.push_back(curr);

    // Visualize clusters
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.header = msg->header; del.ns="clusters"; del.id=0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    int mid = 1;
    for (auto &c : clusters) {
      visualization_msgs::msg::Marker m;
      m.header=msg->header; m.ns="clusters"; m.id=mid++;
      m.type = visualization_msgs::msg::Marker::POINTS;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x=m.scale.y=0.05;
      m.color.a=1.0; m.color.g=1.0;
      for (auto &p : c) {
        geometry_msgs::msg::Point gp; gp.x=p.x; gp.y=p.y; gp.z=0;
        m.points.push_back(gp);
      }
      ma.markers.push_back(m);
    }
    vis_pub_->publish(ma);

    // RANSAC detect lines
    struct Line { Point2D a,b; double cx,cy; int cluster_id; };
    std::vector<Line> lines;
    std::mt19937 rng{std::random_device{}()};

    for (int ci = 0; ci < (int)clusters.size(); ++ci) {
      auto c = clusters[ci];
      while ((int)c.size() >= min_size) {
        size_t N = c.size(), best_cnt=0;
        Point2D best_a, best_b;
        std::uniform_int_distribution<size_t> idist(0, N-1);

        for (int it=0; it<line_iters; ++it) {
          auto &p1 = c[idist(rng)], &p2 = c[idist(rng)];
          if (pointDist(p1,p2)<1e-3) continue;
          size_t cnt=0;
          for (auto &p : c)
            if (pointToLineDistance(p,p1,p2) < ransac_th) ++cnt;
          if (cnt>best_cnt) {
            best_cnt=cnt; best_a=p1; best_b=p2;
          }
        }

        double ratio = double(best_cnt)/double(N);
        if (ratio<0.5 || best_cnt<(size_t)min_size) break;

        double sx=0, sy=0;
        for (auto &p : c) { sx+=p.x; sy+=p.y; }
        lines.push_back({best_a,best_b,sx/N,sy/N,ci});

        std::vector<Point2D> next;
        for (auto &p : c)
          if (pointToLineDistance(p,best_a,best_b) >= ransac_th)
            next.push_back(p);
        c.swap(next);
      }
    }

    // Corner detection in cluster
    std::vector<bool> clusterHasCorner(clusters.size(), false);
    geometry_msgs::msg::PoseArray corner_msg;
    corner_msg.header = msg->header;

    for (size_t i=0; i<lines.size(); ++i) {
      for (size_t j=i+1; j<lines.size(); ++j) {
        // consider 2 lines in 1 cluster
        if (lines[i].cluster_id != lines[j].cluster_id)
          continue;

        // calculate angle between 2 lines
        Point2D v1{lines[i].b.x-lines[i].a.x, lines[i].b.y-lines[i].a.y};
        Point2D v2{lines[j].b.x-lines[j].a.x, lines[j].b.y-lines[j].a.y};
        double dot = v1.x*v2.x + v1.y*v2.y;
        double norm = std::hypot(v1.x,v1.y)*std::hypot(v2.x,v2.y);
        double cosv = std::clamp(dot/norm, -1.0, 1.0);
        double ang = std::acos(cosv);
        if (std::abs(ang - M_PI/2) > corner_ang_tol)
          continue;

        // calculate intersect
        Point2D ip = intersect(lines[i].a,lines[i].b, lines[j].a,lines[j].b);

        // mark cluster is a corner
        int cid = lines[i].cluster_id;
        clusterHasCorner[cid] = true;

        geometry_msgs::msg::Pose pc;
        pc.position.x = ip.x;
        pc.position.y = ip.y;
        pc.orientation.w = 1.0;
        corner_msg.poses.push_back(pc);
      }
    }
    // publishing
    corner_pub_->publish(corner_msg);

    geometry_msgs::msg::PoseArray line_msg;
    line_msg.header = msg->header;
    for (auto &L : lines) {
      if (clusterHasCorner[L.cluster_id]) continue;
      geometry_msgs::msg::Pose ps;
      ps.position.x = (L.a.x + L.b.x)*0.5;
      ps.position.y = (L.a.y + L.b.y)*0.5;
      ps.orientation.w = 1.0;
      line_msg.poses.push_back(ps);
    }
    line_pub_->publish(line_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr line_pub_, corner_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkDetector>());
  rclcpp::shutdown();
  return 0;
}
