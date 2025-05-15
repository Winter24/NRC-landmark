#include <memory>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

struct Landmark {
  double x, y;
};

class ReflectorLocalizer : public rclcpp::Node {
public:
  ReflectorLocalizer()
  : Node("reflector_localizer")
  {
    // params
    declare_parameter<std::string>("landmark_map", "map.txt");
    declare_parameter<double>("assoc_max_dist", 0.3);

    get_parameter("landmark_map", map_file_);
    get_parameter("assoc_max_dist", max_assoc_dist_);

    loadMap(map_file_);

    sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/reflector_poses", 10,
      std::bind(&ReflectorLocalizer::onReflectors, this, std::placeholders::_1));

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/localized_pose", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void loadMap(const std::string &f) {
    std::ifstream in(f);
    if(!in) { RCLCPP_ERROR(get_logger(), "Cannot open map file"); return; }
    std::string line;
    while(std::getline(in, line)) {
      std::istringstream ss(line);
      Landmark lm;
      ss >> lm.x >> lm.y;
      map_.push_back(lm);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu landmarks", map_.size());
  }

  void onReflectors(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // build pairs
    std::vector<std::pair<double,double>> D, L;
    for(auto &p : msg->poses) {
      double dx = p.position.x, dy = p.position.y;
      // find nearest map landmark
      double best = 1e9; int bi=-1;
      for(size_t i=0;i<map_.size();i++){
        double mx = map_[i].x - dx;  
        double my = map_[i].y - dy;
        double d2 = mx*mx + my*my;
        if(d2 < best && d2 < max_assoc_dist_*max_assoc_dist_){
          best = d2; bi = i;
        }
      }
      if(bi>=0) {
        D.emplace_back(dx,dy);
        L.emplace_back(map_[bi].x, map_[bi].y);
      }
    }
    if(D.size() < 2) {
      RCLCPP_WARN(get_logger(), "Too few associations: %zu", D.size());
      return;
    }

    // compute centroids
    double dx=0, dy=0, lx=0, ly=0;
    int M = D.size();
    for(int i=0;i<M;i++){
      dx += D[i].first;  dy += D[i].second;
      lx += L[i].first;  ly += L[i].second;
    }
    dx/=M; dy/=M; lx/=M; ly/=M;

    // compute Sx, Sy
    double Sx=0, Sy=0;
    for(int i=0;i<M;i++){
      double ux = D[i].first  - dx;
      double uy = D[i].second - dy;
      double vx = L[i].first  - lx;
      double vy = L[i].second - ly;
      Sx += ux*vx + uy*vy;
      Sy += ux*vy - uy*vx;
    }

    double theta = std::atan2(Sy, Sx);
    double cos_t = std::cos(theta), sin_t = std::sin(theta);

    // translation
    double tx = lx - ( cos_t*dx - sin_t*dy );
    double ty = ly - ( sin_t*dx + cos_t*dy );

    // publish PoseStamped
    geometry_msgs::msg::PoseStamped out;
    out.header = msg->header;
    out.header.frame_id = "map";
    out.pose.position.x = tx;
    out.pose.position.y = ty;
    tf2::Quaternion q;
    q.setRPY(0,0,theta);
    out.pose.orientation = tf2::toMsg(q);
    pub_->publish(out);

    // broadcast tf
    geometry_msgs::msg::TransformStamped t;
    t.header = out.header;
    t.child_frame_id = "base_link";
    t.transform.translation.x = tx;
    t.transform.translation.y = ty;
    t.transform.rotation = out.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  std::string map_file_;
  double      max_assoc_dist_;
  std::vector<Landmark> map_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReflectorLocalizer>());
  rclcpp::shutdown();
  return 0;
}
