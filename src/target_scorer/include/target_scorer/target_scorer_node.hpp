#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <target_scorer/msg/scored_target.hpp>
#include <target_scorer/msg/scored_target_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>
#include <vector>
#include <set>
#include <cmath>

struct DroneData {
    uint8_t takim_numarasi;
    double enlem;
    double boylam;
    double irtifa;
    double yonelme;
    double hiz;
    int kilitlenme;
};

struct HSSData {
    double enlem;
    double boylam;
    double yaricap;
};

class TargetScorerNode : public rclcpp::Node
{
public:
    TargetScorerNode();

private:
    void sunucuTelemetriCallback(const std_msgs::msg::String::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    double hesaplaMesafe(double lat1, double lon1, double lat2, double lon2);
    double hesaplaYonFarki(double our_heading, double target_heading);
    bool kirmiziAlandaMi(double lat, double lon);
    double puanHesapla(const DroneData& hedef);

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sunucu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

    // Publishers
    rclcpp::Publisher<target_scorer::msg::ScoredTarget>::SharedPtr best_target_pub_;
    rclcpp::Publisher<target_scorer::msg::ScoredTargetArray>::SharedPtr all_targets_pub_;

    // Bizim drone verileri
    double our_lat_, our_lon_, our_alt_;
    double our_heading_, our_speed_;
    bool gps_ready_;

    // HSS (Kırmızı alanlar)
    std::vector<HSSData> hss_list_;

    // Blacklist (son kilitlendiğimiz drone)
    uint8_t last_locked_id_;
};
