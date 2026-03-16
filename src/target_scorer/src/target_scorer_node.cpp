#include "target_scorer/target_scorer_node.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <iostream>

using json = nlohmann::json;

TargetScorerNode::TargetScorerNode()
    : Node("target_scorer_node"),
      our_lat_(0.0), our_lon_(0.0), our_alt_(0.0),
      our_heading_(0.0), our_speed_(0.0),
      gps_ready_(false), last_locked_id_(255)
{
    // Sunucu telemetrisini dinle (arkadaşlarının simülasyonu)
    sunucu_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/sunucu_telemetri", 10,
        std::bind(&TargetScorerNode::sunucuTelemetriCallback, this, std::placeholders::_1));

    // Bizim drone'un GPS konumu
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", 10,
        std::bind(&TargetScorerNode::gpsCallback, this, std::placeholders::_1));

    // Bizim drone'un hızı
    vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mavros/local_position/velocity_body", 10,
        std::bind(&TargetScorerNode::velCallback, this, std::placeholders::_1));

    // Publish: en iyi hedef + tüm hedefler
    best_target_pub_ = this->create_publisher<target_scorer::msg::ScoredTarget>("/best_target", 10);
    all_targets_pub_ = this->create_publisher<target_scorer::msg::ScoredTargetArray>("/all_targets", 10);

    RCLCPP_INFO(this->get_logger(), "Target Scorer baslatildi. /sunucu_telemetri dinleniyor...");
}

void TargetScorerNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    our_lat_ = msg->latitude;
    our_lon_ = msg->longitude;
    our_alt_ = msg->altitude;
    gps_ready_ = true;
}

void TargetScorerNode::velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    double vx = msg->twist.linear.x;
    double vy = msg->twist.linear.y;
    our_speed_ = std::sqrt(vx * vx + vy * vy);
    our_heading_ = std::atan2(vy, vx) * 180.0 / M_PI;
    if (our_heading_ < 0) our_heading_ += 360.0;
}

void TargetScorerNode::sunucuTelemetriCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!gps_ready_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GPS verisi henuz gelmedi, puanlama bekliyor...");
        return;
    }

    try {
        auto data = json::parse(msg->data);

        // HSS (kırmızı alanlar) güncelle
        hss_list_.clear();
        if (data.contains("hss_koordinat_bilgileri")) {
            for (auto& hss : data["hss_koordinat_bilgileri"]) {
                HSSData h;
                h.enlem = hss["hssEnlem"].get<double>();
                h.boylam = hss["hssBoylam"].get<double>();
                h.yaricap = hss["hssYaricap"].get<double>();
                hss_list_.push_back(h);
            }
        }

        // Drone'ları parse et
        std::vector<DroneData> drones;
        if (data.contains("konumBilgileri")) {
            for (auto& uav : data["konumBilgileri"]) {
                DroneData d;
                d.takim_numarasi = uav["takim_numarasi"].get<uint8_t>();
                d.enlem = uav["iha_enlem"].get<double>();
                d.boylam = uav["iha_boylam"].get<double>();
                d.irtifa = uav["iha_irtifa"].get<double>();
                d.yonelme = uav["iha_yonelme"].get<double>();
                d.hiz = uav["iha_hiz"].get<double>();
                d.kilitlenme = uav["iha_kilitlenme"].get<int>();
                drones.push_back(d);
            }
        }

        // Puanla ve filtrele
        std::vector<target_scorer::msg::ScoredTarget> scored_list;

        for (auto& drone : drones) {
            // === FİLTRELEME ===

            // 1. Kırmızı alanda mı? → direkt ele
            if (kirmiziAlandaMi(drone.enlem, drone.boylam)) {
                continue;
            }

            // 2. Son kilitlendiğimiz drone → direkt ele
            if (drone.takim_numarasi == last_locked_id_) {
                continue;
            }

            // 3. Yerdeki drone (irtifa < 5m) → ele
            if (drone.irtifa < 5.0) {
                continue;
            }

            // === PUANLAMA ===
            double puan = puanHesapla(drone);

            target_scorer::msg::ScoredTarget st;
            st.target_id = drone.takim_numarasi;
            st.latitude = drone.enlem;
            st.longitude = drone.boylam;
            st.altitude = drone.irtifa;
            st.score = puan;
            st.heading = drone.yonelme;
            st.speed = drone.hiz;
            st.is_locked_by_others = (drone.kilitlenme == 1);
            st.is_blacklisted = false;

            scored_list.push_back(st);
        }

        // Puana göre sırala (yüksek puan önce)
        std::sort(scored_list.begin(), scored_list.end(),
            [](const auto& a, const auto& b) { return a.score > b.score; });

        // Publish: tüm hedefler
        target_scorer::msg::ScoredTargetArray array_msg;
        array_msg.targets = scored_list;
        if (!scored_list.empty()) {
            array_msg.best_target = scored_list[0];
        }
        all_targets_pub_->publish(array_msg);

        // Publish: en iyi hedef
        if (!scored_list.empty()) {
            best_target_pub_->publish(scored_list[0]);
            RCLCPP_INFO(this->get_logger(),
                "En iyi hedef: Takim %d | Puan: %.1f | Mesafe: %.0fm | Irtifa: %.0fm",
                scored_list[0].target_id, scored_list[0].score,
                hesaplaMesafe(our_lat_, our_lon_, scored_list[0].latitude, scored_list[0].longitude),
                scored_list[0].altitude);
        } else {
            RCLCPP_WARN(this->get_logger(), "Uygun hedef bulunamadi!");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "JSON parse hatasi: %s", e.what());
    }
}

double TargetScorerNode::puanHesapla(const DroneData& hedef)
{
    double puan = 0.0;

    // 1. MESAFE PUANI (0-40 puan, yakın = yüksek)
    double mesafe = hesaplaMesafe(our_lat_, our_lon_, hedef.enlem, hedef.boylam);
    if (mesafe < 50.0) {
        puan += 40.0;
    } else if (mesafe < 200.0) {
        puan += 40.0 * (1.0 - (mesafe - 50.0) / 150.0);
    } else if (mesafe < 500.0) {
        puan += 20.0 * (1.0 - (mesafe - 200.0) / 300.0);
    }
    // 500m+ → 0 mesafe puanı

    // 2. İRTİFA FARKI PUANI (0-25 puan, benzer irtifa = yüksek)
    double irtifa_farki = std::abs(our_alt_ - hedef.irtifa);
    if (irtifa_farki < 10.0) {
        puan += 25.0;
    } else if (irtifa_farki < 30.0) {
        puan += 25.0 * (1.0 - (irtifa_farki - 10.0) / 20.0);
    } else if (irtifa_farki < 50.0) {
        puan += 10.0 * (1.0 - (irtifa_farki - 30.0) / 20.0);
    }

    // 3. HIZ VE YÖN PUANI (0-20 puan, bize doğru gelen = yüksek)
    double dy = (hedef.enlem - our_lat_) * 111320.0;
    double dx = (hedef.boylam - our_lon_) * 111320.0 * std::cos(our_lat_ * M_PI / 180.0);
    double bizden_hedefe_aci = std::atan2(dx, dy) * 180.0 / M_PI;
    if (bizden_hedefe_aci < 0) bizden_hedefe_aci += 360.0;

    // Hedefin yönelme açısı ile bizden hedefe olan açı arasındaki fark
    // Hedef bize doğru geliyorsa fark ~180 derece olur
    double yon_farki = std::abs(hedef.yonelme - bizden_hedefe_aci);
    if (yon_farki > 180.0) yon_farki = 360.0 - yon_farki;

    // 180'e yakınsa bize doğru geliyor
    double karsilikli_faktor = yon_farki / 180.0;  // 0=aynı yön, 1=karşılıklı
    puan += 20.0 * karsilikli_faktor;

    // 4. KİLİTLENME DURUMU (0-15 puan, kilitlenmede olmayan = yüksek)
    if (hedef.kilitlenme == 0) {
        puan += 15.0;  // kimse kilitlenmemiş, kolay hedef
    }
    // Birisi kilitlenmişse 0 bonus

    return puan;
}

bool TargetScorerNode::kirmiziAlandaMi(double lat, double lon)
{
    for (auto& hss : hss_list_) {
        double mesafe = hesaplaMesafe(lat, lon, hss.enlem, hss.boylam);
        if (mesafe < hss.yaricap) {
            return true;
        }
    }
    return false;
}

double TargetScorerNode::hesaplaMesafe(double lat1, double lon1, double lat2, double lon2)
{
    double dy = (lat2 - lat1) * 111320.0;
    double dx = (lon2 - lon1) * 111320.0 * std::cos(lat1 * M_PI / 180.0);
    return std::sqrt(dx * dx + dy * dy);
}

double TargetScorerNode::hesaplaYonFarki(double our_heading, double target_heading)
{
    double fark = std::abs(our_heading - target_heading);
    if (fark > 180.0) fark = 360.0 - fark;
    return fark;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetScorerNode>());
    rclcpp::shutdown();
    return 0;
}
