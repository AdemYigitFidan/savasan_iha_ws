#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

// === CONDITION NODE'LARI ===
#include "savasan_bt/nodes/haberlesme_koptu_mu.hpp"
#include "savasan_bt/nodes/kirmizi_alana_mi_girdin.hpp"
#include "savasan_bt/nodes/sinir_veya_irtifa_asildi_mi.hpp"
#include "savasan_bt/nodes/kamikaze_gorevi_aktif_mi.hpp"
#include "savasan_bt/nodes/yeterli_yukseklik_mi.hpp"
#include "savasan_bt/nodes/qr_konum_bilgisi_girildi_mi.hpp"
#include "savasan_bt/nodes/yukseklik_100m_veya_uzeri_mi.hpp"
#include "savasan_bt/nodes/qr_goruntude_mi.hpp"
#include "savasan_bt/nodes/hedefe_x_kadar_yaklastin_mi.hpp"
#include "savasan_bt/nodes/yukseklik_10m_veya_uzeri_mi.hpp"
#include "savasan_bt/nodes/savasan_gorevi_aktif_mi.hpp"

// === ACTION NODE'LARI ===
#include "savasan_bt/nodes/rtl_baslat_ve_cember_at.hpp"
#include "savasan_bt/nodes/kacinma_moduna_gec.hpp"
#include "savasan_bt/nodes/guvenli_bolgeye_yonel.hpp"
#include "savasan_bt/nodes/otonom_kalkis.hpp"
#include "savasan_bt/nodes/yuksekligi_10m_arttir.hpp"
#include "savasan_bt/nodes/gorev_secimi_moduna_gec.hpp"
#include "savasan_bt/nodes/konuma_ilerle.hpp"
#include "savasan_bt/nodes/yuz_100m_olana_kadar_tirman.hpp"
#include "savasan_bt/nodes/dalisa_gec.hpp"
#include "savasan_bt/nodes/qr_i_ortala.hpp"
#include "savasan_bt/nodes/kodu_tara_ve_kayit_al.hpp"
#include "savasan_bt/nodes/dalisi_sonlandir_guvenli_yukseklige_gik.hpp"
#include "savasan_bt/nodes/paketleri_sunucuya_ilet.hpp"
#include "savasan_bt/nodes/qr_konum_bilgisini_temizle.hpp"
#include "savasan_bt/nodes/dalisi_surdur.hpp"
#include "savasan_bt/nodes/pas_gec.hpp"
#include "savasan_bt/nodes/otonom_inis.hpp"
#include "savasan_bt/nodes/placeholder_savasan.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("savasan_bt_node");

    BT::BehaviorTreeFactory factory;

    // --- Condition Node'larını kaydet ---
    factory.registerNodeType<HaberlesmeKoptuMu>("HaberlesmeKoptuMu");
    factory.registerNodeType<KirmiziAlanaMiGirdin>("KirmiziAlanaMiGirdin");
    factory.registerNodeType<SinirVeyaIrtifaAsildiMi>("SinirVeyaIrtifaAsildiMi");
    factory.registerNodeType<KamikazeGoreviAktifMi>("KamikazeGoreviAktifMi");
    factory.registerNodeType<YeterliYukseklikMi>("YeterliYukseklikMi");
    factory.registerNodeType<QRKonumBilgisiGirildiMi>("QRKonumBilgisiGirildiMi");
    factory.registerNodeType<Yukseklik100mVeyaUzeriMi>("Yukseklik100mVeyaUzeriMi");
    factory.registerNodeType<QRGoruntudeMi>("QRGoruntudeMi");
    factory.registerNodeType<HedefeXKadarYaklastinMi>("HedefeXKadarYaklastinMi");
    factory.registerNodeType<Yukseklik10mVeyaUzeriMi>("Yukseklik10mVeyaUzeriMi");
    factory.registerNodeType<SavasanGoreviAktifMi>("SavasanGoreviAktifMi");

    // --- Action Node'larını kaydet ---
    factory.registerNodeType<RTLBaslatVeCemberAt>("RTLBaslatVeCemberAt");
    factory.registerNodeType<KacinmaModunaGec>("KacinmaModunaGec");
    factory.registerNodeType<GuvenliBolgeyeYonel>("GuvenliBolgeyeYonel");
    factory.registerNodeType<OtonomKalkis>("OtonomKalkis");
    factory.registerNodeType<Yuksekligi10mArttir>("Yuksekligi10mArttir");
    factory.registerNodeType<GorevSecimiModunaGec>("GorevSecimiModunaGec");
    factory.registerNodeType<KonumaIlerle>("KonumaIlerle");
    factory.registerNodeType<Yuz100mOlanaKadarTirman>("Yuz100mOlanaKadarTirman");
    factory.registerNodeType<DalisaGec>("DalisaGec");
    factory.registerNodeType<QRiOrtala>("QRiOrtala");
    factory.registerNodeType<KoduTaraVeKayitAl>("KoduTaraVeKayitAl");
    factory.registerNodeType<DalisiSonlandirGuvenliYuksekligeGik>("DalisiSonlandirGuvenliYuksekligeGik");
    factory.registerNodeType<PaketleriSunucuyaIlet>("PaketleriSunucuyaIlet");
    factory.registerNodeType<QRKonumBilgisiniTemizle>("QRKonumBilgisiniTemizle");
    factory.registerNodeType<DalisiSurdur>("DalisiSurdur");
    factory.registerNodeType<PasGec>("PasGec");
    factory.registerNodeType<OtonomInis>("OtonomInis");
    factory.registerNodeType<PlaceholderSavasan>("PlaceholderSavasan");

    // XML'den ağacı yükle
    auto tree = factory.createTreeFromFile(
        "/home/adem/savasan_iha_ws/src/savasan_bt/config/main_tree.xml");

    RCLCPP_INFO(node->get_logger(), "Behavior Tree yuklendi. Tick basliyor...");

    // 10 Hz döngü (100ms)
    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        tree.tickOnce();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
