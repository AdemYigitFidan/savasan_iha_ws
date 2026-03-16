## Sistem Mimarisi

Yazılım, **Behavior Tree (BT)** tabanlı bir karar verici etrafında inşa edilmiştir. BT, tüm görev akışını (güvenlik, kamikaze, savaşan) tek bir ağaç yapısında yönetir.

```
Root [Fallback]
├── Kurtarma (en yüksek öncelik)
│   ├── Haberleşme Kopukluğu → RTL + Çember
│   ├── Kırmızı Alan Kaçınma → Kaçınma Modu
│   └── Sınır / İrtifa İhlali → Güvenli Bölgeye Yönel
├── Kamikaze Görevi
│   ├── Otonom Kalkış
│   ├── Hedefe Yönel
│   ├── Dalış + QR Okuma
│   ├── Paket Gönder
│   └── Otonom İniş
└── Savaşan Görevi
    ├── Hedef Ara + Takip
    ├── Kilitlenme
    └── Paket Gönder
```

## Teknoloji Yığını

| Bileşen | Teknoloji |
|---------|-----------|
| Karar Verici | BehaviorTree.CPP v4 |
| Middleware | ROS2 Jazzy |
| Otonom Uçuş | ArduPilot SITL + MAVROS |
| Görüntü İşleme | YOLOv11 + OpenCV |
| Programlama Dili | C++ |
| BT Editör | Groot2 |

## Proje Yapısı

```
savasan_iha_ws/
└── src/
    └── savasan_bt/                          # Ana BT karar verici paketi
        ├── config/
        │   └── main_tree.xml                # BT ağacı (Groot2 ile düzenlenebilir)
        ├── include/savasan_bt/nodes/        # Node header dosyaları (.hpp)
        ├── src/
        │   ├── nodes/                       # Node implementasyonları (.cpp)
        │   └── main.cpp                     # BT entry point
        ├── CMakeLists.txt
        └── package.xml
```

## Kurulum

### Gereksinimler
- Ubuntu 24.04
- ROS2 Jazzy
- ArduPilot SITL
- MAVROS

### Derleme

```bash
# Workspace'i klonla
git clone https://github.com/AdemYigitFidan/savasan_iha_ws.git
cd savasan_iha_ws

# BehaviorTree.CPP bağımlılığını indir
cd src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd ..

# Derle
colcon build
source install/setup.bash
```

### Çalıştırma

```bash
# Terminal 1: ArduPilot SITL
sim_vehicle.py -v ArduPlane --map --console

# Terminal 2: MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@

# Terminal 3: BT Karar Verici
ros2 run savasan_bt savasan_bt_node
```

## Geliştirme Rehberi

### Yeni Node Ekleme

1. `include/savasan_bt/nodes/` altına `.hpp` dosyası oluştur (class tanımı)
2. `src/nodes/` altına `.cpp` dosyası oluştur (implementasyon)
3. `main.cpp`'de node'u `factory.registerNodeType<>()` ile kaydet
4. `CMakeLists.txt`'de `.cpp` dosyasını `add_executable`'a ekle
5. Gerekirse `config/main_tree.xml`'de ağaca ekle

### Harici Paket Entegrasyonu

YOLO, QR okuma gibi ağır işlemler ayrı ROS2 paketi olarak geliştirilir. BT node'ları bu paketlerin topic'lerini dinler:

```
savasan_iha_ws/src/
├── savasan_bt/          ← BT karar verici
├── yolo_detector/       ← YOLO paketi → /detected_targets topic
└── qr_reader/           ← QR paketi  → /qr_reader/result topic
```
