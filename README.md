
Otonom Sürü İHA &amp; QR Hassas İniş Sistemi
# 🦅 Teknofest Otonom Sürü İHA & QR Hassas İniş Sistemi (Swarm UAV Precision Landing)

Bu proje, Teknofest ve benzeri otonom İHA yarışmaları için geliştirilmiş, **ROS 2 ve PX4 SITL (Gazebo)** tabanlı gelişmiş bir sürü zekası ve görsel servo (visual servoing) otonomi yazılımıdır. Sistem, bir Gözcü (Lider) ve bir Vurucu (Yancı) İHA'nın haberleşerek ortak hedef tespiti yapmasını ve hedefe milimetrik hassasiyetle inmesini sağlar.

## 🚀 Temel Özellikler (Core Features)

* **Dağıtık Sürü Zekası (Distributed Swarm Intelligence):** İki İHA zıt yönlerde (Kuzey-Güney) otonom arama devriyesi atar. Alanı iki kat daha hızlı tararlar.
* **Görsel Servo & Dinamik Merkezleme:** Kamera çözünürlüğünden bağımsız olarak hedefi (QR Kod) tam merkeze alan dinamik piksel hesaplaması (`w//2, h//2`).
* **Handoff (Hedef Devri) Protokolü:** Lider hedefi bulduğunda merkezlenir, koordinatı kilitler ve 6 metre irtifada "Koruma/Gözlem" moduna geçer. Yancı İHA, o kilitli hedefe uçarak iniş sekansını başlatır.
* **PD Hız Kontrolü ile Sarsıntısız Uçuş (Velocity Control):** P-D (Proportional-Derivative) kontrolcüsü sayesinde İHA'lar "Dur-Kalk" yapmak yerine, hedefi merkezlerken kuğu gibi süzülür (`Velocity Setpoint`).
* **Body to NED Dönüşümü (Evrensel Trigonometri):** İHA'nın anlık pusula yönü (Yaw) baz alınarak, kameradaki "Sağ-Sol" komutları gerçek dünyadaki "Kuzey-Doğu" eksenlerine `math.sin` ve `math.cos` matrisleriyle dönüştürülür. İHA'nın ters yöne kaçması engellenmiştir.
* **Decoupled (Ayrıştırılmış) Asansör İnişi:** Yancı İHA, hedefi tam merkezlemeden irtifa kaybetmez. Sadece güvenli tolerans alanındayken asansör gibi dikine aşağı iner.
* **Dinamik Tolerans ve İniş Takımı Bypass'ı:** Yere yaklaştıkça büyüyen hedefin piksel alanı (`bw * bh`) hesaplanır. Tolerans dinamik olarak artar ve hedefin iniş takımları arkasında kalarak kamerayı kör etmesi durumu (Area > 120.000) bir hata değil, **"Başarılı İniş (Commit to Land)"** olarak değerlendirilerek motorlar otomatik kesilir.

## 🛠️ Kullanılan Teknolojiler

* **ROS 2** (Robot Operating System)
* **PX4 Autopilot & Gazebo Simulator**
* **OpenCV & cv_bridge** (Görüntü İşleme ve QR Tespiti)
* **Python 3**

## ⚙️ Kurulum ve Çalıştırma

**1. ROS 2 Çalışma Alanını Derleyin:**
```bash
cd ~/dds_ws
colcon build --packages-select px4_kontrol
source install/setup.bash

2. PX4 ve Gazebo Simülasyonunu Başlatın:
Terminal 1 (Lider):

Bash
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_mono_cam ./build/px4_sitl_default/bin/px4 -i 1
Terminal 2 (Yancı):

Bash
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_SIM_MODEL=gz_x500_mono_cam ./build/px4_sitl_default/bin/px4 -i 2
3. Görüntü Köprüsünü (ros_gz_bridge) Kurun:

Bash
ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_mono_cam_1/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
/world/default/model/x500_mono_cam_2/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
--ros-args \
-r /world/default/model/x500_mono_cam_1/link/camera_link/sensor/camera/image:=/px4_1/camera/image_raw \
-r /world/default/model/x500_mono_cam_2/link/camera_link/sensor/camera/image:=/px4_2/camera/image_raw
4. Otonomi Düğümünü Başlatın:

Bash
ros2 run px4_kontrol qr_suru_operasyonu
⚠️ Önemli Not
PX4 QGroundControl güvenlik kilitlerini devre dışı bırakmak için Lider ve Yancı terminallerinde (pxh>) şu komutların girilmesi tavsiye edilir:

Bash
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0
