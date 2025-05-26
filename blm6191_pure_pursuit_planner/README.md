

**Adım 1: Çalışma Alanını Build Etme**

Paketlerinizi çalışma alanınızın `src` klasörüne yerleştirdikten sonra, çalışma alanınızı derlemeniz gerekir.

1.  Mevcut tüm ROS düğümlerini ve süreçlerini durdurun (güvenlik için):
    ```bash
    rosnode kill -a
    pkill -f roslaunch
    pkill -f rosmaster
    pkill -f gzserver
    pkill -f gzclient
    pkill -f rviz
    ```
2.  Çalışma alanınızın kök dizinine gidin (`~/robotlar_ws`):
    ```bash
    cd ~/robotlar_ws
    ```
3.  Çalışma alanınızı derleyin:
    ```bash
    catkin_make
    ```
    Bu komut, hem `blm6191_coverage_planner` hem de `blm6191_pure_pursuit_planner` paketlerindeki kodları derleyip kütüphaneleri oluşturacaktır. Derleme sırasında hata olup olmadığını kontrol edin. Hata alırsanız, kodda veya CMakeLists.txt/package.xml dosyalarındaki bağımlılıklarda bir sorun olabilir.
4.  Derlenen paketleri geçerli ROS ortamınıza eklemek için çalışma alanınızı kaynak gösterin:
    ```bash
    source devel/setup.bash
    ```
    **Önemli:** Bundan sonra açacağınız *her yeni terminalde* bu `source devel/setup.bash` komutunu çalıştırmanız gerekecektir.

**Adım 2: ROS Ortamını Başlatma (Gazebo, SLAM, Move Base + Pluginler)**

Şimdi simülasyon ve navigasyon stack'ini başlatma zamanı. Farklı bileşenleri farklı terminallerde çalıştırmak sorunları takip etmeyi kolaylaştırır.

1.  **Terminal 1: Gazebo Simülasyonunu Başlatma**
    Yeni bir terminal açın, çalışma alanınızı kaynak gösterin ve Gazebo'yu başlatın:
    ```bash
    source ~/robotlar_ws/devel/setup.bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```
    Gazebo simülasyon penceresi açılmalı ve TurtleBot3 robotunu görmelisiniz.

2.  **Terminal 2: SLAM Düğümünü Başlatma**
    Yeni bir terminal açın, çalışma alanınızı kaynak gösterin ve SLAM düğümünü başlatın:
    ```bash
    source ~/robotlar_ws/devel/setup.bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch
    ```
    RViz penceresi açılmalı (SLAM launch dosyasında `open_rviz:=true` argümanı varsa) ve robotun etrafında bir costmap (harita) oluşmaya başladığını görmelisiniz. Robotu klavye teleop (veya joystick) ile gezdirerek kapsama planlaması yapmak istediğiniz alanı haritalandırın. Haritanın yeterince büyük olduğundan ve poligonu kapsadığından emin olun.

3.  **Terminal 3: Move Base Düğümünü Başlatma (Plugin'lerimizle)**
    Yeni bir terminal açın, çalışma alanınızı kaynak gösterin ve özel launch dosyanızı kullanarak `move_base`'i başlatın:
    ```bash
    source ~/robotlar_ws/devel/setup.bash
    roslaunch blm6191_coverage_planner run_navigation.launch
    ```
    Bu launch dosyası, `turtlebot3_navigation`'ın standart `move_base.launch` dosyasını dahil eder ancak konfigürasyon dosyalarımızı (Global ve Yerel Planlayıcı plugin isimleri ve parametreleri dahil) yükler. Bu terminalde `move_base` ve planlayıcı plugin'lerinden gelen log mesajlarını göreceksiniz. Plugin'lerin başarıyla yüklendiğine dair mesajları arayın (`... initialized.`).

**Adım 3: Kapsama Alanı Poligonunu Yayınlama**

Global planlayıcının bir kapsama alanı belirlemesi için poligonu yayınlamalısınız.

1.  **Terminal 4: Poligonu Yayınlama**
    Yeni bir terminal açın, çalışma alanınızı kaynak gösterin ve 4 köşeli poligon mesajını `/coverage_polygon` topic'ine yayınlayın. **Poligon koordinatlarının oluşturduğunuz harita (`map` frame'inde) içinde ve planlamak istediğiniz alanda olduğundan emin olun!**
    ```bash
    source ~/robotlar_ws/devel/setup.bash
    rostopic pub /coverage_polygon geometry_msgs/Polygon "points:
    - {x: 1.0, y: 1.0, z: 0.0}
    - {x: 3.0, y: 1.0, z: 0.0}
    - {x: 3.0, y: 3.0, z: 0.0}
    - {x: 1.0, y: 3.0, z: 0.0}" --once
    ```
    `move_base`'in çalıştığı Terminal 3'te, global planlayıcının poligonu aldığına dair bir log mesajı görmelisiniz.

**Adım 4: Yol Planlamayı ve Takibini Başlatma (RViz)**

Şimdi planlama ve takibi tetikleme zamanı.

1.  **RViz Penceresi:** SLAM veya `run_navigation.launch` ile açılan RViz penceresine odaklanın.
2.  **Global Yol Görselleştirmesini Ekleme:** Eğer global plan yolu otomatik görünmüyorsa, RViz'de "Displays" panelindeki "Add" butonuna tıklayın. "By Topic" sekmesinde `/move_base/blm6191_coverage_planner/global_coverage_path` topic'ini bularak ekleyin.
3.  **Planlamayı Tetikleme:** RViz'in üst araç çubuğunda "2D Nav Goal" butonuna tıklayın. Harita üzerinde, yayınladığınız poligon alanının içinde veya yakınında bir hedef noktası çizin (fare ile sürükleyerek yön de verebilirsiniz, ancak kapsama planlayıcısı yönü kullanmaz, sadece hedef komutunu tetiklemek için kullanılır).
4.  `move_base`, 2D Nav Goal komutunu alınca global planlayıcıyı (`CoveragePlanner`) çağıracaktır. Global planlayıcı kapsama yolunu hesaplayıp `/move_base/blm6191_coverage_planner/global_coverage_path` topic'inde yayınlayacak (RViz'de görmelisiniz).
5.  Ardından `move_base`, bu global planı yerel planlayıcıya (`PurePursuitPlanner`) verecek (`setPlan` metodu çağrılır).
6.  Yerel planlayıcı `computeVelocityCommands` metodunu çağırarak yolu takip etmek için hız komutları hesaplamaya ve yayınlamaya başlayacaktır (`/cmd_vel`).
7.  Gazebo'daki robotun hesaplanan kapsama yolunu takip etmeye başladığını görmelisiniz. RViz'de robotun pozisyonunu, haritayı ve global kapsama yolunu izleyebilirsiniz.

**Önemli İpuçları:**

*   **Logları Kontrol Edin:** Terminal 3'teki `move_base` çıktılarını dikkatle izleyin. Hatalar, uyarılar veya planlayıcılarınızdan gelen bilgilendirme mesajları sorun gidermede çok yardımcı olur.
*   **Parametreleri Ayarlayın:** `blm6191_coverage_planner/config/pure_pursuit_planner_params.yaml` dosyasındaki `lookahead_distance` ve `max_linear_velocity` gibi parametreler Pure Pursuit'un takibini doğrudan etkiler. Robotun titrek mi gittiğini, yolu düzgün takip edip etmediğini gözlemleyerek bu parametrelerle oynayabilirsiniz. Daha büyük `lookahead_distance` genellikle daha yumuşak dönüşler sağlar ama köşe dönmelerde zorlanabilir.
*   **Poligon Koordinatları:** Yayınladığınız poligon koordinatlarının doğru frame'de (genellikle `map`) ve haritada kapsama yapmak istediğiniz alanı tanımladığından emin olun.
