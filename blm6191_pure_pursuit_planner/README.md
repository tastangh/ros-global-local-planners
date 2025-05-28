# BLM6191 Robotlar - Ödev 4 / Soru 2: Pure Pursuit Yerel Planlayıcısı

Bu ROS paketi, BLM6191 Robotlar dersi Ödev 4'ün ikinci sorusunu çözmek amacıyla geliştirilmiş bir **yerel planlayıcı eklentisidir**. ROS navigasyon yığınının (`move_base`) bir parçası olarak çalışır ve global planlayıcı tarafından üretilen bir yolu (`nav_msgs::Path`) takip etmek için **Pure Pursuit** algoritmasını kullanır.

Bu planlayıcı, genellikle Soru 1'deki kapsama alanı global planlayıcısı gibi bir global planlayıcının ürettiği yolu takip etmek için kullanılır. `move_base`, global planlayıcının ürettiği yolu bu yerel planlayıcıya iletir ve yerel planlayıcı da robotun bu yolu takip etmesi için gerekli hız komutlarını hesaplar.

## Özellikler

*   Global planlayıcıdan alınan bir yolu (`nav_msgs::Path`) takip etmek için `nav_core::BaseLocalPlanner` arayüzünü uygular.
*   Yol takibi için **Pure Pursuit** algoritmasını kullanır.
*   Robotun o anki pozisyonunu kullanarak (`costmap_2d::Costmap2DROS::getRobotPose`) lookahead noktasını belirler ve eğriliği hesaplar.
*   Belirlenen lookahead noktasına doğru hareket etmek için doğrusal (`linear.x`) ve açısal (`angular.z`) hız komutlarını (`geometry_msgs::Twist`) hesaplar.
*   Pure Pursuit parametrelerini (lookahead mesafesi, maksimum hızlar, hedef toleransı vb.) konfigürasyon dosyası aracılığıyla alır.
*   Global planın sonuna yaklaşıldığında hedefi başarıyla tamamlar (`isGoalReached`).

## Çözüm Yaklaşımı (Pure Pursuit Algoritması)

Pure Pursuit, basit ve etkili bir yol takip algoritmasıdır. Temel fikir, robotun doğrudan global plan üzerindeki bir noktaya doğru dümen kırmasıdır. Bu nokta, robotun o anki pozisyonundan belirli bir `lookahead_distance` (ileriye bakış mesafesi) kadar ileride, global plan üzerinde bulunan noktadır.

Algoritma her kontrol döngüsünde (örneğin, `computeVelocityCommands` çağrıldığında) şu adımları izler:

1.  **Robot Pozisyonunu Al:** Robotun harita (global) koordinat sistemindeki güncel pozisyonunu ve oryantasyonunu al.
2.  **Lookahead Noktasını Bul:** Global plan üzerinde ilerle. Robottan `lookahead_distance` kadar uzakta olan (veya bu mesafeye en yakın) ilk noktayı bul. Bu nokta, takip edilecek hedef noktasıdır. Bu nokta genellikle global plandaki segmentlerle `lookahead_distance` yarıçaplı bir çemberin kesişimi olarak hesaplanır. Robotun kendi koordinat sisteminde (örneğin `base_link` frame) bu noktanın konumunu bul.
3.  **Eğriliği Hesapla:** Robotun kendi frame'inde bulunan lookahead noktasına ulaşmak için robotun izlemesi gereken dairesel yayın eğriliğini hesapla. Differential drive robotlar için bu eğrilik genellikle `2 * y / Ld^2` formülü ile bulunur, burada `y` lookahead noktasının robot frame'indeki Y koordinatı ve `Ld` lookahead mesafesidir.
4.  **Hız Komutlarını Hesapla:** Robotun ileri hızını sabit bir `max_linear_velocity` değerine ayarla. Açısal hızı ise hesaplanan eğrilik ve ileri hızın çarpımı olarak belirle (`angular_velocity = linear_velocity * curvature`). Maksimum açısal hız (`max_angular_velocity`) limitini uygula.
5.  **Hedefe Ulaşma Kontrolü:** Robotun global planın son noktasına olan mesafesini kontrol et. Eğer bu mesafe `goal_tolerance` değerinden küçükse, hedefe ulaşıldığı kabul edilir ve robot durdurulur (hız komutları sıfır olarak ayarlanır).

`lookahead_distance` parametresi Pure Pursuit davranışını büyük ölçüde etkiler:
*   Küçük `lookahead_distance`: Robot yolu daha yakından takip eder, dönüşler daha keskin olur, ancak titreme (oscillation) veya kararsızlık riski artabilir.
*   Büyük `lookahead_distance`: Robot yolu daha yumuşak takip eder, büyük kavisler çizer, ancak engellerden kaçınma veya keskin dönüşler yapma yeteneği azalır.

## Bağımlılıklar

Bu paketin çalışabilmesi için aşağıdaki ROS paketlerine ihtiyacı vardır:

*   `base_local_planner` (Temel yerel planlayıcı arayüzü)
*   `costmap_2d` (Robot pozisyonunu almak için)
*   `geometry_msgs` (Pose, Twist mesajları için)
*   `nav_core` (ROS navigasyon arayüzleri için)
*   `nav_msgs` (Path mesajları için)
*   `pluginlib` (ROS eklenti mekanizması için)
*   `roscpp` (Temel ROS C++ arayüzü)
*   `tf` (Dönüşümler için - tf2 tercih edildi)
*   `tf2_ros` (Modern TF dönüşüm kütüphanesi)
*   `tf2_geometry_msgs` (geometry_msgs ile TF2 dönüşümü için)

Bu bağımlılıkları çalışma alanınızda `rosdep install --from-paths <your_ws_src_path> --ignore-src -r -y` komutu ile kurabilirsiniz.

## Kurulum ve Derleme

1.  ROS çalışma alanınızın (`robotlar_ws` gibi) `src` klasörüne gidin:
    ```bash
    cd ~/robotlar_ws/src
    ```
2.  Bu paketi Gitlab'dan klonlayın. 
    ```bash
    git clone https://gitlab.com/blm6191_2425b_tai/members/24501111/blm6191_pure_pursuit_planner.git
    ```
    *Not: Eğer global planlayıcı paketini aynı `src` klasörüne klonladıysanız, bu paketi de yanına klonlayın.*
3.  Çalışma alanınızın kök dizinine dönün:
    ```bash
    cd ~/robotlar_ws
    ```
4.  Paketi derleyin ve ortamınızı kaynak olarak belirtin:
    ```bash
    catkin_make
    source devel/setup.bash
    ```
    *Not: `.bashrc` dosyanıza `source ~/robotlar_ws/devel/setup.bash` satırını eklerseniz her terminal açılışında tekrar kaynak belirtmeniz gerekmez.*

## Konfigürasyon

Pure Pursuit planlayıcısının davranışını `config` klasöründeki `pure_pursuit_planner_params.yaml` dosyası aracılığıyla ayarlayabilirsiniz:

*   **`lookahead_distance`**: Robotun global plan üzerinde ilerisine baktığı mesafe (metre). Pure Pursuit'un en önemli parametresidir.
*   **`max_linear_velocity`**: Robotun takip sırasında ulaşmaya çalışacağı sabit ileri hız (m/s).
*   **`max_angular_velocity`**: Robotun ulaşabileceği maksimum açısal hız (rad/s). Hesaplanan açısal hız bu limiti aşarsa kırpılır.
*   **`goal_tolerance`**: Robotun global planın son noktasına ne kadar yaklaştığında hedefe ulaşılmış sayılacağını belirleyen mesafe (metre).
*   **`transform_tolerance`**: TF dönüşümleri sırasında beklenecek süre (saniye).

Bu parametrelerin `move_base` düğümüne yüklenmesi gerekir. Genellikle bu, `move_base` launch dosyasında `rosparam` etiketi kullanılarak yapılır (aşağıdaki Kullanım Adımları bölümüne bakın).

Ayrıca, `move_base`'in bu Pure Pursuit planlayıcısını yerel planlayıcı olarak kullanması için `move_base_config.yaml` dosyasını düzenlemeniz gerekir. Dosyanıza şu satırı eklediğinizden veya güncellediğinizden emin olun:

```yaml
move_base:
  base_local_planner: blm6191_pure_pursuit_planner/PurePursuitPlanner

  # Diğer move_base konfigürasyonları...
  # Global planlayıcı konfigürasyonu (Soru 1'den)
  base_global_planner: blm6191_coverage_planner/CoveragePlanner

  # Global planlayıcı parametreleri (Soru 1'den)
  blm6191_coverage_planner:
    robot_radius: 0.30
    polygon_topic: "coverage_polygon"

  # Yerel planlayıcı parametreleri (Soru 2 için bu)
  blm6191_pure_pursuit_planner:
    lookahead_distance: 0.4
    max_linear_velocity: 0.4
    max_angular_velocity: 1.5
    goal_tolerance: 0.1
    transform_tolerance: 0.1
```

## Kullanım Adımları

Bu yerel planlayıcı, Soru 1'deki global planlayıcı ile birlikte çalışarak robotun kapsama yolunu takip etmesini sağlar.

1.  **Önceki ROS Düğümlerini Kapatma (İsteğe Bağlı ama Önerilir):** Herhangi bir çakışmayı önlemek için çalışan ROS düğümlerini kapatın.
    ```bash
    # Tüm ROS node'larını durdur
    rosnode kill -a
    # Arka planda çalışan tüm roslaunch süreçlerini öldür (Dikkatli Kullanın!)
    pkill -f roslaunch
    pkill -f rosmaster
    pkill -f gzserver
    pkill -f gzclient
    pkill -f rviz
    # ROS loglarını temizle (isteğe bağlı)
    rosclean purge -y
    ```
2.  **Çalışma Alanını Kaynak Belirtme:** Yeni terminalde çalışma alanınızın kaynak belirtildiğinden emin olun.
    ```bash
    cd ~/robotlar_ws
    source devel/setup.bash
    ```
3.  **Simülasyon Ortamını Başlatma:** Gazebo simülasyonunu (TurtleBot3 World veya WillowGarage) başlatın.
    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    # veya
    # roslaunch turtlebot3_gazebo turtlebot3_willowgarage.launch
    ```
4.  **SLAM (Haritalama) Başlatma:** Harita oluşturmak için SLAM düğümünü başlatın.
    ```bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch
    ```
5.  Robotu klavye veya joystick ile hareket ettirerek haritanın yeterince genişlemesini sağlayın.
6.  **ROS Navigasyon Yığınını Başlatma (move_base):** Hem global (Soru 1) hem de yerel (Soru 2) planlayıcılarınızı ve parametrelerini yükleyen `move_base` launch dosyasını çalıştırın. (Ödevin örnek paketlerinden alınan `move_base.launch` dosyasını, `move_base_config.yaml` ve ilgili parametre dosyalarını (Soru 1 ve Soru 2 için) yükleyecek şekilde düzenlediğinizden emin olun).
    ```bash
    # Örneğin, global ve yerel planlayıcı konfigürasyonlarını yükleyen kendi launch dosyanızı çalıştırın:
    roslaunch your_navigation_package your_move_base_setup.launch
    ```
    *Not: Soru 1 kodlarınızdaki `blm6191_pure_pursuit_planner_plugin.xml` adlı dosya aslında bir `.launch` dosyasıydı. Bu dosyanın adını örneğin `run_coverage_and_pure_pursuit.launch` olarak değiştirip, içindeki `<launch>` etiketini koruyarak kullanabilirsiniz. İçeriğindeki `rosparam` etiketlerinin hem coverage planner hem de pure pursuit planner parametrelerini yüklediğinden ve `<include>` etiketinin `turtlebot3_navigation/launch/move_base.launch`'ı çağırdığından emin olun.*
7.  **Kapsama Alanı Poligonunu Yayınlama:** Planlama yapmak istediğiniz 4 noktalı poligonu `coverage_polygon` topic'ine yayınlayın (veya Soru 1 parametre dosyasında belirttiğiniz topic'e).
    ```bash
    rostopic pub /coverage_polygon geometry_msgs/Polygon "points:
    - {x: 1.0, y: 1.0, z: 0.0}
    - {x: 3.0, y: 1.0, z: 0.0}
    - {x: 3.0, y: 3.0, z: 0.0}
    - {x: 1.0, y: 3.0, z: 0.0}"
    ```
8.  **RViz'de Planlamayı Tetikleme ve Takibi Gözlemleme:** RViz penceresinde:
    *   Global planlayıcının yayınladığı yolu görmek için "Add" butonuna tıklayıp `global_coverage_path` topic'i altındaki `Path` display'ini ekleyin (Soru 1'deki gibi).
    *   RViz araç çubuğundaki "2D Nav Goal" butonuna tıklayın ve harita üzerinde herhangi bir yere tıklayıp sürükleyerek robotun başlangıç yönünü belirleyin. Bu, `move_base`'in global planlayıcıyı (Soru 1) tetiklemesini sağlar.
    *   Global plan (kırmızı yol) RViz'de göründükten hemen sonra, yerel Pure Pursuit planlayıcınız bu yolu `move_base` üzerinden alacak ve robotun **bu yolu takip ederek hareket etmesini** sağlayacaktır.
9.  **Robotun Takibini Gözlemleme:** RViz'de robotun (genellikle ok veya model olarak görünür) global plan boyunca ilerlemesini gözlemleyin. Robot, Pure Pursuit algoritmasının hesapladığı hız komutları sayesinde yolu takip etmeye çalışacaktır. Yolun sonunda robotun durduğunu görmeniz gerekir (goal tolerance içinde).

## Giriş ve Çıkışlar

*   **Giriş (Abonelikler):**
    *   Global plan (`std::vector<geometry_msgs::PoseStamped>` olarak `setPlan` fonksiyonuna `move_base` tarafından sağlanır).
    *   Robotun anlık pozisyonu (`costmap_ros_->getRobotPose()` ile alınır).
    *   Gerekli TF dönüşümleri (`tf_->lookupTransform()` ile alınır).
*   **Çıkış (Yayınlar):**
    *   `/cmd_vel` (`geometry_msgs::Twist`): Robotun hareket etmesi için gereken doğrusal ve açısal hız komutları (`move_base` aracılığıyla yayınlanır).
