# BLM6191 Robotlar - Ödev 4 / Soru 1: Kapsama Alanı Global Planlayıcısı

Bu ROS paketi, BLM6191 Robotlar dersi Ödev 4'ün ilk sorusunu çözmek amacıyla geliştirilmiş bir **global planlayıcı eklentisidir**. ROS navigasyon yığınının (navigation stack) `move_base` düğümü ile entegre olarak çalışır.

## Özellikler

*   Tanımlanmış, 4 köşeli bir poligon alan içerisinde **tam kapsama yolu (full coverage path)** planlar.
*   Robotun dairesel kapsama alanının yarıçapına (**robot_radius**) göre tarama çizgileri arasındaki mesafeyi parametrik olarak ayarlar.
*   Global planlayıcı olarak `nav_core::BaseGlobalPlanner` arayüzünü uygular.
*   Planlanacak poligon alanını bir ROS topic'i üzerinden `geometry_msgs::Polygon` mesajı olarak alır.
*   Hesaplanan kapsama yolunu bir `nav_msgs::Path` mesajı olarak yayınlar ve `move_base`'e iletir.

## Çözüm Yaklaşımı

Bu global planlayıcı, poligon içerisindeki alanı kapsamak için **yatay tarama (horizontal sweep)** veya **çim biçme (lawnmower)** paterni yöntemini kullanır.

1.  **Poligon Girişi:** Planlayıcı, parametrelerde belirtilen bir topic üzerinden 4 köşeli poligonu (`geometry_msgs::Polygon`) dinler. Planlama, ancak geçerli bir poligon alındıktan sonra yapılır.
2.  **Tarama Alanı ve Yoğunluğu:** Alınan poligonun dikey (Y ekseni) sınırları (minimum ve maksimum Y değerleri) belirlenir. Tarama, bu dikey sınırlar arasında robotun çapı kadar (`2 * robot_radius`) aralıklarla yatay çizgiler çizerek yapılır. Bu aralık, robotun bir tarama çizgisinde ilerlerken yanındaki şeridi de kapsamasını sağlamak içindir.
3.  **Kesişim Noktaları:** Her yatay tarama çizgisinin poligonun kenarlarıyla kesiştiği noktalar hesaplanır.
4.  **Yol Oluşturma:** Her tarama çizgisi üzerindeki kesişim noktaları çiftler halinde gruplanır (poligon içindeki segmentler). Robot, bu segmentler boyunca bir yönde (örneğin soldan sağa) ilerler. Bir sonraki tarama çizgisine geçtiğinde ise yön değiştirir (sağdan sola veya tam tersi), böylece **serpantin (boustron)** benzeri bir yol paterni oluşturulur. Bu yön değiştirme, bir tarama çizgisinden diğerine geçerken kat edilen mesafeyi ve dönüş sayısını azaltır.
5.  **Çıktı:** Elde edilen sıralı noktalar kümesi, `nav_msgs::Path` mesajı olarak paketlenir ve hem görselleştirme için yayınlanır hem de `move_base` tarafından yerel planlayıcıya (Soru 2'deki Pure Pursuit gibi) takip edilmesi için verilir.

Planlayıcı, `makePlan` fonksiyonu çağrıldığında (genellikle RViz'deki "2D Nav Goal" butonuna tıklandığında), mevcut robot pozisyonundan başlayarak alınan poligon içinde kapsama yolunu hesaplar. "2D Nav Goal" ile belirlenen hedef noktası, kapsama planlayıcısı için *gidilecek nihai bir hedef* olmaktan ziyade, sadece *planlama sürecini tetikleyen* bir olaydır. Kapsama yolu poligonu kapsayacak şekilde belirlenir, spesifik hedef noktasına kadar değil.

## Bağımlılıklar

Bu paketin çalışabilmesi için aşağıdaki ROS paketlerine ihtiyacı vardır:

*   `costmap_2d`
*   `geometry_msgs`
*   `nav_core`
*   `nav_msgs`
*   `pluginlib`
*   `roscpp`
*   `tf`
*   `tf2_ros`
*   `tf2_geometry_msgs`

Genellikle `rosdep install --from-paths <your_ws_src_path> --ignore-src -r -y` komutu ile bu bağımlılıkları kurabilirsiniz.

## Kurulum ve Derleme

1.  ROS çalışma alanınızın (`robotlar_ws` gibi) `src` klasörüne gidin:
    ```bash
    cd ~/robotlar_ws/src
    ```
2.  Bu paketi Gitlab'dan klonlayın. 
    ```bash
    git clone https://gitlab.com/blm6191_2425b_tai/members/24501111/blm6191_coverage_planner.git
    ```
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

Planlayıcının davranışını `config` klasöründeki parametre dosyaları aracılığıyla ayarlayabilirsiniz.

*   **`coverage_planner_params.yaml`**: Bu dosya, global planlayıcının kendi özel parametrelerini içerir.
    *   `robot_radius`: Robotun metre cinsinden yarıçapı. Bu değer, kapsanan şeridin genişliğini belirler ve tarama çizgileri arasındaki mesafeyi (`2 * robot_radius`) doğrudan etkiler.
    *   `polygon_topic`: Planlanacak kapsama alanını tanımlayan `geometry_msgs::Polygon` mesajının yayınlandığı topic adı. Varsayılan: `"coverage_polygon"`.

*   **`move_base_config.yaml`**: Bu dosya, `move_base` düğümünün genel konfigürasyonunu yapar ve bu planlayıcının global planlayıcı olarak kullanılacağını belirtir.
    *   `base_global_planner: blm6191_coverage_planner/CoveragePlanner` satırı, `move_base`'in bu eklentiyi kullanmasını sağlar.
    *   Planlayıcının kendi parametreleri (`robot_radius`, `polygon_topic`) de burada `blm6191_coverage_planner` namespace'i altında yeniden belirtilir ve `move_base` başlatılırken planlayıcıya iletilir.

Bu konfigürasyon dosyalarının `move_base.launch` dosyanız tarafından doğru şekilde yüklendiğinden emin olun. Genellikle `move_base` düğümünün `params` veya `respawn_params` argümanları kullanılarak yapılır.

## Kullanım Adımları

Bu planlayıcıyı simülasyon ortamında test etmek için aşağıdaki adımları izleyebilirsiniz:

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

    ```
4.  **SLAM (Haritalama) Başlatma:** Harita oluşturmak için SLAM düğümünü başlatın.
    ```bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch
    ```
5.  Robotu klavye veya joystick ile hareket ettirerek haritanın yeterince genişlemesini ve kapsama planlaması yapmak istediğiniz alanı içermesini sağlayın.
6.  **ROS Navigasyon Yığınını Başlatma (move_base):** Özelleştirilmiş konfigürasyon dosyalarınızı yükleyen `move_base` launch dosyasını çalıştırın. (Bu launch dosyasının, `move_base_config.yaml` dosyasını ve dolayısıyla sizin `coverage_planner_params.yaml` dosyanızı yüklediğinden emin olun.)
    ```bash
    roslaunch turtlebot3_navigation move_base.launch
    ```
 
7.  **Kapsama Alanı Poligonunu Yayınlama:** Planlama yapmak istediğiniz 4 noktalı poligonu `coverage_polygon` topic'ine yayınlayın (veya parametre dosyasında belirttiğiniz topic'e).
    ```bash
    rostopic pub /coverage_polygon geometry_msgs/Polygon "points:
    - {x: 1.0, y: 1.0, z: 0.0}
    - {x: 3.0, y: 1.0, z: 0.0}
    - {x: 3.0, y: 3.0, z: 0.0}
    - {x: 1.0, y: 3.0, z: 0.0}"
    ```

8.  **Planlamayı Tetikleme:** RViz araç çubuğundaki "2D Nav Goal" butonuna tıklayın ve harita üzerinde herhangi bir yere tıklayıp sürükleyerek robotun başlangıç yönünü belirleyin. Bu, `move_base`'in `makePlan` fonksiyonunuzu çağırmasını tetikleyecektir.

## Giriş ve Çıkışlar

*   **Giriş (Abonelikler):**
    *   `/coverage_polygon` (`geometry_msgs::Polygon`): Planlanacak 4 köşeli poligon alanı. Topic adı `polygon_topic` parametresi ile değiştirilebilir.
*   **Çıkış (Yayınlar):**
    *   `~/global_coverage_path` (`nav_msgs::Path`): Global planlayıcı tarafından hesaplanan kapsama yolu. Düğümün kendi namespace'i altında yayınlanır.
