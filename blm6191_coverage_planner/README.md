# BLM6191 Coverage Planner

Bu paket, BLM6191 Robotlar dersi Ödev 4'ün **Soru 1**'ini karşılayan bir global planlayıcı eklentisi (plugin) içermektedir. Eklenti, tanımlanan 4 köşeli bir poligon alan için robotun yarıçapına göre hesaplanmış yatay süpürme (horizontal sweep) yöntemiyle tam kapsama (full coverage) yolu planlaması yapar.

## İçerik

*   `include/blm6191_coverage_planner/coverage_planner.h`: Global planlayıcı sınıfının başlık dosyası.
*   `src/coverage_planner.cpp`: Global planlayıcı sınıfının kaynak dosyası (yatay süpürme algoritması burada yer alır).
*   `blm6191_coverage_planner_plugin.xml`: Pluginlib tanımlama dosyası.
*   `CMakeLists.txt`: Derleme ayarları.
*   `package.xml`: Paket bilgileri ve bağımlılıkları.
*   `config/`: `move_base` ve planlayıcı için örnek konfigürasyon dosyaları (Örneğin `move_base_params.yaml`, `coverage_planner_params.yaml`). 

## Gereksinimler

*   ROS (Melodic veya Noetic önerilir)
*   `catkin` build sistemi
*   `move_base` navigasyon paketi
*   `costmap_2d`
*   `nav_core`
*   `geometry_msgs`, `nav_msgs`
*   `pluginlib`
*   `roscpp`
*   `tf`, `tf2_ros`, `tf2_geometry_msgs`
*   `turtlebot3_gazebo`, `turtlebot3_slam`, `turtlebot3_navigation` (Gazebo simülasyonu ve haritalama için)

## Kurulum ve Build Etme

ROS çalışma alanınızın (`robotlar_ws` gibi) `src` klasörüne paketi klonlayın ve derleyin:

```bash
rosnode kill -a
pkill -f roslaunch # Arka planda çalışan roslaunch süreçlerini öldür
pkill -f rosmaster
pkill -f gzserver
pkill -f gzclient
pkill -f rviz


# Çalışma alanınıza gidin
cd ~/robotlar_ws/src

# Çalışma alanı kök klasörüne gidin
cd ~/robotlar_ws

# Paketi build edin
catkin_make

# Çalışma alanınızı kaynak gösterin
source devel/setup.bash

```

`catkin_make` komutunun başarılı bir şekilde tamamlandığından emin olun.

## Çalıştırma

Planlayıcıyı test etmek için tipik bir ROS navigasyon ortamına ihtiyacınız olacaktır: bir simülasyon ortamı (Gazebo), haritalama (SLAM) ve `move_base` navigasyon düğümü. Coverage planlayıcı, `move_base` içinde bir plugin olarak çalışacaktır.

1.  **ROS Ortamını Başlatma (Simülasyon, SLAM, Move Base):**

    Gazebo simülasyonunu, SLAM'i ve `move_base`'i başlatın.

    ```bash
    # Terminal 1: Gazebo simülasyonunu başlatın
    source ~/robotlar_ws/devel/setup.bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```
    ```bash
    # Terminal 2: SLAM düğümünü başlatın (Harita oluşturmak için)
    source ~/robotlar_ws/devel/setup.bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch

    # Robotu manuel olarak Gazebo veya RViz'de gezdirerek haritayı oluşturun.
    # Haritanın kapsama alanı belirleyeceğiniz bölgeyi içerecek kadar büyük olduğundan emin olun.
    ```
    ```bash
    # Terminal 3: move_base düğümünü başlatın (Global planlayıcı olarak sizin plugin'inizi kullanacak şekilde ayarlı olmalı)
    source ~/robotlar_ws/devel/setup.bash
    roslaunch turtlebot3_navigation move_base.launch
    ```


2.  **Kapsama Alanı Poligonunu Yayınlama:**

    Global planlayıcınız, `/coverage_polygon` topic'inde `geometry_msgs::Polygon` tipinde bir mesajı dinleyecektir. Ödevde belirtildiği gibi bu poligon 4 noktadan oluşmalıdır. Poligon noktalarının harita (map) frame'inde olduğunu varsayın.

    Yeni bir terminal açarak poligonu yayınlayın:

    ```bash
    # Terminal 4: Kapsama poligonunu yayınlayın
    source ~/robotlar_ws/devel/setup.bash
    rostopic pub /coverage_polygon geometry_msgs/Polygon "points:
    - {x: 1.0, y: 1.0, z: 0.0}
    - {x: 3.0, y: 1.0, z: 0.0}
    - {x: 3.0, y: 3.0, z: 0.0}
    - {x: 1.0, y: 3.0, z: 0.0}" --once # --once sadece bir kere yayınlar
    ```

    `move_base`'i çalıştıran terminalde planlayıcının "Coverage polygon received..." mesajını logladığını görmelisiniz. Eğer nokta sayısı 4 değilse uyarı göreceksiniz.

3.  **Yol Planlamayı Tetikleme (2D Nav Goal):**

    RViz arayüzünde (genellikle `move_base` launch dosyası RViz'i de başlatır), üst araç çubuğunda bulunan "2D Nav Goal" butonuna tıklayın. Harita üzerinde herhangi bir yere bir hedef noktası çizin (sürükleyerek yön de verebilirsiniz).

    Bu işlem, `move_base`'in global planlayıcısını tetikleyecektir. Eğer geçerli bir 4 köşeli poligon alınmışsa, planlayıcı kapsama yolunu hesaplayacaktır.

4.  **Hesaplanan Yolu Görselleştirme:**

    Planlayıcı tarafından hesaplanan kapsama yolu, default olarak `global_coverage_path` topic'inde (`/move_base/blm6191_coverage_planner/global_coverage_path`) `nav_msgs::Path` mesajı olarak yayınlanır.

    RViz'de bu yolu görmek için:
    *   "Displays" panelindeki "Add" butonuna tıklayın.
    *   "By Topic" sekmesine gidin.
    *   Yayınlanan `Path` topic'ini (`/move_base/blm6191_coverage_planner/global_coverage_path`) bulun ve ekleyin.
    *   RViz ekranında hesaplanan kapsama yolunu görmelisiniz (Ödevdeki Şekil 1 ve 2'ye benzer).

## Parametreler

`blm6191_coverage_planner` düğümünün kullandığı bazı parametreler (`move_base` tarafından yüklenen konfigürasyon dosyalarında, `blm6191_coverage_planner` namespace'i altında ayarlanır):

*   `robot_radius` (double, default: 0.25): Robotun yarıçapı. Bu değer, tarama şeritleri arasındaki mesafeyi (swath width = 2 * robot_radius) belirler.
*   `polygon_topic` (string, default: "coverage_polygon"): Planlayıcının kapsama alanı poligonunu dinleyeceği topic adı.

## Giriş ve Çıkış Topicleri

*   **Giriş:**
    *   `/coverage_polygon` (`geometry_msgs::Polygon`): Kapsama alanı olarak kullanılacak 4 köşeli poligon. Planlayıcı bu topic'e abone olur.
*   **Çıkış:**
    *   `/move_base/blm6191_coverage_planner/global_coverage_path` (`nav_msgs::Path`): Hesaplanan kapsama yolunu yayınlar. RViz'de görselleştirmek için kullanılabilir. Topic adı, `move_base` düğümü altında, plugin'in adı ile oluşur.

