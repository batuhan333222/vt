import sys
import time
import threading
import cv2
import numpy as np

from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QPushButton, QLineEdit, QTextEdit,
                             QMessageBox, QGridLayout, QSizePolicy)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException

# ArduPilot'un VTOL modları listesi (yaygın olanlar)
VTOL_MODES = ["AUTO", "GUIDED", "LOITER", "QHOVER", "QLAND", "QSTABILIZE"] # Q* modları VTOL'a özgüdür

class DronekitThread(QThread):
    """
    Dronekit bağlantısını yöneten ve araç verilerini alan arka plan iş parçacığı.
    """
    connection_status_signal = pyqtSignal(str) # Bağlantı durumu için sinyal
    vehicle_data_signal = pyqtSignal(dict)     # Araç verileri için sinyal (konum, mod, arm durumu vb.)
    log_message_signal = pyqtSignal(str)      # Log mesajları için sinyal

    def __init__(self, connection_string):
        super().__init__()
        self.connection_string = connection_string
        self.vehicle = None
        self._stop_event = threading.Event() # Thread'i durdurmak için Event

    def run(self):
        """
        Thread'in ana çalışma fonksiyonu. Bağlantı kurar ve veri döngüsünü çalıştırır.
        """
        self.connection_status_signal.emit(f"Connecting to {self.connection_string}...")
        try:
            # Bağlantıyı kur
            # timeout=60: Bağlantı denemesi için 60 saniye süre ver
            # baud: Seri bağlantılar için baud rate (örneğin: 57600 veya 115200)
            # Eğer bağlantı stringi udp veya tcp ise baud rate gerekmez.
            # self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60, baud=57600) # Seri port için baud ekle
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
            self.connection_status_signal.emit("Connected!")
            self.log_message_signal.emit("Vehicle connected successfully.")

            # Bağlantı kalitesini kontrol etmek için (basit bir kontrol)
            if not self.vehicle.is_armable:
                self.log_message_signal.emit("WARNING: Vehicle not armable (check pre-arm checks).")

            # Veri alma döngüsü
            while not self._stop_event.is_set():
                if self.vehicle:
                    try:
                        # Araç verilerini al
                        # Konum bilgisi geldiğinden emin olana kadar bekleyebiliriz:
                        # if self.vehicle.location.global_frame.lat == 0 or self.vehicle.location.global_frame.lon == 0:
                        #    self.log_message_signal.emit("Waiting for valid GPS fix...")
                        #    time.sleep(1)
                        #    continue # Döngünün başına dön

                        location = self.vehicle.location.global_frame
                        mode = self.vehicle.mode.name
                        is_armable = self.vehicle.is_armable
                        armed = self.vehicle.armed
                        system_status = self.vehicle.system_status.state
                        # battery = self.vehicle.battery # Pil bilgisini de alabilirsiniz

                        # Verileri ana GUI thread'ine sinyal ile gönder
                        self.vehicle_data_signal.emit({
                            "location": {"lat": location.lat, "lon": location.lon, "alt": location.alt} if location and location.lat is not None else None,
                            "mode": mode,
                            "is_armable": is_armable,
                            "armed": armed,
                            "system_status": system_status,
                            # "battery": {"voltage": battery.voltage, "current": battery.current, "level": battery.level} if battery else None
                        })

                        # Daha sık veri almak isterseniz buradaki sleep süresini kısaltın
                        time.sleep(0.2) # 200 ms bekle (5 Hz güncelleme)

                    except APIException as e:
                        self.log_message_signal.emit(f"API Exception during data fetch: {e}")
                    except Exception as e:
                         # Çoğu zaman bağlantı hatası buradan yakalanır
                         self.log_message_signal.emit(f"Error during data fetch: {e}")
                         # Hata durumunda bağlantıyı kesip thread'i durdurabiliriz
                         self.stop() # Güvenli çıkış için stop'u çağır
                         break # Döngüyü kır

                else:
                     # Vehicle nesnesi yoksa, bir sorun var demektir (bağlantı başarısız oldu)
                     break # Döngüyü kır

        except APIException as e:
            self.connection_status_signal.emit("Connection Failed")
            self.log_message_signal.emit(f"Connection API Exception: {e}")
        except Exception as e:
            self.connection_status_signal.emit("Connection Failed")
            self.log_message_signal.emit(f"Connection Error: {e}")

        # Bağlantı kesildiğinde veya thread durdurulduğunda
        if self.vehicle:
            try:
                self.vehicle.close()
                self.log_message_signal.emit("Vehicle connection closed.")
            except Exception as e:
                 self.log_message_signal.emit(f"Error closing vehicle connection: {e}")
            self.vehicle = None

        self.connection_status_signal.emit("Disconnected")


    def stop(self):
        """
        Thread'in çalışma döngüsünü durdurmak için kullanılır.
        """
        self._stop_event.set()
        # vehicle.close() run metodu sonunda çağrılıyor, thread'in bitmesi beklenmeli.


    def arm_vehicle(self):
        """Aracı arm etme komutunu gönderir."""
        if self.vehicle and self.vehicle.is_armable:
            if not self.vehicle.armed:
                self.log_message_signal.emit("Arming vehicle...")
                try:
                    # Copter/QuadPlane VTOL'da, stabilize modunda arm edip motorları döndürmek için:
                    # self.vehicle.mode = VehicleMode("STABILIZE")
                    # self.vehicle.arm(wait=True) # wait=True aracı arm olana kadar bekler
                    # Dronekit arm() metodunu kullanmak genellikle yeterlidir:
                    self.vehicle.arm()
                    self.log_message_signal.emit("Arming command sent.")
                    # ARM durumunun güncellenmesi update_vehicle_data sinyali ile takip edilir
                except APIException as e:
                    self.log_message_signal.emit(f"Arming failed: {e}")
                except Exception as e:
                     self.log_message_signal.emit(f"Arming error: {e}")
            else:
                self.log_message_signal.emit("Vehicle is already armed.")
        elif self.vehicle and not self.vehicle.is_armable:
             self.log_message_signal.emit("Vehicle not armable. Check pre-arm requirements.")
        else:
            self.log_message_signal.emit("Cannot arm: Vehicle not connected.")


    def disarm_vehicle(self):
        """Aracı disarm etme komutunu gönderir."""
        if self.vehicle and self.vehicle.armed:
            self.log_message_signal.emit("Disarming vehicle...")
            try:
                # DISARM komutu genellikle yalnızca aracı yerde iken çalışır.
                # Havada DISARM tehlikelidir ve çoğu modda engellenir.
                # Eğer aracın yerde olduğundan eminseniz kullanın.
                self.vehicle.armed = False
                 # DISARM durumunun güncellenmesi update_vehicle_data sinyali ile takip edilir
                self.log_message_signal.emit("Disarming command sent.")
            except APIException as e:
                self.log_message_signal.emit(f"Disarming failed: {e}")
            except Exception as e:
                 self.log_message_signal.emit(f"Disarming error: {e}")
        elif self.vehicle and not self.vehicle.armed:
            self.log_message_signal.emit("Vehicle is already disarmed.")
        else:
            self.log_message_signal.emit("Cannot disarm: Vehicle not connected.")

    def set_mode(self, mode_name):
        """Aracın uçuş modunu değiştirir."""
        if self.vehicle:
            try:
                mode_name = mode_name.upper() # Mod isimleri genellikle büyük harf
                if mode_name not in self.vehicle.mode.name: # Eğer zaten istenen modda değilse
                    self.log_message_signal.emit(f"Attempting to set mode to {mode_name}...")
                    self.vehicle.mode = VehicleMode(mode_name)
                    # Mod değişiminin tamamlanmasını beklemek için kısa bir gecikme
                    # veya durum kontrolü eklenebilir, ancak thread'in data döngüsü zaten takip ediyor.
                    # time.sleep(0.5)
                    self.log_message_signal.emit(f"Mode change command sent for {mode_name}.")
                else:
                     self.log_message_signal.emit(f"Vehicle is already in {mode_name} mode.")
            except APIException as e:
                self.log_message_signal.emit(f"Setting mode failed: {e}")
            except Exception as e:
                self.log_message_signal.emit(f"Setting mode error: {e}")
        else:
            self.log_message_signal.emit("Cannot set mode: Vehicle not connected.")


class VideoThread(QThread):
    """
    RTSP yayınını okuyan ve GUI'ye frame gönderen thread.
    """
    frame_signal = pyqtSignal(QImage)
    stream_status_signal = pyqtSignal(str)
    log_message_signal = pyqtSignal(str)

    def __init__(self, rtsp_url):
        super().__init__()
        self.rtsp_url = rtsp_url
        self._stop_event = threading.Event()
        self.cap = None

    def run(self):
        self.stream_status_signal.emit("Connecting to stream...")
        self.log_message_signal.emit(f"Attempting to connect to RTSP stream: {self.rtsp_url}")

        # OpenCV VideoCapture
        # cv2.CAP_FFMPEG backend'ini belirtmek bazen RTSP desteğini iyileştirebilir
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            self.stream_status_signal.emit("Stream Connection Failed")
            self.log_message_signal.emit(f"Failed to open RTSP stream: {self.rtsp_url}")
            self.cap = None
            return

        self.stream_status_signal.emit("Streaming...")
        self.log_message_signal.emit("RTSP stream connected successfully.")

        while not self._stop_event.is_set() and self.cap.isOpened():
            ret, frame = self.cap.read() # Kareyi oku

            if not ret:
                # Stream sonu veya hata
                self.log_message_signal.emit("Stream ended or error reading frame.")
                break

            # OpenCV frame'i BGR formatındadır. PyQt'nin QImage'ı genellikle RGB bekler.
            # RGB'ye çevir:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w # Satır başına düşen byte sayısı

            # QImage oluştur:
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # GUI'ye sinyal gönder:
            self.frame_signal.emit(qt_image)

            # Akışı çok hızlı okumaması için küçük bir bekleme (isteğe bağlı, read() zaten bloke olabilir)
            # time.sleep(0.01) # 10 ms bekle (yaklaşık 100 FPS, çok fazla olabilir)
            # Genellikle buna gerek yoktur, cv2.VideoCapture.read() bir sonraki kare gelene kadar bloke olur.

        # Döngü bittiğinde veya hata oluştuğunda
        if self.cap:
            self.cap.release()
            self.log_message_signal.emit("RTSP stream released.")
            self.cap = None

        if not self._stop_event.is_set():
             self.stream_status_signal.emit("Stream Disconnected") # Normal durdurulmadıysa durumu güncelle
        else:
             self.stream_status_signal.emit("Stream Stopped") # Stop metodu çağrıldıysa


    def stop(self):
        """Thread'in çalışma döngüsünü durdurmak için kullanılır."""
        self._stop_event.set()
        # cap.release() run metodu sonunda çağrılıyor, thread'in bitmesi beklenmeli.


class GCSWindow(QMainWindow):
    """
    Ana yer kontrol istasyonu penceresi.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Basic VTOL GCS with RTSP")
        self.setGeometry(100, 100, 1000, 800) # Pencere boyutunu ayarla

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget) # Ana layout yatay olsun

        # Sol Panel: Bağlantı, Durum, Kontroller, Loglar
        self.left_panel_layout = QVBoxLayout()
        self.main_layout.addLayout(self.left_panel_layout, 1) # Sol panel 1 oranında yer kaplasın

        # Sağ Panel: Harita/Video
        self.right_panel_layout = QVBoxLayout()
        self.main_layout.addLayout(self.right_panel_layout, 2) # Sağ panel 2 oranında yer kaplasın

        self.dronekit_thread = None # Dronekit thread nesnesi
        self.video_thread = None    # Video thread nesnesi

        self._create_connection_interface(self.left_panel_layout)
        self._create_status_display(self.left_panel_layout)
        self._create_control_buttons(self.left_panel_layout)
        self._create_log_area(self.left_panel_layout)

        self._create_map_placeholder(self.right_panel_layout) # Harita sağ panele
        self._create_video_interface(self.right_panel_layout) # Video sağ panele


        self._update_button_states(connected=False, armed=False, armable=False) # Başlangıçta butonları devre dışı bırak
        self._update_video_button_state(streaming=False)


    def _create_connection_interface(self, parent_layout):
        """Bağlantı arayüzü widget'larını oluşturur."""
        conn_group_layout = QVBoxLayout()
        conn_group_layout.addWidget(QLabel("<b>Vehicle Connection</b>"))

        conn_input_layout = QHBoxLayout()
        self.conn_label = QLabel("Conn String:")
        self.conn_input = QLineEdit()
        self.conn_input.setText("udp:127.0.0.1:14550") # Varsayılan bağlantı stringi (SITL için)
        self.conn_input.setPlaceholderText("E.g., udp:127.0.0.1:14550 or /dev/ttyACM0")

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_vehicle)

        conn_input_layout.addWidget(self.conn_label)
        conn_input_layout.addWidget(self.conn_input)
        conn_input_layout.addWidget(self.connect_button)

        self.connection_status_label = QLabel("Status: Disconnected")
        self.connection_status_label.setStyleSheet("color: red;") # Bağlantı yoksa kırmızı

        conn_group_layout.addLayout(conn_input_layout)
        conn_group_layout.addWidget(self.connection_status_label)
        conn_group_layout.addStretch(1) # Boş alanı doldur

        parent_layout.addLayout(conn_group_layout)
        parent_layout.addSpacing(20) # Bölümler arasına boşluk ekle


    def _create_status_display(self, parent_layout):
        """Araç durumu (konum, mod vb.) gösteren widget'ları oluşturur."""
        status_group_layout = QVBoxLayout()
        status_group_layout.addWidget(QLabel("<b>Vehicle Status</b>"))

        self.location_label = QLabel("Location: N/A")
        self.mode_label = QLabel("Mode: N/A")
        self.armable_label = QLabel("Armable: N/A")
        self.armed_label = QLabel("Armed: N/A")
        self.system_status_label = QLabel("System Status: N/A") # Bağlantı kalitesi placeholder

        status_group_layout.addWidget(self.location_label)
        status_group_layout.addWidget(self.mode_label)
        status_group_layout.addWidget(self.armable_label)
        status_group_layout.addWidget(self.armed_label)
        status_group_layout.addWidget(self.system_status_label)

        parent_layout.addLayout(status_group_layout)
        parent_layout.addSpacing(20)


    def _create_map_placeholder(self, parent_layout):
        """Harita görselleştirme için bir yer tutucu (placeholder) oluşturur."""
        map_group_layout = QVBoxLayout()
        map_group_layout.addWidget(QLabel("<b>Map Placeholder</b>"))

        # Gerçek bir harita widget'ı buraya entegre edilebilir
        # Şimdilik sadece bir etiket ve çerçeve
        self.map_placeholder_label = QLabel("Vehicle Position will be shown here or coordinates displayed.")
        self.map_placeholder_label.setAlignment(Qt.AlignCenter)
        self.map_placeholder_label.setStyleSheet("border: 1px solid gray; min-height: 200px;") # Görsel bir sınır ekle
        # Bu widget'ın büyümesine izin ver
        self.map_placeholder_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        map_group_layout.addWidget(self.map_placeholder_label)

        parent_layout.addLayout(map_group_layout)
        # Harita ve video arasında boşluk bırakmayın, sağ panelin kalanını doldursunlar


    def _create_control_buttons(self, parent_layout):
        """Kontrol butonlarını (ARM/DISARM, Modlar) oluşturur."""
        control_group_layout = QVBoxLayout()
        control_group_layout.addWidget(QLabel("<b>Vehicle Control</b>"))

        # ARM/DISARM Butonları
        arm_disarm_layout = QHBoxLayout()
        self.arm_button = QPushButton("ARM")
        self.arm_button.setStyleSheet("background-color: lightgreen;")
        self.arm_button.clicked.connect(self.on_arm_button_clicked)

        self.disarm_button = QPushButton("DISARM")
        self.disarm_button.setStyleSheet("background-color: salmon;")
        self.disarm_button.clicked.connect(self.on_disarm_button_clicked)

        arm_disarm_layout.addWidget(self.arm_button)
        arm_disarm_layout.addWidget(self.disarm_button)
        control_group_layout.addLayout(arm_disarm_layout)

        # Mod Butonları
        mode_layout = QGridLayout()
        mode_layout.addWidget(QLabel("<b>Modes:</b>"), 0, 0, 1, -1) # Başlık satırı
        row, col = 1, 0
        for mode_name in VTOL_MODES:
            button = QPushButton(mode_name)
            # Lambda kullanarak butona özel mod adını clicked sinyaline bağla
            button.clicked.connect(lambda checked, mode=mode_name: self.on_mode_button_clicked(mode))
            mode_layout.addWidget(button, row, col)
            col += 1
            if col > 3: # Her satırda 4 buton olsun
                col = 0
                row += 1
        control_group_layout.addLayout(mode_layout)

        parent_layout.addLayout(control_group_layout)
        parent_layout.addSpacing(20)


    def _create_video_interface(self, parent_layout):
        """RTSP video arayüzü widget'larını oluşturur."""
        video_group_layout = QVBoxLayout()
        video_group_layout.addWidget(QLabel("<b>Video Stream</b>"))

        video_input_layout = QHBoxLayout()
        self.rtsp_label = QLabel("RTSP URL:")
        self.rtsp_input = QLineEdit()
        # Örnek RTSP URL (Genellikle kameranın IP adresi ve portu/yolu)
        # Test için bir test yayını kullanabilirsiniz, örn: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mp4
        self.rtsp_input.setText("rtsp://<kameranın_ip_adresi>:<port>/<yayın_yolu>")
        self.rtsp_input.setPlaceholderText("E.g., rtsp://192.168.1.100:554/stream1")

        self.stream_button = QPushButton("Start Stream")
        self.stream_button.clicked.connect(self.start_stop_stream)

        video_input_layout.addWidget(self.rtsp_label)
        video_input_layout.addWidget(self.rtsp_input)
        video_input_layout.addWidget(self.stream_button)

        self.video_display_label = QLabel("No video stream")
        self.video_display_label.setAlignment(Qt.AlignCenter)
        self.video_display_label.setStyleSheet("border: 1px solid gray; background-color: black;")
        # Video görüntüsünün label'ın boyutuna sığmasını sağla
        self.video_display_label.setScaledContents(True)
        # Bu widget'ın büyümesine izin ver
        self.video_display_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.stream_status_label = QLabel("Status: Idle")
        self.stream_status_label.setStyleSheet("color: gray;")

        video_group_layout.addLayout(video_input_layout)
        video_group_layout.addWidget(self.video_display_label)
        video_group_layout.addWidget(self.stream_status_label)

        parent_layout.addLayout(video_group_layout)
        # Sağ paneldeki harita ve video alanlarının esnek olması için stretch ekle
        parent_layout.addStretch(1) # Hem harita hem video placeholder'ı genişleyebilir hale gelir


    def _create_log_area(self, parent_layout):
        """Log mesajlarını gösteren alanı oluşturur."""
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setFont(QFont("Courier", 9)) # Monospace font loglar için iyi olabilir
        self.log_text_edit.setMinimumHeight(100) # Log alanı için minimum yükseklik belirle

        parent_layout.addWidget(QLabel("<b>Log / Status Messages</b>"))
        parent_layout.addWidget(self.log_text_edit)
        # Log alanının aşağıya doğru genişlemesini sağlayabiliriz
        # parent_layout.addStretch(1) # İsteğe bağlı: Logların en altta kalmasını sağlar


    # --- Signal Slot Bağlantıları ve İşleyicileri ---

    def connect_vehicle(self):
        """Bağlantı butonuna basıldığında çağrılır."""
        if self.dronekit_thread is not None and self.dronekit_thread.isRunning():
            # Bağlantı zaten aktif veya kuruluyor - disconnect işlemi yapalım
            self.disconnect_vehicle()
            return

        connection_string = self.conn_input.text()
        if not connection_string:
            self.log_message("Please enter a connection string.")
            return

        self.dronekit_thread = DronekitThread(connection_string)
        # Thread sinyallerini GUI slotlarına bağla
        self.dronekit_thread.connection_status_signal.connect(self.update_connection_status)
        self.dronekit_thread.vehicle_data_signal.connect(self.update_vehicle_data)
        self.dronekit_thread.log_message_signal.connect(self.log_message)

        self.connect_button.setEnabled(False) # Bağlantı kurulurken butonu devre dışı bırak
        self.connect_button.setText("Connecting...")
        self.dronekit_thread.start() # Thread'i başlat

    def disconnect_vehicle(self):
         """Bağlantıyı kesme butonuna basıldığında veya Connect butonu tekrar basıldığında çağrılır."""
         if self.dronekit_thread and self.dronekit_thread.isRunning():
             self.log_message("Disconnecting vehicle...")
             self.dronekit_thread.stop() # Durdurma sinyali ver
             self.connect_button.setEnabled(False) # Disconnect sırasında butonu devre dışı bırak
             self.connect_button.setText("Disconnecting...")
             # Thread'in durduğunu update_connection_status sinyali ile anlayacağız
         else:
             self.log_message("Vehicle not connected.")


    def update_connection_status(self, status):
        """Dronekit thread'inden gelen bağlantı durumu sinyalini işler."""
        self.connection_status_label.setText(f"Status: {status}")
        if "Connected" in status:
            self.connection_status_label.setStyleSheet("color: green;")
            self.connect_button.setEnabled(True)
            self.connect_button.setText("Disconnect")
            # İlk bağlantıda buton durumları update_vehicle_data gelince güncellenir
        else:
            self.connection_status_label.setStyleSheet("color: red;")
            self.connect_button.setEnabled(True)
            self.connect_button.setText("Connect")
            if self.dronekit_thread and not self.dronekit_thread.isRunning():
                 # Eğer thread durduysa ve bağlantı yoksa, thread referansını temizle
                 self.dronekit_thread.wait() # Thread'in tamamen bitmesini bekle
                 self.dronekit_thread = None
            # Bağlantı kesildiğinde tüm kontrol butonlarını devre dışı bırak
            self._update_button_states(connected=False, armed=False, armable=False)


    def update_vehicle_data(self, data):
        """Dronekit thread'inden gelen araç verileri sinyalini işler."""
        location = data.get("location")
        mode = data.get("mode", "N/A")
        is_armable = data.get("is_armable", False)
        armed = data.get("armed", False)
        system_status = data.get("system_status", "N/A")
        # battery = data.get("battery")

        if location and location['lat'] is not None and location['lon'] is not None:
             self.location_label.setText(f"Location: Lat={location['lat']:.6f}, Lon={location['lon']:.6f}, Alt={location['alt']:.2f}m")
             # İleride buraya harita marker güncelleme kodu gelecek
             self.map_placeholder_label.setText(f"Current Location:\nLat: {location['lat']:.6f}\nLon: {location['lon']:.6f}\nAlt: {location['alt']:.2f}m")
        else:
            self.location_label.setText("Location: Waiting for GPS...")
            self.map_placeholder_label.setText("Waiting for location data...")


        self.mode_label.setText(f"Mode: {mode}")
        self.armable_label.setText(f"Armable: {'Yes' if is_armable else 'No'}")
        self.armed_label.setText(f"Armed: {'Yes' if armed else 'No'}")
        self.system_status_label.setText(f"System Status: {system_status}")
        # if battery:
        #      self.battery_label.setText(f"Battery: {battery['voltage']:.2f}V, {battery['level']}%")


        # ARM/DISARM ve Mod butonlarının durumunu güncelle
        self._update_button_states(connected=True, armed=armed, armable=is_armable)


    def _update_button_states(self, connected, armed, armable):
        """Araç kontrol butonlarının enabled/disabled durumlarını ayarlar."""
        # Mod butonları sadece bağlıyken ve disarmed durumdayken (genellikle) aktif olsun
        # Bazı modlar arm durumdayken de seçilebilir, bu duruma göre ayarlanabilir.
        # Basitlik için şimdilik sadece bağlı ve disarmed iken mod değişimi etkin:
        can_change_mode = connected and not armed
        for button in self.findChildren(QPushButton):
            if button.text() in VTOL_MODES:
                button.setEnabled(can_change_mode)

        # ARM/DISARM butonları
        self.arm_button.setEnabled(connected and armable and not armed)
        self.disarm_button.setEnabled(connected and armed)

        # Bağlantı butonu durumu update_connection_status içinde yönetiliyor


    def on_arm_button_clicked(self):
        """ARM butonuna basıldığında çağrılır."""
        if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
             # Komutu thread'e gönder
            self.dronekit_thread.arm_vehicle()
        else:
            self.log_message("Cannot arm: Vehicle not connected.")


    def on_disarm_button_clicked(self):
        """DISARM butonuna basıldığında çağrılır."""
        if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
            self.dronekit_thread.disarm_vehicle()
        else:
            self.log_message("Cannot disarm: Vehicle not connected.")

    def on_mode_button_clicked(self, mode_name):
        """Mod butonlarından birine basıldığında çağrılır."""
        if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
            self.dronekit_thread.set_mode(mode_name)
        else:
            self.log_message(f"Cannot set mode {mode_name}: Vehicle not connected.")

    # --- Video Stream Signal Slot Bağlantıları ve İşleyicileri ---

    def start_stop_stream(self):
        """Video stream başlat/durdur butonuna basıldığında çağrılır."""
        if self.video_thread is not None and self.video_thread.isRunning():
            # Akış aktif, durdurma talebi
            self.stop_stream()
        else:
            # Akış inaktif, başlatma talebi
            rtsp_url = self.rtsp_input.text()
            if not rtsp_url:
                self.log_message("Please enter an RTSP URL.")
                return

            self.video_thread = VideoThread(rtsp_url)
            # Thread sinyallerini GUI slotlarına bağla
            self.video_thread.frame_signal.connect(self.update_video_frame)
            self.video_thread.stream_status_signal.connect(self.update_stream_status)
            self.video_thread.log_message_signal.connect(self.log_message)

            self.stream_button.setEnabled(False) # Başlatılırken butonu devre dışı bırak
            self.stream_button.setText("Connecting...")
            self.stream_status_label.setStyleSheet("color: orange;")
            self.video_thread.start() # Thread'i başlat


    def stop_stream(self):
        """Video stream durdurma işlemi."""
        if self.video_thread and self.video_thread.isRunning():
            self.log_message("Stopping video stream...")
            self.video_thread.stop() # Durdurma sinyali ver
            self.stream_button.setEnabled(False) # Durdurulurken butonu devre dışı bırak
            self.stream_button.setText("Stopping...")
             # Thread'in durduğunu update_stream_status sinyali ile anlayacağız
        else:
            self.log_message("Video stream is not active.")


    def update_video_frame(self, q_image):
        """Video thread'inden gelen QImage karesini gösterir."""
        # QImage'ı QPixmap'e çevir ve Label'a ata
        pixmap = QPixmap.fromImage(q_image)
        self.video_display_label.setPixmap(pixmap)
        # setScaledContents(True) zaten resmin label'ın boyutuna sığmasını sağlıyor.

    def update_stream_status(self, status):
        """Video thread'inden gelen stream durumu sinyalini işler."""
        self.stream_status_label.setText(f"Status: {status}")
        if "Streaming" in status:
            self.stream_status_label.setStyleSheet("color: green;")
            self.stream_button.setEnabled(True)
            self.stream_button.setText("Stop Stream")
            self._update_video_button_state(streaming=True)
        elif "Failed" in status or "Disconnected" in status:
             self.stream_status_label.setStyleSheet("color: red;")
             self.stream_button.setEnabled(True)
             self.stream_button.setText("Start Stream")
             self.video_display_label.setText("No video stream") # Hata veya kopma durumunda ekranı temizle
             self.video_display_label.setPixmap(QPixmap()) # Pixmap'i temizle
             if self.video_thread and not self.video_thread.isRunning():
                  self.video_thread.wait() # Thread'in tamamen bitmesini bekle
                  self.video_thread = None
             self._update_video_button_state(streaming=False)
        elif "Connecting" in status:
            self.stream_status_label.setStyleSheet("color: orange;")
            # Buton durumu zaten start_stop_stream içinde ayarlanıyor
            self._update_video_button_state(streaming=False) # Bağlantı sırasında buton pasif
        elif "Stopped" in status:
             self.stream_status_label.setStyleSheet("color: gray;")
             self.stream_button.setEnabled(True)
             self.stream_button.setText("Start Stream")
             self.video_display_label.setText("No video stream") # Durdurulduğunda ekranı temizle
             self.video_display_label.setPixmap(QPixmap()) # Pixmap'i temizle
             if self.video_thread and not self.video_thread.isRunning():
                  self.video_thread.wait()
                  self.video_thread = None
             self._update_video_button_state(streaming=False)


    def _update_video_button_state(self, streaming):
        """Video stream butonunun enabled/disabled durumunu ayarlar."""
        # Start/Stop butonu durumu update_stream_status içinde yönetiliyor
        pass # Şimdilik ekstra kontrol yok, duruma göre buton zaten ayarlanıyor


    def log_message(self, message):
        """Log alanına mesaj ekler."""
        # GUI thread'inde olduğundan emin ol
        if threading.current_thread() is threading.main_thread():
             self.log_text_edit.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        else:
             # Eğer başka bir thread'den çağrılırsa, GUI thread'ine sinyal gönder
             # Bu örnekte log_message_signal kullandığımız için bu else bloğuna gerek kalmıyor,
             # ama başka doğrudan erişimlerde sinyal kullanmak gerekir.
             # Şimdilik doğrudan append() kullanabiliriz çünkü log_message_signal zaten bunu sağlıyor.
             self.log_text_edit.append(f"[{time.strftime('%H:%M:%S')}] {message}")


    def closeEvent(self, event):
        """Pencere kapatıldığında çağrılır. Thread'leri güvenli bir şekilde durdurur."""
        self.log_message("Closing application...")
        if self.dronekit_thread and self.dronekit_thread.isRunning():
            self.log_message("Stopping Dronekit thread...")
            self.dronekit_thread.stop() # Durdurma sinyali ver
            self.dronekit_thread.wait() # Thread'in bitmesini bekle
            self.log_message("Dronekit thread stopped.")

        if self.video_thread and self.video_thread.isRunning():
            self.log_message("Stopping Video thread...")
            self.video_thread.stop() # Durdurma sinyali ver
            self.video_thread.wait() # Thread'in bitmesini bekle
            self.log_message("Video thread stopped.")

        self.log_message("Application closed.")
        event.accept() # Pencereyi kapatma olayını kabul et


if __name__ == "__main__":
    # OpenCV'nin GUI ile ilgili sorun yaşamaması için başlatmadan önce
    # cv2.startWindowThread() veya cv2.destroyAllWindows() gibi şeyleri çağırmamaya dikkat edin.
    # PyQt ile kullandığımızda OpenCV'nin kendi GUI döngüsüne ihtiyacı olmaz.

    app = QApplication(sys.argv)
    main_window = GCSWindow()
    main_window.show()
    sys.exit(app.exec_())
