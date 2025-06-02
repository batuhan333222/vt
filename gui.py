# vtol_gcs_full.py (Ana GCS kodu)

import sys
import time
import threading
import cv2
import numpy as np
import io # Görev dosyası okumak için

from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QPushButton, QLineEdit, QTextEdit,
                             QMessageBox, QGridLayout, QSizePolicy, QListWidget,
                             QListWidgetItem, QFileDialog, QDialog, QFormLayout,
                             QDoubleSpinBox, QSpinBox, QDialogButtonBox, QAction)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap, QIcon

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
from pymavlink import mavutil

# Harita Widget'ını import et
# Eğer map_widget.py olarak ayrı kaydettiyseniz:
# from map_widget import MapWidget
# Eğer aynı dosyada tanımladıysanız:
# (Üstteki MapWidget sınıfı tanımlandı varsayılıyor)

# ArduPilot'un VTOL modları listesi (yaygın olanlar)
VTOL_MODES = ["AUTO", "GUIDED", "LOITER", "QHOVER", "QLAND", "QSTABILIZE"] # Q* modları VTOL'a özgüdür

class DronekitThread(QThread):
    """
    Dronekit bağlantısını yöneten ve araç verilerini alan arka plan iş parçacığı.
    """
    # ... (Önceki DronekitThread kodu aynı kalabilir) ...
    connection_status_signal = pyqtSignal(str) # Bağlantı durumu için sinyal
    vehicle_data_signal = pyqtSignal(dict)     # Araç verileri için sinyal (konum, mod, arm durumu vb.)
    log_message_signal = pyqtSignal(str)      # Log mesajları için sinyal
    mission_downloaded_signal = pyqtSignal(list) # Görev indirildiğinde

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
            # wait_ready=True, timeout=60
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
            self.connection_status_signal.emit("Connected!")
            self.log_message_signal.emit("Vehicle connected successfully.")

            # Bağlantı kalitesini kontrol için system_status.state kullanılabilir
            # Diğer bağlantı kalitesi metrikleri (örneğin, MAVLink mesaj kayıpları)
            # daha alt seviye pymavlink kullanımı gerektirebilir.
            # self.log_message_signal.emit(f"System Status: {self.vehicle.system_status.state}")


            # Bağlantı kurulduktan sonra araç parametrelerini okumak veya görev indirmek gibi
            # işlemler burada veya ayrı metodlarla yapılabilir.
            # Örnek: Otomatik görev indirme (isteğe bağlı)
            # self.download_mission()


            # Veri alma döngüsü
            while not self._stop_event.is_set():
                if self.vehicle and self.vehicle.is_connected: # Bağlantının hala aktif olduğundan emin ol
                    try:
                        # Araç verilerini al
                        # GPS fix gelene kadar konumun None veya 0,0 olabileceğini unutmayın.
                        location = self.vehicle.location.global_frame
                        mode = self.vehicle.mode.name
                        is_armable = self.vehicle.is_armable
                        armed = self.vehicle.armed
                        system_status = self.vehicle.system_status.state
                        gps_info = self.vehicle.gps_0 # GPS bilgisi (fix tipi, uydu sayısı vb.)
                        # battery = self.vehicle.battery # Pil bilgisini de alabilirsiniz


                        # Verileri ana GUI thread'ine sinyal ile gönder
                        self.vehicle_data_signal.emit({
                            "location": {"lat": location.lat, "lon": location.lon, "alt": location.alt} if location and location.lat is not None else None,
                            "mode": mode,
                            "is_armable": is_armable,
                            "armed": armed,
                            "system_status": system_status,
                            "gps_info": {"fix_type": gps_info.fix_type, "satellites_visible": gps_info.satellites_visible} if gps_info else None,
                            # "battery": {"voltage": battery.voltage, "current": battery.current, "level": battery.level} if battery else None
                        })

                        # Daha sık veri almak isterseniz buradaki sleep süresini kısaltın
                        time.sleep(0.2) # 200 ms bekle (5 Hz güncelleme)

                    except APIException as e:
                        # Belirli API hatalarını logla (örn: vehicle.location erişiminde hata)
                        self.log_message_signal.emit(f"API Exception during data fetch: {e}")
                    except Exception as e:
                         # Daha genel hatalar (örn: bağlantı kesilmesi)
                         self.log_message_signal.emit(f"Error during data fetch, disconnecting: {e}")
                         # Hata durumunda bağlantıyı kesip thread'i durdurabiliriz
                         self.stop() # Güvenli çıkış için stop'u çağır
                         break # Döngüyü kır

                else:
                     # Vehicle nesnesi yoksa veya bağlantı kesildiyse, döngüyü kır
                     self.log_message_signal.emit("Vehicle connection lost or not established.")
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
                # Eğer vehicle nesnesi hala varsa ve bağlantı açıksa kapatmayı dene
                if self.vehicle.is_connected:
                    self.vehicle.close()
                    self.log_message_signal.emit("Vehicle connection closed.")
            except Exception as e:
                 self.log_message_signal.emit(f"Error closing vehicle connection: {e}")
            self.vehicle = None # Referansı temizle

        self.connection_status_signal.emit("Disconnected")


    def stop(self):
        """
        Thread'in çalışma döngüsünü durdurmak için kullanılır.
        """
        self._stop_event.set()
        # vehicle.close() run metodu sonunda çağrılıyor, thread'in bitmesi beklenmeli.

    # --- Araç Kontrol Metodları ---
    # Bu metodlar GUI thread'inden çağrılır ve DronekitThread'in kendi bağlamında çalışır.

    def arm_vehicle(self):
        # ... (Önceki arm_vehicle kodu aynı) ...
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
        # ... (Önceki disarm_vehicle kodu aynı) ...
        if self.vehicle and self.vehicle.armed:
            self.log_message_signal.emit("Disarming vehicle...")
            try:
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
         # ... (Önceki set_mode kodu aynı) ...
        if self.vehicle:
            try:
                mode_name = mode_name.upper() # Mod isimleri genellikle büyük harf
                if mode_name not in self.vehicle.mode.name: # Eğer zaten istenen modda değilse
                    self.log_message_signal.emit(f"Attempting to set mode to {mode_name}...")
                    self.vehicle.mode = VehicleMode(mode_name)
                    # Mod değişiminin tamamlanmasını beklemek için kısa bir gecikme eklenebilir
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

    # --- Görev Yönetimi Metodları ---
    # Bu metodlar GUI thread'inden çağrılır.

    def download_mission(self):
        """Araçtan görev komutlarını indirir."""
        if self.vehicle:
            self.log_message_signal.emit("Downloading mission from vehicle...")
            try:
                # Görev objesini al
                cmds = self.vehicle.commands
                cmds.download() # Komutları araçtan indir
                cmds.wait_ready() # İndirmenin tamamlanmasını bekle

                mission_list = []
                # İndirilen komutları listeye ekle
                for cmd in cmds:
                    mission_list.append(cmd)

                self.log_message_signal.emit(f"Downloaded {len(mission_list)} mission commands.")
                # İndirilen görevi GUI'ye sinyal ile gönder
                self.mission_downloaded_signal.emit(mission_list)

            except APIException as e:
                self.log_message_signal.emit(f"Mission download failed: {e}")
                self.mission_downloaded_signal.emit([]) # Hata durumunda boş liste gönder
            except Exception as e:
                 self.log_message_signal.emit(f"Mission download error: {e}")
                 self.mission_downloaded_signal.emit([]) # Hata durumunda boş liste gönder
        else:
            self.log_message_signal.emit("Cannot download mission: Vehicle not connected.")
            self.mission_downloaded_signal.emit([]) # Bağlantı yoksa boş liste gönder


    def upload_mission(self, mission_commands):
        """Araçtan görev komutlarını yükler."""
        if self.vehicle:
            self.log_message_signal.emit(f"Uploading {len(mission_commands)} mission commands to vehicle...")
            try:
                # Görev objesini al
                cmds = self.vehicle.commands
                cmds.clear() # Araçtaki mevcut görevi temizle
                cmds.upload() # Yükleme için hazırla (asenkron)

                # Yeni komutları ekle
                for command in mission_commands:
                    cmds.add(command)

                # Yüklemeyi tamamla
                cmds.upload()

                self.log_message_signal.emit("Mission upload command sent. Waiting for confirmation...")
                # Araçtan görevin başarılı yüklendiğine dair onay beklenmesi gerekebilir.
                # Dronekit'in varsayılan upload() metodu asenkrondur.
                # Belirli MAVLink mesajlarını dinleyerek onay mekanizması kurulabilir (daha gelişmiş).
                self.log_message_signal.emit("Mission upload process initiated.")


            except APIException as e:
                self.log_message_signal.emit(f"Mission upload failed: {e}")
            except Exception as e:
                 self.log_message_signal.emit(f"Mission upload error: {e}")
        else:
            self.log_message_signal.emit("Cannot upload mission: Vehicle not connected.")

    def set_current_mission_waypoint(self, wp_index):
        """Aracın mevcut görev noktasını ayarlar (görev çalışırken)."""
        if self.vehicle:
            self.log_message_signal.emit(f"Setting current waypoint to index {wp_index}...")
            try:
                self.vehicle.commands.next = wp_index # Next komutunu ayarla
                self.log_message_signal.emit(f"Current waypoint index set to {wp_index}.")
            except APIException as e:
                self.log_message_signal.emit(f"Setting current waypoint failed: {e}")
            except Exception as e:
                self.log_message_signal.emit(f"Setting current waypoint error: {e}")
        else:
             self.log_message_signal.emit("Cannot set current waypoint: Vehicle not connected.")

    def clear_mission_on_vehicle(self):
        """Araçtaki görevi temizler."""
        if self.vehicle:
            self.log_message_signal.emit("Clearing mission on vehicle...")
            try:
                cmds = self.vehicle.commands
                cmds.clear()
                cmds.upload() # Değişiklikleri araca yükle
                self.log_message_signal.emit("Mission clear command sent.")
            except APIException as e:
                self.log_message_signal.emit(f"Clearing mission failed: {e}")
            except Exception as e:
                 self.log_message_signal.emit(f"Clearing mission error: {e}")
        else:
             self.log_message_signal.emit("Cannot clear mission: Vehicle not connected.")


class MissionEditorDialog(QDialog):
    """
    Yeni görev komutu eklemek için basit diyalog penceresi.
    Şimdilik sadece WAYPOINT komutunu destekler.
    """
    def __init__(self, parent=None, default_lat=None, default_lon=None):
        super().__init__(parent)
        self.setWindowTitle("Add Mission Command (Waypoint)")
        self.setModal(True) # Diyalog açıkken ana pencereye erişimi engelle

        self.layout = QFormLayout(self)

        # mavutil.mavlink.MAV_CMD listesinden komut seçimi eklenebilir
        # Basitlik için sadece MAV_CMD_NAV_WAYPOINT varsayılıyor
        self.command_label = QLabel("Command:")
        self.command_value = QLabel("MAV_CMD_NAV_WAYPOINT") # Şimdilik sabit
        self.layout.addRow(self.command_label, self.command_value)

        # Parametre girişleri (WAYPOINT için p1-p4, x, y, z)
        # p1: hold time (s), p2: acceptance radius (m), p3: pass through (0/1), p4: yaw angle (deg)
        # x: latitude, y: longitude, z: altitude (AMSL veya Relative)

        self.param1_input = QDoubleSpinBox(self)
        self.param1_input.setRange(0, 1000)
        self.param1_input.setSingleStep(1.0)
        self.param1_input.setValue(0.0) # Varsayılan hold time 0
        self.layout.addRow("Param 1 (Hold Time):", self.param1_input)

        self.param2_input = QDoubleSpinBox(self)
        self.param2_input.setRange(0, 1000)
        self.param2_input.setSingleStep(0.1)
        self.param2_input.setValue(2.0) # Varsayılan acceptance radius 2m
        self.layout.addRow("Param 2 (Acceptance Radius):", self.param2_input)

        self.param3_input = QSpinBox(self)
        self.param3_input.setRange(0, 1)
        self.param3_input.setValue(0) # Varsayılan pass through 0 (durdur/geç)
        self.layout.addRow("Param 3 (Pass Through):", self.param3_input)

        self.param4_input = QDoubleSpinBox(self)
        self.param4_input.setRange(-180, 180)
        self.param4_input.setSingleStep(1.0)
        self.param4_input.setValue(0.0) # Varsayılan yaw 0
        self.layout.addRow("Param 4 (Yaw Angle):", self.param4_input)

        self.latitude_input = QDoubleSpinBox(self)
        self.latitude_input.setRange(-90, 90)
        self.latitude_input.setDecimals(6)
        self.latitude_input.setValue(default_lat if default_lat is not None else 0.0)
        self.layout.addRow("Latitude:", self.latitude_input)

        self.longitude_input = QDoubleSpinBox(self)
        self.longitude_input.setRange(-180, 180)
        self.longitude_input.setDecimals(6)
        self.longitude_input.setValue(default_lon if default_lon is not None else 0.0)
        self.layout.addRow("Longitude:", self.longitude_input)

        self.altitude_input = QDoubleSpinBox(self)
        self.altitude_input.setRange(0, 10000)
        self.altitude_input.setSingleStep(1.0)
        self.altitude_input.setValue(10.0) # Varsayılan yükseklik 10m
        self.layout.addRow("Altitude:", self.altitude_input)

        # Altitude tipi seçimi eklenebilir (AMSL, RELATIVE, FRAME_GLOBAL_RELATIVE_ALT)
        # Şimdilik sadece FRAME_GLOBAL_RELATIVE_ALT varsayılıyor.

        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        self.layout.addRow(self.button_box)

    def get_command(self):
        """Diyalogdan girilen bilgileri kullanarak bir Dronekit Command objesi oluşturur."""
        # Frame: mavutil.mavlink.MAV_FRAME (örneğin FRAME_GLOBAL_RELATIVE_ALT = 3)
        # Command: mavutil.mavlink.MAV_CMD (örneğin MAV_CMD_NAV_WAYPOINT = 16)
        # is_current: False (görev listesindeki geçerli komut değil)
        # autocontinue: True (bir sonraki komuta otomatik geç)

        # Güvenlik için bir başlangıç komutu (Genellikle HOME veya TAKE_OFF)
        # MAV_CMD_NAV_WAYPOINT = 16
        command = Command(
            0, 0, 0, # target_system, target_component (genellikle 0,0)
            mavutil.mavlink.MAV_FRAME.GLOBAL_RELATIVE_ALT, # FRAME_GLOBAL_RELATIVE_ALT = 3
            mavutil.mavlink.MAV_CMD.NAV_WAYPOINT, # MAV_CMD_NAV_WAYPOINT = 16
            0, # is_current (görev listesindeki index, ilk komut 1 olur)
            1, # autocontinue (True)
            self.param1_input.value(), # param1 (Hold Time)
            self.param2_input.value(), # param2 (Acceptance Radius)
            self.param3_input.value(), # param3 (Pass Through)
            self.param4_input.value(), # param4 (Yaw Angle)
            self.latitude_input.value(), # x (Latitude)
            self.longitude_input.value(), # y (Longitude)
            self.altitude_input.value() # z (Altitude)
        )
        return command


# VideoThread sınıfı aynı kalır, buraya tekrar eklemiyorum.
# class VideoThread(QThread): ... (Yukarıdaki koddan alın)


class GCSWindow(QMainWindow):
    """
    Ana yer kontrol istasyonu penceresi.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Basic VTOL GCS - Planning & Map")
        self.setGeometry(100, 100, 1200, 900) # Pencere boyutunu artır

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget) # Ana layout yatay olsun

        # Sol Panel: Bağlantı, Durum, Kontroller, Loglar, Görev Planlama
        self.left_panel_layout = QVBoxLayout()
        self.main_layout.addLayout(self.left_panel_layout, 1) # Sol panel 1 oranında yer kaplasın

        # Sağ Panel: Harita, Video
        self.right_panel_layout = QVBoxLayout()
        self.main_layout.addLayout(self.right_panel_layout, 3) # Sağ panel 3 oranında yer kaplasın (harita/video daha büyük)

        self.dronekit_thread = None
        self.video_thread = None

        self.mission_commands = [] # GUI tarafında tutulan görev komutları listesi

        self._create_connection_interface(self.left_panel_layout)
        self._create_status_display(self.left_panel_layout)
        self._create_control_buttons(self.left_panel_layout)
        self._create_mission_planning_interface(self.left_panel_layout) # Görev planlama arayüzü
        self._create_log_area(self.left_panel_layout)

        self._create_map_widget(self.right_panel_layout) # Harita widget'ı
        self._create_video_interface(self.right_panel_layout) # Video sağ panele


        self._update_button_states(connected=False, armed=False, armable=False)
        self._update_video_button_state(streaming=False)
        self._update_mission_planning_button_states(connected=False, mission_loaded=False)

    # --- UI Oluşturma Metodları ---

    def _create_connection_interface(self, parent_layout):
        # ... (Önceki create_connection_interface kodu aynı) ...
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
        # conn_group_layout.addStretch(1) # Boş alanı doldur

        parent_layout.addLayout(conn_group_layout)
        parent_layout.addSpacing(10)


    def _create_status_display(self, parent_layout):
        # ... (Önceki create_status_display kodu aynı) ...
        status_group_layout = QVBoxLayout()
        status_group_layout.addWidget(QLabel("<b>Vehicle Status</b>"))

        self.location_label = QLabel("Location: N/A")
        self.mode_label = QLabel("Mode: N/A")
        self.armable_label = QLabel("Armable: N/A")
        self.armed_label = QLabel("Armed: N/A")
        self.system_status_label = QLabel("System Status: N/A")
        self.gps_info_label = QLabel("GPS: N/A") # Yeni GPS bilgi etiketi

        status_group_layout.addWidget(self.location_label)
        status_group_layout.addWidget(self.mode_label)
        status_group_layout.addWidget(self.armable_label)
        status_group_layout.addWidget(self.armed_label)
        status_group_layout.addWidget(self.system_status_label)
        status_group_layout.addWidget(self.gps_info_label) # GPS etiketini ekle

        parent_layout.addLayout(status_group_layout)
        parent_layout.addSpacing(10)


    def _create_map_widget(self, parent_layout):
        """Harita widget'ını oluşturur ve layout'a ekler."""
        self.map_widget = MapWidget(self)
        self.map_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # Harita tıklama sinyalini görev ekleme slotuna bağla
        self.map_widget.map_clicked.connect(self.on_map_clicked)
        parent_layout.addWidget(QLabel("<b>Map</b>"))
        parent_layout.addWidget(self.map_widget)


    def _create_control_buttons(self, parent_layout):
        # ... (Önceki create_control_buttons kodu aynı) ...
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
            button.clicked.connect(lambda checked, mode=mode_name: self.on_mode_button_clicked(mode))
            mode_layout.addWidget(button, row, col)
            col += 1
            if col > 3: # Her satırda 4 buton olsun
                col = 0
                row += 1
        control_group_layout.addLayout(mode_layout)

        parent_layout.addLayout(control_group_layout)
        parent_layout.addSpacing(10)


    def _create_mission_planning_interface(self, parent_layout):
        """Görev planlama arayüzü widget'larını oluşturur."""
        mission_group_layout = QVBoxLayout()
        mission_group_layout.addWidget(QLabel("<b>Mission Planning</b>"))

        # Görev komutları listesi
        self.mission_list_widget = QListWidget(self)
        self.mission_list_widget.setMinimumHeight(150)
        # Liste öğesine çift tıklama ile düzenleme özelliği eklenebilir
        # self.mission_list_widget.itemDoubleClicked.connect(self.edit_mission_command)


        # Görev kontrol butonları
        mission_buttons_layout = QHBoxLayout()
        self.add_wp_button = QPushButton("Add Waypoint")
        self.add_wp_button.clicked.connect(self.add_mission_command)

        self.remove_wp_button = QPushButton("Remove Selected")
        self.remove_wp_button.clicked.connect(self.remove_selected_mission_command)

        self.clear_mission_button = QPushButton("Clear Local")
        self.clear_mission_button.clicked.connect(self.clear_local_mission)

        self.upload_mission_button = QPushButton("Upload to Vehicle")
        self.upload_mission_button.clicked.connect(self.upload_local_mission)

        self.download_mission_button = QPushButton("Download from Vehicle")
        self.download_mission_button.clicked.connect(self.download_vehicle_mission)

        # Dosyadan yükle/kaydet butonları (isteğe bağlı)
        # self.load_mission_file_button = QPushButton("Load from File")
        # self.load_mission_file_button.clicked.connect(self.load_mission_from_file)
        # self.save_mission_file_button = QPushButton("Save to File")
        # self.save_mission_file_button.clicked.connect(self.save_mission_to_file)


        mission_buttons_layout.addWidget(self.add_wp_button)
        mission_buttons_layout.addWidget(self.remove_wp_button)
        mission_buttons_layout.addWidget(self.clear_mission_button)
        mission_buttons_layout.addStretch(1) # Boşluk bırak
        mission_buttons_layout.addWidget(self.download_mission_button)
        mission_buttons_layout.addWidget(self.upload_mission_button)

        mission_group_layout.addWidget(self.mission_list_widget)
        mission_group_layout.addLayout(mission_buttons_layout)

        parent_layout.addLayout(mission_group_layout)
        parent_layout.addSpacing(10)


    def _create_video_interface(self, parent_layout):
        # ... (Önceki create_video_interface kodu aynı) ...
        video_group_layout = QVBoxLayout()
        video_group_layout.addWidget(QLabel("<b>Video Stream</b>"))

        video_input_layout = QHBoxLayout()
        self.rtsp_label = QLabel("RTSP URL:")
        self.rtsp_input = QLineEdit()
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
        self.video_display_label.setScaledContents(True)
        self.video_display_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_display_label.setMinimumSize(320, 240) # Minimum video boyutu

        self.stream_status_label = QLabel("Status: Idle")
        self.stream_status_label.setStyleSheet("color: gray;")

        video_group_layout.addLayout(video_input_layout)
        video_group_layout.addWidget(self.video_display_label)
        video_group_layout.addWidget(self.stream_status_label)

        parent_layout.addLayout(video_group_layout)
        parent_layout.addStretch(1) # Video alanının aşağıya doğru genişlemesini sağlar


    def _create_log_area(self, parent_layout):
        # ... (Önceki create_log_area kodu aynı) ...
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setFont(QFont("Courier", 9))
        self.log_text_edit.setMinimumHeight(100)

        parent_layout.addWidget(QLabel("<b>Log / Status Messages</b>"))
        parent_layout.addWidget(self.log_text_edit)
        parent_layout.addStretch(1) # Log alanının en altta kalmasını sağlar


    # --- Signal Slot Bağlantıları ve İşleyicileri ---

    def connect_vehicle(self):
       # ... (Önceki connect_vehicle kodu aynı) ...
       if self.dronekit_thread is not None and self.dronekit_thread.isRunning():
            # Bağlantı zaten aktif veya kuruluyor - disconnect işlemi yapalım
            self.disconnect_vehicle()
            return

       connection_string = self.conn_input.text()
       if not connection_string:
           self.log_message("Please enter a connection string.")
           return

       self.dronekit_thread = DronekitThread(connection_string)
       self.dronekit_thread.connection_status_signal.connect(self.update_connection_status)
       self.dronekit_thread.vehicle_data_signal.connect(self.update_vehicle_data)
       self.dronekit_thread.log_message_signal.connect(self.log_message)
       self.dronekit_thread.mission_downloaded_signal.connect(self.on_mission_downloaded) # Yeni bağlantı

       self.connect_button.setEnabled(False)
       self.connect_button.setText("Connecting...")
       self.connection_status_label.setStyleSheet("color: orange;") # Bağlantı kuruluyor rengi
       self.dronekit_thread.start()

    def disconnect_vehicle(self):
        # ... (Önceki disconnect_vehicle kodu aynı) ...
        if self.dronekit_thread and self.dronekit_thread.isRunning():
            self.log_message("Disconnecting vehicle...")
            self.dronekit_thread.stop()
            self.connect_button.setEnabled(False)
            self.connect_button.setText("Disconnecting...")
            # Thread'in durduğunu update_connection_status sinyali ile anlayacağız
        else:
            self.log_message("Vehicle not connected.")


    def update_connection_status(self, status):
        # ... (Önceki update_connection_status kodu aynı) ...
        self.connection_status_label.setText(f"Status: {status}")
        if "Connected" in status:
            self.connection_status_label.setStyleSheet("color: green;")
            self.connect_button.setEnabled(True)
            self.connect_button.setText("Disconnect")
            self._update_button_states(connected=True, armed=False, armable=False)
            self._update_mission_planning_button_states(connected=True, mission_loaded=len(self.mission_commands) > 0) # Bağlantı kuruldu, görev listesi durumuna göre ayarla
        else:
            self.connection_status_label.setStyleSheet("color: red;")
            self.connect_button.setEnabled(True)
            self.connect_button.setText("Connect")
            if self.dronekit_thread and not self.dronekit_thread.isRunning():
                 self.dronekit_thread.wait()
                 self.dronekit_thread = None
            self._update_button_states(connected=False, armed=False, armable=False)
            self._update_mission_planning_button_states(connected=False, mission_loaded=len(self.mission_commands) > 0) # Bağlantı kesildi, butonları devre dışı bırak


    def update_vehicle_data(self, data):
        """Dronekit thread'inden gelen araç verileri sinyalini işler."""
        location = data.get("location")
        mode = data.get("mode", "N/A")
        is_armable = data.get("is_armable", False)
        armed = data.get("armed", False)
        system_status = data.get("system_status", "N/A")
        gps_info = data.get("gps_info")
        # battery = data.get("battery")

        # Durum etiketlerini güncelle
        if location and location['lat'] is not None and location['lon'] is not None:
             self.location_label.setText(f"Location: Lat={location['lat']:.6f}, Lon={location['lon']:.6f}, Alt={location['alt']:.2f}m")
             # Harita üzerindeki araç marker'ını güncelle
             self.map_widget.update_vehicle_location(location['lat'], location['lon'])
        else:
            self.location_label.setText("Location: Waiting for GPS...")
            # Eğer GPS yoksa harita marker'ını temizleyebiliriz veya göstermeyebiliriz.
            # self.map_widget.update_vehicle_location(0, 0) # Veya belirgin olmayan bir yere taşı
            pass # Marker'ı olduğu yerde bırakmak da bir seçenek


        self.mode_label.setText(f"Mode: {mode}")
        self.armable_label.setText(f"Armable: {'Yes' if is_armable else 'No'}")
        self.armed_label.setText(f"Armed: {'Yes' if armed else 'No'}")
        self.system_status_label.setText(f"System Status: {system_status}")

        if gps_info:
             self.gps_info_label.setText(f"GPS: Fix Type={gps_info['fix_type']}, Satellites={gps_info['satellites_visible']}")
        else:
             self.gps_info_label.setText("GPS: N/A")


        # Kontrol ve Görev butonlarının durumunu güncelle
        self._update_button_states(connected=True, armed=armed, armable=is_armable)
        self._update_mission_planning_button_states(connected=True, mission_loaded=len(self.mission_commands) > 0)


    def _update_button_states(self, connected, armed, armable):
        # ... (Önceki _update_button_states kodu aynı) ...
        can_change_mode = connected and not armed
        for button in self.findChildren(QPushButton):
            if button.text() in VTOL_MODES:
                button.setEnabled(can_change_mode)

        self.arm_button.setEnabled(connected and armable and not armed)
        self.disarm_button.setEnabled(connected and armed)


    def _update_mission_planning_button_states(self, connected, mission_loaded):
        """Görev planlama butonlarının enabled/disabled durumlarını ayarlar."""
        # Görev ekleme/kaldırma/temizleme butonları sadece bağlıyken aktif olsun
        self.add_wp_button.setEnabled(connected)
        self.remove_wp_button.setEnabled(connected and mission_loaded) # Yüklü görev varsa kaldırabiliriz
        self.clear_mission_button.setEnabled(connected and mission_loaded) # Yüklü görev varsa temizleyebiliriz

        # Upload/Download butonları sadece bağlıyken aktif olsun
        self.upload_mission_button.setEnabled(connected and mission_loaded) # Yüklü görev varsa yükleyebiliriz
        self.download_mission_button.setEnabled(connected)


    # --- Görev Planlama İşleyicileri ---

    def on_map_clicked(self, lat, lon):
        """Harita tıklandığında çağrılır. Yeni görev noktası eklemek için diyalog açar."""
        self.log_message(f"Map clicked at: Lat={lat:.6f}, Lon={lon:.6f}")

        # Bağlı değilse görev noktası eklemeye izin verme
        if not (self.dronekit_thread and self.dronekit_thread.vehicle):
             self.log_message("Cannot add waypoint from map: Vehicle not connected.")
             return

        # Tıklanan konum için bir görev noktası ekleme diyaloğu aç
        dialog = MissionEditorDialog(self, default_lat=lat, default_lon=lon)
        if dialog.exec_(): # Diyalog modal olarak açılır ve kullanıcı OK'e basarsa True döner
            new_command = dialog.get_command()
            if new_command:
                self.mission_commands.append(new_command)
                self._update_mission_list_widget()
                self._update_mission_planning_button_states(connected=True, mission_loaded=True)
                self.log_message(f"Waypoint added at Lat={lat:.6f}, Lon={lon:.6f}, Alt={new_command.z:.2f}m")
                # Haritaya marker ekle (isteğe bağlı, liste widget'ı güncellenince de yapılabilir)
                # self.map_widget.add_mission_waypoint(lat, lon, len(self.mission_commands))

    def add_mission_command(self):
        """'Add Waypoint' butonuna basıldığında çağrılır. Diyalog açar."""
        # Bağlı değilse görev noktası eklemeye izin verme
        if not (self.dronekit_thread and self.dronekit_thread.vehicle):
             self.log_message("Cannot add waypoint: Vehicle not connected.")
             return

        # Varsayılan olarak aracın mevcut konumu kullanılabilir veya 0,0
        default_lat, default_lon = 0.0, 0.0
        if self.dronekit_thread and self.dronekit_thread.vehicle and self.dronekit_thread.vehicle.location.global_frame.lat is not None:
             loc = self.dronekit_thread.vehicle.location.global_frame
             default_lat, default_lon = loc.lat, loc.lon

        dialog = MissionEditorDialog(self, default_lat=default_lat, default_lon=default_lon)
        if dialog.exec_():
            new_command = dialog.get_command()
            if new_command:
                self.mission_commands.append(new_command)
                self._update_mission_list_widget()
                self._update_mission_planning_button_states(connected=True, mission_loaded=True)
                self.log_message(f"Waypoint added (dialog) at Lat={new_command.x:.6f}, Lon={new_command.y:.6f}, Alt={new_command.z:.2f}m")


    def remove_selected_mission_command(self):
        """'Remove Selected' butonuna basıldığında çağrılır."""
        selected_items = self.mission_list_widget.selectedItems()
        if not selected_items:
            self.log_message("No mission command selected to remove.")
            return

        # Seçili item'in indexini bul
        row = self.mission_list_widget.row(selected_items[0])
        if 0 <= row < len(self.mission_commands):
            removed_command = self.mission_commands.pop(row)
            self._update_mission_list_widget()
            self._update_mission_planning_button_states(connected=True, mission_loaded=len(self.mission_commands) > 0)
            self.log_message(f"Mission command at index {row} removed.")
            # Haritadan ilgili marker'ı da silmek gerekir (MapWidget sınıfına metod eklenmeli)
            # self.map_widget.remove_mission_marker(row) # Bu metodun MapWidget'ta tanımlanması ve index takibi yapması gerekir.

    def clear_local_mission(self):
        """'Clear Local' butonuna basıldığında çağrılır. GUI'deki görevi temizler."""
        reply = QMessageBox.question(self, 'Clear Mission', 'Are you sure you want to clear the local mission?',
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.mission_commands = []
            self._update_mission_list_widget()
            self._update_mission_planning_button_states(connected=True, mission_loaded=False)
            self.log_message("Local mission cleared.")
            self.map_widget.clear_mission_markers() # Haritadaki markerları da temizle


    def upload_local_mission(self):
        """'Upload to Vehicle' butonuna basıldığında çağrılır."""
        if not self.mission_commands:
            self.log_message("Local mission is empty. Nothing to upload.")
            return
        if not (self.dronekit_thread and self.dronekit_thread.vehicle):
             self.log_message("Cannot upload mission: Vehicle not connected.")
             return

        # Görevi yükleme komutunu Dronekit thread'ine gönder
        self.dronekit_thread.upload_mission(self.mission_commands)

    def download_vehicle_mission(self):
        """'Download from Vehicle' butonuna basıldığında çağrılır."""
        if not (self.dronekit_thread and self.dronekit_thread.vehicle):
             self.log_message("Cannot download mission: Vehicle not connected.")
             return

        # Görev indirme komutunu Dronekit thread'ine gönder
        self.dronekit_thread.download_mission()


    def on_mission_downloaded(self, mission_list):
        """Dronekit thread'inden indirilen görev geldiğinde çağrılır."""
        self.log_message("Mission download complete.")
        self.mission_commands = mission_list # İndirilen görevi yerel listeye ata
        self._update_mission_list_widget() # GUI listesini güncelle
        self._update_mission_planning_button_states(connected=True, mission_loaded=len(self.mission_commands) > 0)
        self.log_message(f"Updated local mission with {len(self.mission_commands)} commands from vehicle.")


    def _update_mission_list_widget(self):
        """GUI'deki görev listesi widget'ını günceller ve haritaya marker ekler."""
        self.mission_list_widget.clear() # Mevcut öğeleri temizle
        self.map_widget.clear_mission_markers() # Haritadaki eski markerları temizle

        # İlk komut genellikle HOME veya TAKE_OFF'tur ve index 0'dır.
        # Waypoint'ler genellikle 1'den başlar.
        # Dronekit cmds objesi 0. komutu otomatik ekleyebilir.

        if not self.mission_commands:
             self.log_message("Mission list is empty.")
             # clear_mission_markers already called
             return

        self.log_message(f"Displaying {len(self.mission_commands)} mission commands.")
        for i, command in enumerate(self.mission_commands):
            # Görev komutlarını liste widget'ında göster
            # Komut türünü ve önemli parametrelerini göster
            item_text = f"{i}: CMD={command.command} FRAME={command.frame} Lat={command.x:.6f} Lon={command.y:.6f} Alt={command.z:.2f}"
            self.mission_list_widget.addItem(item_text)

            # Eğer komut bir navigasyon komutuysa (örn: WAYPOINT), haritaya marker ekle
            # mavutil.mavlink.MAV_CMD.NAV_WAYPOINT = 16
            # Diğer navigasyon komutları da eklenebilir (NAV_TAKEOFF, NAV_LAND, NAV_LOITER vb.)
            if command.command in [mavutil.mavlink.MAV_CMD.NAV_WAYPOINT,
                                   mavutil.mavlink.MAV_CMD.NAV_TAKEOFF,
                                   mavutil.mavlink.MAV_CMD.NAV_LAND]:
                 # Konum geçerliyse haritaya marker ekle
                 if command.x != 0 and command.y != 0: # Basit kontrol
                      self.map_widget.add_mission_waypoint(command.x, command.y, i) # x=lat, y=lon for these commands

        self._update_mission_planning_button_states(connected=True, mission_loaded=True) # Liste doluysa butonları aktif et


    # --- Video Stream Signal Slot Bağlantıları ve İşleyicileri ---
    # ... (Önceki video işleyicileri aynı) ...

    def start_stop_stream(self):
        """Video stream başlat/durdur butonuna basıldığında çağrılır."""
        if self.video_thread is not None and self.video_thread.isRunning():
            self.stop_stream()
        else:
            rtsp_url = self.rtsp_input.text()
            if not rtsp_url or rtsp_url == "rtsp://<kameranın_ip_adresi>:<port>/<yayın_yolu>":
                self.log_message("Please enter a valid RTSP URL.")
                return

            self.video_thread = VideoThread(rtsp_url)
            self.video_thread.frame_signal.connect(self.update_video_frame)
            self.video_thread.stream_status_signal.connect(self.update_stream_status)
            self.video_thread.log_message_signal.connect(self.log_message)

            self.stream_button.setEnabled(False)
            self.stream_button.setText("Connecting...")
            self.stream_status_label.setStyleSheet("color: orange;")
            self.video_thread.start()


    def stop_stream(self):
        """Video stream durdurma işlemi."""
        if self.video_thread and self.video_thread.isRunning():
            self.log_message("Stopping video stream...")
            self.video_thread.stop()
            self.stream_button.setEnabled(False)
            self.stream_button.setText("Stopping...")
        else:
            self.log_message("Video stream is not active.")


    def update_video_frame(self, q_image):
        """Video thread'inden gelen QImage karesini gösterir."""
        pixmap = QPixmap.fromImage(q_image)
        self.video_display_label.setPixmap(pixmap)

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
             self.video_display_label.setText("No video stream")
             self.video_display_label.setPixmap(QPixmap())
             if self.video_thread and not self.video_thread.isRunning():
                  self.video_thread.wait()
                  self.video_thread = None
             self._update_video_button_state(streaming=False)
        elif "Connecting" in status:
            self.stream_status_label.setStyleSheet("color: orange;")
            self._update_video_button_state(streaming=False)
        elif "Stopped" in status:
             self.stream_status_label.setStyleSheet("color: gray;")
             self.stream_button.setEnabled(True)
             self.stream_button.setText("Start Stream")
             self.video_display_label.setText("No video stream")
             self.video_display_label.setPixmap(QPixmap())
             if self.video_thread and not self.video_thread.isRunning():
                  self.video_thread.wait()
                  self.video_thread = None
             self._update_video_button_state(streaming=False)

    def _update_video_button_state(self, streaming):
        pass # Durum update_stream_status içinde yönetiliyor


    def log_message(self, message):
        """Log alanına mesaj ekler."""
        if threading.current_thread() is threading.main_thread():
             self.log_text_edit.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        else:
             # Sinyal kullandığımız için bu satıra gerek yok, sinyal GUI thread'ine gönderir.
             pass


    def closeEvent(self, event):
        """Pencere kapatıldığında çağrılır. Thread'leri güvenli bir şekilde durdurur."""
        self.log_message("Closing application...")
        if self.dronekit_thread and self.dronekit_thread.isRunning():
            self.log_message("Stopping Dronekit thread...")
            self.dronekit_thread.stop()
            self.dronekit_thread.wait()
            self.log_message("Dronekit thread stopped.")

        if self.video_thread and self.video_thread.isRunning():
            self.log_message("Stopping Video thread...")
            self.video_thread.stop()
            self.video_thread.wait()
            self.log_message("Video thread stopped.")

        self.log_message("Application closed.")
        event.accept()


if __name__ == "__main__":
    # MapWidget sınıfını ana kod dosyasına eklediyseniz buraya tekrar import etmeye gerek yok.
    # Eğer ayrı bir dosyada (map_widget.py) ise, en başta import edilmesi gerekir.

    app = QApplication(sys.argv)
    main_window = GCSWindow()
    main_window.show()
    sys.exit(app.exec_())
