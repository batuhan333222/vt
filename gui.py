# vtol_gcs_full.py (Ana GCS kodu)

import sys
import time
import threading
import cv2
import numpy as np
import io
import os # map.html dosya yolu için

from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QPushButton, QLineEdit, QTextEdit,
                             QMessageBox, QGridLayout, QSizePolicy, QListWidget,
                             QListWidgetItem, QFileDialog, QDialog, QFormLayout,
                             QDoubleSpinBox, QSpinBox, QDialogButtonBox, QAction,
                             QGroupBox) # QGroupBox eksik import edilmişti
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer, QUrl # QUrl eksik import edilmişti
from PyQt5.QtGui import QFont, QImage, QPixmap, QIcon

# PyQtWebEngineWidgets ve Folium için import
from PyQt5.QtWebEngineWidgets import QWebEngineView # WebEngineView eksik import edilmişti
import folium # Folium eksik import edilmişti

# Dronekit ve Pymavlink
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
    from pymavlink import mavutil
    DRONEKIT_INSTALLED = True
except ImportError:
    print("DroneKit veya Folium kütüphanesi kurulu değil.")
    print("Lütfen 'pip install dronekit folium PyQt5 PyQtWebEngine opencv-python' çalıştırın.")
    DRONEKIT_INSTALLED = False

# ArduPilot'un VTOL modları listesi (yaygın olanlar)
VTOL_MODES = ["AUTO", "GUIDED", "LOITER", "QLOITER", "QHOVER", "QLAND", "QSTABILIZE", "RTL", "LAND", "TAKEOFF"] # Daha kapsamlı liste

# --- Placeholder/Basit Implementasyon Sınıfları ---

class MapWidget(QWidget):
    """
    Harita görselleştirme widget'ı.
    Folium ile HTML harita oluşturup QWebEngineView içinde gösterir.
    Marker ekleme ve konum güncelleme gibi temel harita fonksiyonlarını içerir.
    """
    # Harita tıklandığında gönderilecek sinyal (latitude, longitude)
    map_clicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.web_view = QWebEngineView(self)
        layout = QVBoxLayout(self)
        layout.addWidget(self.web_view)
        layout.setContentsMargins(0,0,0,0)

        self.map_file = os.path.join(os.path.dirname(__file__), "map.html") # Harita dosyası (betik ile aynı klasörde)
        self.vehicle_marker = None # Araç marker objesi (basit implementasyonda kullanılmıyor, yeniden çiziliyor)
        self.mission_markers = {} # Görev markerları {index: folium.Marker}

        self._init_map() # Başlangıç haritasını oluştur

        # Harita üzerinde tıklama olayını yakalamak için JavaScript enjekte etme
        self.web_view.loadFinished.connect(self._on_map_load_finished)


    def _init_map(self, center_lat=41.0082, center_lon=28.9784, zoom=13):
        """Başlangıç Folium haritasını oluşturur ve yükler."""
        if not DRONEKIT_INSTALLED:
             # Kütüphane kurulu değilse boş veya placeholder harita
             m = folium.Map(location=[center_lat, center_lon], zoom_start=zoom, tiles='OpenStreetMap')
        else:
            try:
                m = folium.Map(location=[center_lat, center_lon], zoom_start=zoom, tiles='OpenStreetMap')
                folium.LayerControl().add_to(m) # Layer kontrolü ekle
            except Exception as e:
                 print(f"MapWidget init_map error: {e}")
                 m = folium.Map(location=[center_lat, center_lon], zoom_start=zoom, tiles='OpenStreetMap')


        # Haritayı HTML dosyasına kaydet
        try:
            m.save(self.map_file)
            # QWebEngineView'da dosyayı yükle
            self.web_view.setUrl(QUrl.fromLocalFile(os.path.abspath(self.map_file)))
        except Exception as e:
             print(f"MapWidget file save/load error: {e}")


    def _on_map_load_finished(self, ok):
        """Harita yüklendiğinde çalışır. JavaScript ekler."""
        if ok:
            # Harita tıklandığında Python tarafına sinyal gönderecek JavaScript kodu
            # Dikkat: Bu basit bir örnektir. Detaylı marker/polygon etkileşimleri için daha karmaşık JS gerekir.
            js_code = """
            map.on('click', function(e) {
                var lat = e.latlng.lat;
                var lon = e.latlng.lng;
                // Python slotunu çağırmak için QWebChannel veya özel bir JavaScript hook kullanılabilir.
                // Basitlik için burada sadece bir console.log yapalım ve Python tarafından yakalamaya çalışmayalım.
                // Gerçek bir GCS için QWebChannel önerilir.
                console.log('Map clicked at: ' + lat + ', ' + lon);

                // QWebChannel kurulu ise:
                // if (typeof qtwebchannel !== 'undefined') {
                //     new QWebChannel(qt.webChannelTransport, function(channel) {
                //         window.py_backend = channel.objects.py_backend;
                //         py_backend.mapClicked(lat, lon); // Python slotunu çağır
                //     });
                // }
                // QWebChannel olmadan doğrudan signal emit etme bu şekilde mümkün değil.
                // Alternatif: Python tarafından periyodik olarak haritadaki tıklama bilgisini kontrol etme (karmaşık)
                // En basit yol: Tıklama koordinatını JS değişkeninde tut, Python ihtiyaç duyduğunda JS çalıştırarak oku.
                // Şimdilik sadece sinyali doğrudan emit ediyormuş gibi placeholder bırakalım.
                // Gerçek implementasyonda QWebChannel gerekir veya map_clicked sinyalini bu noktada doğrudan emit etmek yerine,
                // Python'dan çalıştırılan JS ile tıklama koordinatını alıp sonra emit etmek gerekir.
            });
            """
            # Şu anki basit yapıda JS'den Python sinyaline doğrudan geçiş yok.
            # map_clicked sinyalini kullanmak için QWebChannel kurmak gereklidir.
            # Örnekte, map_clicked sinyalini manuel olarak tetiklemek veya
            # on_map_clicked slotunu doğrudan çağırmak gerekir.
            # Tıklama koordinatlarını alıp sinyali emit etmek için QWebChannel en iyi yoldur.
            # QWebChannel kurulumu bu örneğin kapsamı dışında olduğundan, şimdilik
            # map_clicked sinyalini doğrudan harita tıklamasından otomatik çağırmak
            # bu placeholder MapWidget ile mümkün olmayacaktır.
            # Manuel ekleme fonksiyonu kullanılacak.

    def update_vehicle_location(self, lat, lon):
        """Araç marker'ının konumunu haritada günceller."""
        if not DRONEKIT_INSTALLED: return

        # Folium HTML'ini yeniden oluşturarak marker'ı güncellemek (basit ama yavaş yöntem)
        # Daha iyi yöntem: JavaScript ile sadece marker'ı hareket ettirmek.
        try:
            # Mevcut harita ve markerları Folium objesine yeniden yükle
            # Bu kısım, harita durumunu (zoom, center, diğer markerlar) korumak için karmaşıktır.
            # En basit yol, tüm haritayı yeniden çizmektir:
            # Mevcut zoom seviyesini korumaya çalış
            current_zoom = 13
            try:
                 if self.web_view.page():
                    zoom_result = self.web_view.page().runJavaScript("map.getZoom();").results()
                    if zoom_result is not None:
                         current_zoom = int(zoom_result)
            except Exception: pass # Hata olursa varsayılan zoom kullan

            # Mevcut merkez konumunu korumaya çalış
            current_center_lat, current_center_lon = lat, lon # Aracın konumunu merkez yap
            try:
                 if self.web_view.page():
                    center_result = self.web_view.page().runJavaScript("map.getCenter();").results()
                    if center_result is not None:
                         current_center_lat, current_center_lon = center_result['lat'], center_result['lng']
            except Exception: pass # Hata olursa aracın konumunu kullan

            m = folium.Map(location=[current_center_lat, current_center_lon], zoom_start=current_zoom, tiles='OpenStreetMap')
            folium.LayerControl().add_to(m)

            # Araç marker'ını ekle
            folium.Marker([lat, lon], popup="Vehicle").add_to(m)

            # Mevcut görev markerlarını da yeniden ekle
            for index, command in self.mission_markers.items():
                 if command.x != 0 and command.y != 0:
                      folium.Marker([command.x, command.y], popup=f"WP {index}", icon=folium.Icon(color='blue', icon='info-sign')).add_to(m)


            m.save(self.map_file)
            self.web_view.setUrl(QUrl.fromLocalFile(os.path.abspath(self.map_file)))

        except Exception as e:
             print(f"MapWidget update_vehicle_location error: {e}")


    def add_mission_waypoint(self, lat, lon, index):
        """Haritaya bir görev noktası marker'ı ekler."""
        if not DRONEKIT_INSTALLED: return
        # Note: Bu metod şu an sadece _update_mission_list_widget içinde kullanılıyor.
        # Markerlar her güncellemede yeniden çiziliyor. Daha iyi yöntem JS kullanmaktır.
        pass # Marker ekleme mantığı update_vehicle_location içinde yeniden çizimde.


    def clear_mission_markers(self):
        """Haritadaki tüm görev markerlarını temizler."""
        self.mission_markers = {} # Sadece referansları temizle. Yeniden çizim her şeyi siler.
        # update_vehicle_location çağrıldığında markerlar temizlenip yeniden eklenir.


    # Harita tıklamasını simüle etmek için, bu metodu GUI'den çağırabiliriz
    # veya QWebChannel kurup _on_map_load_finished içinde JS ile çağırmasını sağlayabiliriz.
    # Şimdilik, GUI'deki on_map_clicked slotu doğrudan harita tıklamasına bağlı değil.
    # Harita tıklamasını işlemek için QWebChannel kurulumu GEREKLİDİR.
    # Bu örnekte harita tıklamasından görev ekleme özelliği,
    # QWebChannel kurulumu yapılmadığı sürece ÇALIŞMAYACAKTIR.


class VideoThread(QThread):
    """
    RTSP video akışını almak ve kareleri işlemek için arka plan iş parçacığı.
    """
    frame_signal = pyqtSignal(QImage) # GUI'ye gönderilecek kare (QImage formatında)
    stream_status_signal = pyqtSignal(str) # Akış durumu için sinyal
    log_message_signal = pyqtSignal(str) # Log mesajları için sinyal

    def __init__(self, rtsp_url):
        super().__init__()
        self.rtsp_url = rtsp_url
        self._stop_event = threading.Event() # Thread'i durdurmak için Event
        self.cap = None # OpenCV VideoCapture objesi

    def run(self):
        """
        Video thread'inin ana çalışma fonksiyonu. RTSP akışını açar ve kareleri okur.
        """
        self.stream_status_signal.emit("Connecting...")
        self.log_message_signal.emit(f"Attempting to connect to RTSP stream: {self.rtsp_url}")

        # OpenCV VideoCapture ile akışı aç
        self.cap = cv2.VideoCapture(self.rtsp_url)

        if not self.cap.isOpened():
            self.stream_status_signal.emit("Connection Failed")
            self.log_message_signal.emit(f"Failed to open video stream from {self.rtsp_url}")
            self.cap = None # Cleanup
            return

        self.stream_status_signal.emit("Streaming...")
        self.log_message_signal.emit("Video stream connected successfully.")

        # Kare okuma döngüsü
        while not self._stop_event.is_set():
            ret, frame = self.cap.read() # Akıştan bir kare oku

            if not ret:
                # Kare okunamadı (akış kesildi veya bitti)
                self.log_message_signal.emit("Stream ended or error reading frame. Stopping...")
                self.stream_status_signal.emit("Disconnected")
                break # Döngüyü kır

            # OpenCV karesini (BGR formatında numpy dizisi) QImage'e dönüştür
            # RGB'ye çevir
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # QImage oluştur (format: QImage.Format_RGB888)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            q_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # QImage'i GUI thread'ine sinyal ile gönder
            self.frame_signal.emit(q_image)

            # Akış hızını kontrol etmek için gecikme eklenebilir, ancak genellikle gerekmez
            # çünkü read() metodu akış hızına uygun olarak bloklar.
            # time.sleep(0.01) # Çok kısa bir gecikme

        # Döngü bittiğinde veya thread durdurulduğunda
        if self.cap:
            self.cap.release() # Akışı serbest bırak
            self.cap = None
            self.log_message_signal.emit("Video stream released.")

        # stop() metodu çağrılmadan thread biterse (örn: akış hatası)
        if self._stop_event.is_set():
             self.stream_status_signal.emit("Stopped")
        else:
             # Akış hatası nedeniyle bittiyse
             self.stream_status_signal.emit("Stream Error")


    def stop(self):
        """
        Thread'in çalışma döngüsünü durdurmak için kullanılır.
        """
        self._stop_event.set()
        # VideoCapture'ı serbest bırakmak run metodu sonunda gerçekleşir.
        # Thread'in bitmesi için wait() GUI tarafında çağrılmalı.


# DronekitThread sınıfı (Verdiğiniz koddan alınmıştır, görev metotları eklenmiştir)
# Yukarıdaki "class DronekitThread(QThread):" bloğu buraya yerleştirilmiştir.
# Kod tekrarını önlemek için burada tam içeriği vermiyorum ama dosya oluştururken eklenmeli.
class DronekitThread(QThread):
    connection_status_signal = pyqtSignal(str)
    vehicle_data_signal = pyqtSignal(dict)
    log_message_signal = pyqtSignal(str)
    mission_downloaded_signal = pyqtSignal(list)

    def __init__(self, connection_string):
        super().__init__()
        self.connection_string = connection_string
        self.vehicle = None
        self._stop_event = threading.Event()

    def run(self):
        self.connection_status_signal.emit(f"Connecting to {self.connection_string}...")
        try:
            # wait_ready=True, timeout=60 (default)
            # If using serial, specify baud e.g., '/dev/ttyACM0,57600'
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
            if self._stop_event.is_set():
                 if self.vehicle: self.vehicle.close()
                 self.connection_status_signal.emit("Connection Cancelled")
                 self.log_message_signal.emit("Connection attempt cancelled.")
                 return

            self.connection_status_signal.emit("Connected")
            self.log_message_signal.emit("Vehicle connected successfully.")

            while not self._stop_event.is_set():
                if self.vehicle and self.vehicle.is_connected:
                    try:
                        location = self.vehicle.location.global_frame
                        mode = self.vehicle.mode.name
                        is_armable = self.vehicle.is_armable
                        armed = self.vehicle.armed
                        system_status = self.vehicle.system_status.state
                        gps_info = self.vehicle.gps_0

                        self.vehicle_data_signal.emit({
                            "location": {"lat": location.lat, "lon": location.lon, "alt": location.alt} if location and location.lat is not None else None,
                            "mode": mode,
                            "is_armable": is_armable,
                            "armed": armed,
                            "system_status": system_status,
                            "gps_info": {"fix_type": gps_info.fix_type, "satellites_visible": gps_info.satellites_visible} if gps_info else None,
                        })

                        time.sleep(0.2)

                    except APIException as e:
                        self.log_message_signal.emit(f"API Exception during data fetch: {e}")
                    except Exception as e:
                         self.log_message_signal.emit(f"Error during data fetch, disconnecting: {e}")
                         self.stop()
                         break

                else:
                     self.log_message_signal.emit("Vehicle connection lost or not established.")
                     break

        except APIException as e:
            self.connection_status_signal.emit("Connection Failed")
            self.log_message_signal.emit(f"Connection API Exception: {e}")
        except Exception as e:
            self.connection_status_signal.emit("Connection Failed")
            self.log_message_signal.emit(f"Connection Error: {e}")

        if self.vehicle:
            try:
                if self.vehicle.is_connected: # Check if connection is still active before closing
                    self.vehicle.close()
                    self.log_message_signal.emit("Vehicle connection closed.")
            except Exception as e:
                 self.log_message_signal.emit(f"Error closing vehicle connection: {e}")
            self.vehicle = None

        self.connection_status_signal.emit("Disconnected")

    def stop(self):
        self._stop_event.set()
        # Closing the vehicle connection upon thread completion in run() is usually sufficient.

    # --- Araç Kontrol Metodları (GUI thread'inden çağrılır, thread bağlamında çalışır) ---

    def arm_vehicle(self):
         if self.vehicle and self.vehicle.is_armable:
            if not self.vehicle.armed:
                self.log_message_signal.emit("Arming vehicle...")
                try:
                    # wait=False kullanmak GUI threadini dondurmaz. Durum güncellemeleri polling ile gelir.
                    # vehicle.arm() varsayılan olarak wait=True olabilir, bu yüzden açıkça belirtmek iyi.
                    self.vehicle.arm(True, wait=False)
                    self.log_message_signal.emit("Arming command sent. Waiting for confirmation...")
                except APIException as e:
                    self.log_message_signal.emit(f"Arming failed: {e}")
                except Exception as e:
                     self.log_message_signal.emit(f"Arming error: {e}")
            else:
                self.log_message_signal.emit("Vehicle is already armed.")
         elif self.vehicle and not self.vehicle.is_armable:
              self.log_message_signal.emit("Vehicle not armable. Check pre-arm requirements (GPS, Calibrations etc.).")
         else:
            self.log_message_signal.emit("Cannot arm: Vehicle not connected.")


    def disarm_vehicle(self):
        if self.vehicle and self.vehicle.armed:
            self.log_message_signal.emit("Disarming vehicle...")
            try:
                # Setting armed = False is usually non-blocking and safe.
                self.vehicle.armed = False
                self.log_message_signal.emit("Disarming command sent. Waiting for confirmation...")
            except APIException as e:
                self.log_message_signal.emit(f"Disarming failed: {e}")
            except Exception as e:
                 self.log_message_signal.emit(f"Disarming error: {e}")
        elif self.vehicle and not self.vehicle.armed:
            self.log_message_signal.emit("Vehicle is already disarmed.")
        else:
            self.log_message_signal.emit("Cannot disarm: Vehicle not connected.")

    def set_mode(self, mode_name):
        if self.vehicle:
            try:
                mode_name = mode_name.upper() # Mod isimleri genellikle büyük harf
                if mode_name != self.vehicle.mode.name: # Eğer zaten istenen modda değilse
                    self.log_message_signal.emit(f"Attempting to set mode to {mode_name}...")
                    # Mod değiştirmek genellikle bir miktar sürer ve bloklayıcı olabilir.
                    # wait=False seçeneği mode atamasında genellikle yoktur.
                    # Bu işlem DronekitThread içinde olduğu için GUI bloklanmaz,
                    # ancak bu thread'in kendisi kısa süreli bloklanabilir.
                    self.vehicle.mode = VehicleMode(mode_name)
                    self.log_message_signal.emit(f"Mode change command sent for {mode_name}. Waiting for confirmation...")
                else:
                     self.log_message_signal.emit(f"Vehicle is already in {mode_name} mode.")
            except APIException as e:
                self.log_message_signal.emit(f"Setting mode failed: {e}")
            except Exception as e:
                self.log_message_signal.emit(f"Setting mode error: {e}")
        else:
            self.log_message_signal.emit("Cannot set mode: Vehicle not connected.")

    # --- Görev Yönetimi Metodları (DronekitThread içinde çalışır) ---

    def download_mission(self):
        if self.vehicle:
            self.log_message_signal.emit("Downloading mission from vehicle...")
            try:
                cmds = self.vehicle.commands
                cmds.download() # Komutları araçtan indir
                cmds.wait_ready() # İndirmenin tamamlanmasını bekle

                mission_list = []
                # İndirilen komutları listeye ekle (0. komut genellikle HOME komutudur)
                for cmd in cmds:
                    mission_list.append(cmd)

                self.log_message_signal.emit(f"Downloaded {len(mission_list)} mission commands.")
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
                cmds = self.vehicle.commands
                cmds.clear() # Araçtaki mevcut görevi temizle
                cmds.upload() # Yükleme için hazırla (asenkron, gerekli olabilir)

                # Yeni komutları ekle
                for command in mission_commands:
                    cmds.add(command)

                # Yüklemeyi tamamla
                cmds.upload()

                self.log_message_signal.emit("Mission upload command sent.")
                # NOT: Başarılı yükleme onayı MAVLink mesajları dinlenerek yapılabilir.
                # Dronekit upload() metodunun başarılı dönmesi sadece komutun gönderildiğini belirtir.


            except APIException as e:
                self.log_message_signal.emit(f"Mission upload failed: {e}")
            except Exception as e:
                 self.log_message_signal.emit(f"Mission upload error: {e}")
        else:
            self.log_message_signal.emit("Cannot upload mission: Vehicle not connected.")

    def set_current_mission_waypoint(self, wp_index):
        if self.vehicle:
            self.log_message_signal.emit(f"Setting current waypoint to index {wp_index}...")
            try:
                # Dikkat: Bu komut genellikle görev AUTO modunda çalışırken kullanılır.
                self.vehicle.commands.next = wp_index
                self.log_message_signal.emit(f"Current waypoint index set to {wp_index}.")
            except APIException as e:
                self.log_message_signal.emit(f"Setting current waypoint failed: {e}")
            except Exception as e:
                self.log_message_signal.emit(f"Setting current waypoint error: {e}")
        else:
             self.log_message_signal.emit("Cannot set current waypoint: Vehicle not connected.")

    def clear_mission_on_vehicle(self):
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


# MissionEditorDialog sınıfı (Verdiğiniz koddan alınmıştır)
# Yukarıdaki "class MissionEditorDialog(QDialog):" bloğu buraya yerleştirilmiştir.
# Kod tekrarını önlemek için burada tam içeriği vermiyorum ama dosya oluştururken eklenmeli.
class MissionEditorDialog(QDialog):
    def __init__(self, parent=None, default_lat=None, default_lon=None):
        super().__init__(parent)
        self.setWindowTitle("Add Mission Command (Waypoint)")
        self.setModal(True)

        self.layout = QFormLayout(self)

        self.command_label = QLabel("Command:")
        self.command_value = QLabel("MAV_CMD_NAV_WAYPOINT")
        self.layout.addRow(self.command_label, self.command_value)

        self.param1_input = QDoubleSpinBox(self)
        self.param1_input.setRange(0, 1000)
        self.param1_input.setSingleStep(1.0)
        self.param1_input.setValue(0.0)
        self.layout.addRow("Param 1 (Hold Time):", self.param1_input)

        self.param2_input = QDoubleSpinBox(self)
        self.param2_input.setRange(0, 1000)
        self.param2_input.setSingleStep(0.1)
        self.param2_input.setValue(2.0)
        self.layout.addRow("Param 2 (Acceptance Radius):", self.param2_input)

        self.param3_input = QSpinBox(self)
        self.param3_input.setRange(0, 1)
        self.param3_input.setValue(0)
        self.layout.addRow("Param 3 (Pass Through):", self.param3_input)

        self.param4_input = QDoubleSpinBox(self)
        self.param4_input.setRange(-180, 180)
        self.param4_input.setSingleStep(1.0)
        self.param4_input.setValue(0.0)
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
        self.altitude_input.setValue(10.0)
        self.layout.addRow("Altitude:", self.altitude_input)

        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        self.layout.addRow(self.button_box)

    def get_command(self):
        command = Command(
            0, 0, 0, # target_system, target_component
            mavutil.mavlink.MAV_FRAME.GLOBAL_RELATIVE_ALT, # FRAME_GLOBAL_RELATIVE_ALT = 3
            mavutil.mavlink.MAV_CMD.NAV_WAYPOINT, # MAV_CMD_NAV_WAYPOINT = 16
            0, # is_current (görev listesindeki index)
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


# GCSWindow sınıfı (Verdiğiniz koddan alınmıştır, eksik importlar ve metodlar tamamlanmıştır)
# Yukarıdaki "class GCSWindow(QMainWindow):" bloğu buraya yerleştirilmiştir.
# Kod tekrarını önlemek için burada tam içeriği vermiyorum ama dosya oluştururken eklenmeli.
class GCSWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Basic VTOL GCS - Planning & Map & Video") # Pencere başlığı güncellendi
        self.setGeometry(100, 100, 1200, 900)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)

        self.left_panel_layout = QVBoxLayout()
        self.main_layout.addLayout(self.left_panel_layout, 1)

        self.right_panel_layout = QVBoxLayout()
        self.main_layout.addLayout(self.right_panel_layout, 3)

        self.dronekit_thread = None
        self.video_thread = None

        self.mission_commands = []

        self._create_connection_interface(self.left_panel_layout)
        self._create_status_display(self.left_panel_layout)
        self._create_control_buttons(self.left_panel_layout)
        self._create_mission_planning_interface(self.left_panel_layout)
        self._create_log_area(self.left_panel_layout) # Log alanı en altta

        self._create_map_widget(self.right_panel_layout)
        self._create_video_interface(self.right_panel_layout)

        # Başlangıçta tüm butonları güncelle
        self._update_button_states(connected=False, armed=False, armable=False)
        self._update_video_button_state(streaming=False)
        self._update_mission_planning_button_states(connected=False, mission_loaded=False)

        if not DRONEKIT_INSTALLED:
            self.log_message("Warning: DroneKit or Folium is not installed. Connection and map functionality will be limited.")
            self.connect_button.setEnabled(False) # Bağlantı butonunu devre dışı bırak


    # --- UI Oluşturma Metodları (Verdiğiniz koddan alınmıştır) ---

    def _create_connection_interface(self, parent_layout):
        conn_group = QGroupBox("Bağlantı")
        conn_layout = QVBoxLayout(conn_group) # Layout'u gruba ata

        conn_layout.addWidget(QLabel("<b>Vehicle Connection</b>"))

        conn_input_layout = QHBoxLayout()
        self.conn_label = QLabel("Conn String:")
        self.conn_input = QLineEdit()
        self.conn_input.setText("udp:127.0.0.1:14550")
        self.conn_input.setPlaceholderText("E.g., udp:127.0.0.1:14550 or /dev/ttyACM0")

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_vehicle)

        conn_input_layout.addWidget(self.conn_label)
        conn_input_layout.addWidget(self.conn_input)
        conn_input_layout.addWidget(self.connect_button)

        self.connection_status_label = QLabel("Status: Disconnected")
        self.connection_status_label.setStyleSheet("color: red;")

        conn_layout.addLayout(conn_input_layout)
        conn_layout.addWidget(self.connection_status_label)

        parent_layout.addWidget(conn_group)
        # parent_layout.addSpacing(10) # Boşluklar GroupBox içinde de verilebilir


    def _create_status_display(self, parent_layout):
        status_group = QGroupBox("Vehicle Status")
        status_layout = QVBoxLayout(status_group)

        status_layout.addWidget(QLabel("<b>Vehicle Status</b>"))

        self.location_label = QLabel("Location: N/A")
        self.mode_label = QLabel("Mode: N/A")
        self.armable_label = QLabel("Armable: N/A")
        self.armed_label = QLabel("Armed: N/A")
        self.system_status_label = QLabel("System Status: N/A")
        self.gps_info_label = QLabel("GPS: N/A")

        status_layout.addWidget(self.location_label)
        status_layout.addWidget(self.mode_label)
        status_layout.addWidget(self.armable_label)
        status_layout.addWidget(self.armed_label)
        status_layout.addWidget(self.system_status_label)
        status_layout.addWidget(self.gps_info_label)

        parent_layout.addWidget(status_group)
        # parent_layout.addSpacing(10)


    def _create_map_widget(self, parent_layout):
        """Harita widget'ını oluşturur ve layout'a ekler."""
        # Harita widget'ı MapWidget sınıfından oluşturuluyor
        self.map_widget = MapWidget(self)
        self.map_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # HARİTA TIKLAMA ÖZELLİĞİ İÇİN QWebChannel GEREKLİDİR!
        # Bu örnekte harita tıklama sinyali doğrudan kullanılamayacaktır.
        # self.map_widget.map_clicked.connect(self.on_map_clicked) # Bağlantı şimdilik çalışmayacak

        parent_layout.addWidget(QLabel("<b>Map</b>"))
        parent_layout.addWidget(self.map_widget)


    def _create_control_buttons(self, parent_layout):
        control_group = QGroupBox("Kontroller")
        control_layout = QVBoxLayout(control_group)

        control_layout.addWidget(QLabel("<b>Vehicle Control</b>"))

        arm_disarm_layout = QHBoxLayout()
        self.arm_button = QPushButton("ARM")
        self.arm_button.setStyleSheet("background-color: lightgreen;")
        self.arm_button.clicked.connect(self.on_arm_button_clicked)

        self.disarm_button = QPushButton("DISARM")
        self.disarm_button.setStyleSheet("background-color: salmon;")
        self.disarm_button.clicked.connect(self.on_disarm_button_clicked)

        arm_disarm_layout.addWidget(self.arm_button)
        arm_disarm_layout.addWidget(self.disarm_button)
        control_layout.addLayout(arm_disarm_layout)

        mode_layout = QGridLayout()
        mode_layout.addWidget(QLabel("<b>Modes:</b>"), 0, 0, 1, -1)
        row, col = 1, 0
        for mode_name in VTOL_MODES:
            button = QPushButton(mode_name)
            button.clicked.connect(lambda checked, mode=mode_name: self.on_mode_button_clicked(mode))
            mode_layout.addWidget(button, row, col)
            col += 1
            if col > 3:
                col = 0
                row += 1
        control_layout.addLayout(mode_layout)

        parent_layout.addWidget(control_group)
        # parent_layout.addSpacing(10)


    def _create_mission_planning_interface(self, parent_layout):
        mission_group = QGroupBox("Mission Planning")
        mission_layout = QVBoxLayout(mission_group)

        mission_layout.addWidget(QLabel("<b>Mission Commands</b>"))

        self.mission_list_widget = QListWidget(self)
        self.mission_list_widget.setMinimumHeight(150)
        # Çift tıklama ile düzenleme (gelecekte eklenebilir)
        # self.mission_list_widget.itemDoubleClicked.connect(self.edit_mission_command)

        # Görev kontrol butonları
        mission_buttons_layout = QHBoxLayout()
        # Map tıklaması şimdilik çalışmadığı için Add Waypoint butonu ana ekranda
        self.add_wp_button = QPushButton("Add Waypoint (Dialog)")
        self.add_wp_button.clicked.connect(self.add_mission_command_dialog) # Diyalog ile ekle fonksiyonuna bağla

        self.remove_wp_button = QPushButton("Remove Selected")
        self.remove_wp_button.clicked.connect(self.remove_selected_mission_command)

        self.clear_local_mission_button = QPushButton("Clear Local") # İsim güncellendi
        self.clear_local_mission_button.clicked.connect(self.clear_local_mission)

        self.upload_mission_button = QPushButton("Upload to Vehicle")
        self.upload_mission_button.clicked.connect(self.upload_local_mission)

        self.download_mission_button = QPushButton("Download from Vehicle")
        self.download_mission_button.clicked.connect(self.download_vehicle_mission)

        # Dosya butonları eklenmedi, gerekirse eklenebilir
        # self.load_mission_file_button = QPushButton("Load from File")
        # self.save_mission_file_button = QPushButton("Save to File")

        mission_buttons_layout.addWidget(self.add_wp_button)
        mission_buttons_layout.addWidget(self.remove_wp_button)
        mission_buttons_layout.addWidget(self.clear_local_mission_button)
        mission_buttons_layout.addStretch(1) # Boşluk bırak
        mission_buttons_layout.addWidget(self.download_mission_button)
        mission_buttons_layout.addWidget(self.upload_mission_button)

        mission_layout.addWidget(self.mission_list_widget)
        mission_layout.addLayout(mission_buttons_layout)

        parent_layout.addWidget(mission_group)
        # parent_layout.addSpacing(10)


    def _create_video_interface(self, parent_layout):
        video_group = QGroupBox("Video Stream")
        video_layout = QVBoxLayout(video_group)

        video_layout.addWidget(QLabel("<b>Video Stream</b>"))

        video_input_layout = QHBoxLayout()
        self.rtsp_label = QLabel("RTSP URL:")
        self.rtsp_input = QLineEdit()
        # Örnek URL'yi güncelleyin veya boş bırakın
        self.rtsp_input.setText("rtsp://<camera_ip>:<port>/<stream_path>")
        self.rtsp_input.setPlaceholderText("E.g., rtsp://192.168.1.100:554/stream1")

        self.stream_button = QPushButton("Start Stream")
        self.stream_button.clicked.connect(self.start_stop_stream)

        video_input_layout.addWidget(self.rtsp_label)
        video_input_layout.addWidget(self.rtsp_input)
        video_input_layout.addWidget(self.stream_button)

        self.video_display_label = QLabel("No video stream")
        self.video_display_label.setAlignment(Qt.AlignCenter)
        self.video_display_label.setStyleSheet("border: 1px solid gray; background-color: black; color: white;") # Yazı rengi eklendi
        self.video_display_label.setScaledContents(True)
        self.video_display_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_display_label.setMinimumSize(320, 240)

        self.stream_status_label = QLabel("Status: Idle")
        self.stream_status_label.setStyleSheet("color: gray;")

        video_layout.addLayout(video_input_layout)
        video_layout.addWidget(self.video_display_label)
        video_layout.addWidget(self.stream_status_label)

        parent_layout.addWidget(video_group)
        parent_layout.addStretch(1) # Video alanının aşağıya doğru genişlemesini sağlar


    def _create_log_area(self, parent_layout):
        log_group = QGroupBox("Log / Status Messages")
        log_layout = QVBoxLayout(log_group)

        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setFont(QFont("Courier", 9))
        self.log_text_edit.setMinimumHeight(100)

        log_layout.addWidget(self.log_text_edit)

        parent_layout.addWidget(log_group)
        parent_layout.addStretch(1) # Log alanının en altta kalmasını sağlar


    # --- Signal Slot Bağlantıları ve İşleyicileri ---

    def connect_vehicle(self):
       # Eğer thread zaten çalışıyorsa, connect butonu aslında disconnect görevi görür
       if self.dronekit_thread is not None and self.dronekit_thread.isRunning():
            self.disconnect_vehicle()
            return

       connection_string = self.conn_input.text()
       if not connection_string:
           self.log_message("Please enter a connection string.")
           return
       if not DRONEKIT_INSTALLED:
           self.log_message("DroneKit is not installed. Cannot connect.")
           return


       self.dronekit_thread = DronekitThread(connection_string)
       self.dronekit_thread.connection_status_signal.connect(self.update_connection_status)
       self.dronekit_thread.vehicle_data_signal.connect(self.update_vehicle_data)
       self.dronekit_thread.log_message_signal.connect(self.log_message)
       self.dronekit_thread.mission_downloaded_signal.connect(self.on_mission_downloaded)

       self.connect_button.setEnabled(False)
       self.connect_button.setText("Connecting...")
       self.connection_status_label.setStyleSheet("color: orange;")
       self.dronekit_thread.start()

    def disconnect_vehicle(self):
        if self.dronekit_thread and self.dronekit_thread.isRunning():
            self.log_message("Disconnecting vehicle...")
            self.dronekit_thread.stop()
            # Thread'in durmasını beklemek GUI'yi dondurabilir.
            # Daha iyi bir yaklaşım, thread durduğunda sinyal gönderip
            # UI güncellemesini o sinyalde yapmaktır. update_connection_status bunu yapıyor.
            # self.dronekit_thread.wait() # Bloklamamak için wait() burada çağrılmamalı.
            self.connect_button.setEnabled(False) # Tekrar basılmasını engelle
            self.connect_button.setText("Disconnecting...")

        elif self.dronekit_thread is not None: # Thread başlatılmış ama çalışmıyor olabilir (örn: hata verdi)
             self.log_message("Dronekit thread exists but is not running. Cleaning up.")
             if not self.dronekit_thread.isFinished():
                 self.dronekit_thread.wait()
             self.dronekit_thread = None
             # UI state'i update_connection_status içinde "Disconnected" olarak ayarlanacak.
             self.update_connection_status("Disconnected") # UI'ı hemen güncelle

        else:
            self.log_message("Vehicle not connected.")
            self.update_connection_status("Disconnected") # UI'ı "Disconnected" yap


    def update_connection_status(self, status):
        self.connection_status_label.setText(f"Status: {status}")
        if "Connected" in status:
            self.connection_status_label.setStyleSheet("color: green;")
            self.connect_button.setEnabled(True)
            self.connect_button.setText("Disconnect")
            # Bağlantı kurulduğunda kontrol butonlarını etkinleştir, armed/armable durumuna göre ayarla
            # vehicle_data_signal ilk data geldiğinde buton state'ini doğru ayarlar.
            # Şimdilik varsayılan durumları verelim:
            self._update_button_states(connected=True, armed=False, armable=False)
            self._update_mission_planning_button_states(connected=True, mission_loaded=len(self.mission_commands) > 0)

        elif "Connecting" in status:
             self.connection_status_label.setStyleSheet("color: orange;")
             self.connect_button.setEnabled(False) # Bağlantı devam ederken Connect/Disconnect basılamasın
             self.connect_button.setText("Connecting...")
             self._update_button_states(connected=False, armed=False, armable=False)
             self._update_mission_planning_button_states(connected=False, mission_loaded=False) # Bağlanırken görev butonları pasif


        else: # Disconnected, Connection Failed vb. durumlar
            self.connection_status_label.setStyleSheet("color: red;")
            self.connect_button.setEnabled(True)
            self.connect_button.setText("Connect")
            # Eğer thread hala varsa ama durduysa temizle
            if self.dronekit_thread and not self.dronekit_thread.isRunning():
                 # Thread'in sonlandığından emin olalım
                 if not self.dronekit_thread.isFinished():
                     self.dronekit_thread.wait()
                 self.dronekit_thread = None # Thread objesini serbest bırak

            # Bağlantı kesildiğinde tüm kontrol ve görev butonlarını devre dışı bırak
            self._update_button_states(connected=False, armed=False, armable=False)
            self._update_mission_planning_button_states(connected=False, mission_loaded=False)


    def update_vehicle_data(self, data):
        """Dronekit thread'inden gelen araç verileri sinyalini işler."""
        # GUI thread'inde çalışır.
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
             self.map_widget.update_vehicle_location(location['lat'], location['lon']) # MapWidget metodunu çağır
        else:
            self.location_label.setText("Location: Waiting for GPS...")
            # GPS yoksa harita marker'ını temizleyebiliriz veya görünmez yapabiliriz
            # self.map_widget.update_vehicle_location(0, 0) # Veya görünmez bir koordinata taşı


        self.mode_label.setText(f"Mode: {mode}")
        self.armable_label.setText(f"Armable: {'Yes' if is_armable else 'No'}")
        self.armed_label.setText(f"Armed: {'Yes' if armed else 'No'}")
        self.system_status_label.setText(f"System Status: {system_status}")

        if gps_info:
             self.gps_info_label.setText(f"GPS: Fix Type={gps_info['fix_type']}, Satellites={gps_info['satellites_visible']}")
             # GPS fix type > 0 ise konum geçerli sayılabilir
             # Fix Type 0=No GPS, 1=No Fix, 2=2D Fix, 3=3D Fix, 4=DGPS, 5=RTK Float, 6=RTK Fixed
             if gps_info['fix_type'] < 2: # 2D Fix veya daha iyisi yoksa
                  if location and location['lat'] is not None:
                      # Konum bilgisi gelmiş ama fix type düşük olabilir.
                      # DroneKit bazen fix type 0 olsa bile konum dönebilir.
                      pass # Konumu göster ama GPS durumunu belirt
                  else:
                      self.location_label.setText("Location: Waiting for GPS Fix...")

        else:
             self.gps_info_label.setText("GPS: N/A")


        # Kontrol ve Görev butonlarının durumunu güncelle
        self._update_button_states(connected=True, armed=armed, armable=is_armable)
        # Görev butonları bağlıyken ve yerel görev listesi doluysa aktif olmalı
        self._update_mission_planning_button_states(connected=True, mission_loaded=len(self.mission_commands) > 0)


    def _update_button_states(self, connected, armed, armable):
        """Kontrol butonlarının enabled/disabled durumlarını ayarlar."""
        # Mod değiştirme butonları sadece bağlıyken ve araç ARMED değilken aktif olsun
        can_change_mode = connected and not armed
        for button in self.findChildren(QPushButton):
            if button.text() in VTOL_MODES:
                button.setEnabled(can_change_mode)
                # Aktif modun butonunu görsel olarak işaretleme
                if connected and self.dronekit_thread and self.dronekit_thread.vehicle:
                     current_mode = str(self.dronekit_thread.vehicle.mode.name)
                     if button.text() == current_mode:
                          button.setStyleSheet("background-color: yellow;")
                     else:
                          button.setStyleSheet("") # Diğer mod butonlarını temizle

        # ARM butonu bağlıyken, ARMABLE ise ve ARMED değilken aktif olsun
        self.arm_button.setEnabled(connected and armable and not armed)
        # DISARM butonu bağlıyken ve ARMED ise aktif olsun
        self.disarm_button.setEnabled(connected and armed)

        # ARM/DISARM butonlarının renklerini durumlarına göre ayarla
        if connected:
             if armed:
                  self.arm_button.setStyleSheet("background-color: lightgreen;")
                  self.disarm_button.setStyleSheet("background-color: salmon;")
             else:
                  # Armed değilse, armable durumuna göre renk değişebilir
                  if armable:
                       self.arm_button.setStyleSheet("background-color: lightgreen;") # Arm edilebilir yeşil
                  else:
                       self.arm_button.setStyleSheet("background-color: lightgray;") # Arm edilemez gri
                  self.disarm_button.setStyleSheet("background-color: salmon;")
        else: # Bağlı değilse renkleri sıfırla ve devre dışı bırak
             self.arm_button.setStyleSheet("")
             self.disarm_button.setStyleSheet("")


    def _update_mission_planning_button_states(self, connected, mission_loaded):
        """Görev planlama butonlarının enabled/disabled durumlarını ayarlar."""
        # Görev ekleme/kaldırma/temizleme butonları
        self.add_wp_button.setEnabled(connected)
        self.remove_wp_button.setEnabled(connected and mission_loaded)
        self.clear_local_mission_button.setEnabled(connected and mission_loaded)

        # Upload/Download butonları
        self.upload_mission_button.setEnabled(connected and mission_loaded) # Yüklü görev varsa yükleyebiliriz
        self.download_mission_button.setEnabled(connected) # Bağlıyken her zaman indirebiliriz


    # --- Kontrol Butonu İşleyicileri ---
    # Bu fonksiyonlar GUI thread'inde çalışır, DronekitThread metotlarını çağırırlar.

    def on_arm_button_clicked(self):
         if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
            # Komutu Dronekit thread'ine gönder
            self.dronekit_thread.arm_vehicle()
         else:
            self.log_message("Cannot send ARM command: Vehicle not connected.")

    def on_disarm_button_clicked(self):
         if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
            # Komutu Dronekit thread'ine gönder
            self.dronekit_thread.disarm_vehicle()
         else:
            self.log_message("Cannot send DISARM command: Vehicle not connected.")

    def on_mode_button_clicked(self, mode_name):
        if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
             self.dronekit_thread.set_mode(mode_name)
        else:
            self.log_message(f"Cannot set mode to {mode_name}: Vehicle not connected.")


    # --- Görev Planlama İşleyicileri (GUI thread'inde çalışır) ---

    # @pyqtSlot(float, float) # QWebChannel kurulursa bu sinyal MapWidget'tan gelebilir
    def on_map_clicked(self, lat, lon):
        """Harita tıklandığında çağrılır (QWebChannel ile). Yeni görev noktası ekleme diyaloğu açar."""
        # NOT: Bu fonksiyon şu an QWebChannel kurulumu yapılmadığı için harita tıklaması ile otomatik ÇALIŞMAYACAKTIR.
        # Manuel olarak çağırırsanız çalışır.
        self.log_message(f"Map clicked at: Lat={lat:.6f}, Lon={lon:.6f}")

        if not (self.dronekit_thread and self.dronekit_thread.vehicle):
             self.log_message("Cannot add waypoint from map: Vehicle not connected.")
             return

        self.add_mission_command_dialog(lat, lon) # Diyalog açma fonksiyonunu çağır


    def add_mission_command_dialog(self, default_lat=None, default_lon=None):
        """'Add Waypoint' butonuna basıldığında veya harita tıklandığında çağrılır. Diyalog açar."""
        if not (self.dronekit_thread and self.dronekit_thread.vehicle):
             self.log_message("Cannot add waypoint: Vehicle not connected.")
             return

        # Eğer haritadan gelmiyorsa ve aracın konumu varsa, onu varsayılan yap
        if default_lat is None and self.dronekit_thread.vehicle.location.global_frame.lat is not None:
             loc = self.dronekit_thread.vehicle.location.global_frame
             default_lat, default_lon = loc.lat, loc.lon
        elif default_lat is None:
            default_lat, default_lon = 0.0, 0.0 # Bağlı ama konumu yoksa varsayılan 0,0


        dialog = MissionEditorDialog(self, default_lat=default_lat, default_lon=default_lon)
        if dialog.exec_(): # Diyalog modal olarak açılır ve kullanıcı OK'e basarsa True döner
            new_command = dialog.get_command()
            if new_command:
                # Komutu yerel listeye ekle (Genellikle index 0, HOME komutudur. Yeni WP'ler sona eklenir.)
                # Görev listesine eklerken sırasına dikkat etmek önemlidir.
                # Basitlik için şimdilik listenin sonuna ekleyelim.
                self.mission_commands.append(new_command)
                self._update_mission_list_widget() # GUI listesini ve haritayı güncelle
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
            self.log_message(f"Mission command at index {row} removed.")

    def clear_local_mission(self):
        """'Clear Local' butonuna basıldığında çağrılır. GUI'deki görevi temizler."""
        reply = QMessageBox.question(self, 'Clear Mission', 'Are you sure you want to clear the local mission?',
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.mission_commands = [] # Yerel listeyi temizle
            self._update_mission_list_widget() # GUI listesini ve haritayı güncelle
            self.log_message("Local mission cleared.")

    def upload_local_mission(self):
        """'Upload to Vehicle' butonuna basıldığında çağrılır."""
        if not self.mission_commands:
            self.log_message("Local mission is empty. Nothing to upload.")
            return
        if not (self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle):
             self.log_message("Cannot upload mission: Vehicle not connected.")
             return

        # Görevi yükleme komutunu Dronekit thread'ine gönder
        self.dronekit_thread.upload_mission(self.mission_commands)

    def download_vehicle_mission(self):
        """'Download from Vehicle' butonuna basıldığında çağrılır."""
        if not (self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle):
             self.log_message("Cannot download mission: Vehicle not connected.")
             return

        # Görev indirme komutunu Dronekit thread'ine gönder
        self.dronekit_thread.download_mission()


    @pyqtSlot(list)
    def on_mission_downloaded(self, mission_list):
        """Dronekit thread'inden indirilen görev geldiğinde çağrılır (GUI thread'inde çalışır)."""
        self.log_message("Mission download complete.")
        self.mission_commands = mission_list # İndirilen görevi yerel listeye ata
        self._update_mission_list_widget() # GUI listesini ve haritayı güncelle
        self.log_message(f"Updated local mission with {len(self.mission_commands)} commands from vehicle.")


    def _update_mission_list_widget(self):
        """GUI'deki görev listesi widget'ını günceller ve haritaya marker ekler."""
        self.mission_list_widget.clear() # Mevcut öğeleri temizle
        self.map_widget.clear_mission_markers() # Haritadaki eski görev markerlarını temizle
        self.map_widget.mission_markers = {} # MapWidget'taki referansları da temizle

        if not self.mission_commands:
             self.log_message("Mission list is empty.")
             self._update_mission_planning_button_states(connected=True, mission_loaded=False)
             # clear_mission_markers already called by map_widget
             return

        self.log_message(f"Displaying {len(self.mission_commands)} mission commands.")
        waypoints_to_draw = []
        for i, command in enumerate(self.mission_commands):
            # Görev komutlarını liste widget'ında göster
            # Komut türünü ve önemli parametrelerini göster
            item_text = f"{i}: CMD={command.command} FRAME={command.frame} p1={command.param1:.2f} p2={command.param2:.2f} p3={command.param3:.2f} p4={command.param4:.2f} x={command.x:.6f} y={command.y:.6f} z={command.z:.2f}"
            self.mission_list_widget.addItem(item_text)

            # Eğer komut bir navigasyon komutuysa (örn: WAYPOINT, TAKEOFF, LAND) ve konumu varsa, haritaya eklemek için listeye al
            if command.command in [mavutil.mavlink.MAV_CMD.NAV_WAYPOINT,
                                   mavutil.mavlink.MAV_CMD.NAV_TAKEOFF,
                                   mavutil.mavlink.MAV_CMD.NAV_LAND,
                                   mavutil.mavlink.MAV_CMD.NAV_LOITER_UNLIM,
                                   mavutil.mavlink.MAV_CMD.NAV_LOITER_TIME,
                                   mavutil.mavlink.MAV_CMD.NAV_LOITER_TO_ALT]:
                 # Konum geçerliyse (genellikle x=lat, y=lon)
                 if abs(command.x) > 0.000001 or abs(command.y) > 0.000001: # Basit sıfır kontrolü
                      waypoints_to_draw.append((command.x, command.y, i, command.command)) # lat, lon, index, command_type
                      # MapWidget'ın mission_markers'ına command objesini ekle
                      self.map_widget.mission_markers[i] = command


        # Haritaya waypoint markerlarını ve çizgilerini çizdir (MapWidget update_vehicle_location içinde zaten yapılıyor)
        # Sadece map_widget.mission_markers'ı güncelledik, harita güncelleme update_vehicle_data sinyaliyle tetiklenecek.
        # İstenirse burada da harita güncelleme çağrılabilir: self.map_widget.update_map_with_mission()
        # Ancak bu, araç konumu gelmeden önce görev markerlarını gösterir.
        # Şimdilik update_vehicle_data içindeki güncelleme yeterli.

        self._update_mission_planning_button_states(connected=True, mission_loaded=True) # Liste doluysa butonları aktif et


    # --- Video Stream İşleyicileri (GUI thread'inde çalışır) ---

    def start_stop_stream(self):
        """Video stream başlat/durdur butonuna basıldığında çağrılır."""
        if self.video_thread is not None and self.video_thread.isRunning():
            self.stop_stream()
        else:
            rtsp_url = self.rtsp_input.text()
            if not rtsp_url or rtsp_url == "rtsp://<camera_ip>:<port>/<stream_path>":
                self.log_message("Please enter a valid RTSP URL.")
                return

            # OpenCV kurulu mu kontrol et
            if 'cv2' not in sys.modules:
                 self.log_message("OpenCV (cv2) is not installed. Cannot start video stream.")
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
            # Thread'in durmasını beklemek GUI'yi dondurabilir.
            # update_stream_status sinyali thread durduğunda UI'ı günceller.
            self.stream_button.setEnabled(False)
            self.stream_button.setText("Stopping...")
        elif self.video_thread is not None: # Thread başlatılmış ama çalışmıyor olabilir
             self.log_message("Video thread exists but is not running. Cleaning up.")
             if not self.video_thread.isFinished():
                 self.video_thread.wait()
             self.video_thread = None
             self.update_stream_status("Stopped") # UI'ı hemen güncelle
        else:
            self.log_message("Video stream is not active.")
            self.update_stream_status("Stopped") # UI'ı "Stopped" yap


    @pyqtSlot(QImage)
    def update_video_frame(self, q_image):
        """Video thread'inden gelen QImage karesini gösterir (GUI thread'inde çalışır)."""
        pixmap = QPixmap.fromImage(q_image)
        # Video display label'ının boyutuna uydurmak için ölçeklendir
        scaled_pixmap = pixmap.scaled(self.video_display_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.video_display_label.setPixmap(scaled_pixmap)
        self.video_display_label.setText("") # Resim geldiğinde "No video stream" yazısını gizle

    @pyqtSlot(str)
    def update_stream_status(self, status):
        """Video thread'inden gelen stream durumu sinyalini işler (GUI thread'inde çalışır)."""
        self.stream_status_label.setText(f"Status: {status}")
        if "Streaming" in status:
            self.stream_status_label.setStyleSheet("color: green;")
            self.stream_button.setEnabled(True)
            self.stream_button.setText("Stop Stream")
            self._update_video_button_state(streaming=True)
        elif "Failed" in status or "Disconnected" in status or "Error" in status or "Stopped" in status:
             self.stream_status_label.setStyleSheet("color: red;")
             self.stream_button.setEnabled(True)
             self.stream_button.setText("Start Stream")
             self.video_display_label.setText("No video stream") # Hata/Kesilme durumunda placeholder yazısı
             self.video_display_label.setPixmap(QPixmap()) # Resmi temizle
             # Thread'i temizle (eğer kendiliğinden durduysa veya durdurma komutu tamamlandıysa)
             if self.video_thread and not self.video_thread.isRunning():
                  if not self.video_thread.isFinished():
                      self.video_thread.wait()
                  self.video_thread = None
             self._update_video_button_state(streaming=False)
        elif "Connecting" in status:
            self.stream_status_label.setStyleSheet("color: orange;")
            self.stream_button.setEnabled(False) # Bağlantı devam ederken buton pasif
            self._update_video_button_state(streaming=False)
        else: # Idle vs.
             self.stream_status_label.setStyleSheet("color: gray;")
             self.stream_button.setEnabled(True)
             self.stream_button.setText("Start Stream")
             self._update_video_button_state(streaming=False)


    def _update_video_button_state(self, streaming):
        # Bu metodun içi aslında update_stream_status içinde yönetiliyor.
        pass


    # --- Genel Yardımcı Metotlar ---
    @pyqtSlot(str)
    def log_message(self, message):
        """Log alanına zaman damgalı mesaj ekler (GUI thread'inde çalışır)."""
        # Sinyal/Slot mekanizması sayesinde bu metodun GUI thread'inde çalıştığı garanti edilir.
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.log_text_edit.append(f"[{timestamp}] {message}")
        # QTextEdit otomatik olarak en alta kaydırır.


    def closeEvent(self, event):
        """Pencere kapatıldığında çağrılır. Thread'leri güvenli bir şekilde durdurur."""
        self.log_message("Closing application...")

        # Dronekit thread'i durdur
        if self.dronekit_thread and self.dronekit_thread.isRunning():
            self.log_message("Stopping Dronekit thread...")
            self.dronekit_thread.stop()
            self.dronekit_thread.wait(5000) # 5 saniye bekle
            if self.dronekit_thread.isRunning():
                self.log_message("Dronekit thread did not stop gracefully.")
            else:
                 self.log_message("Dronekit thread stopped.")

        # Video thread'i durdur
        if self.video_thread and self.video_thread.isRunning():
            self.log_message("Stopping Video thread...")
            self.video_thread.stop()
            self.video_thread.wait(5000) # 5 saniye bekle
            if self.video_thread.isRunning():
                 self.log_message("Video thread did not stop gracefully.")
            else:
                 self.log_message("Video thread stopped.")


        self.log_message("Application closed.")
        event.accept() # Pencere kapatma olayına izin ver


# --- Uygulamanın Başlangıç Noktası ---
if __name__ == "__main__":
    # MapWidget, VideoThread ve DronekitThread sınıfları yukarıda tanımlanmıştır.
    # Artık tek bir dosya olarak çalıştırılabilir.

    app = QApplication(sys.argv)
    main_window = GCSWindow()
    main_window.show()
    sys.exit(app.exec_())
