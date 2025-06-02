# vtol_gcs_full_no_folium.py (Ana GCS kodu - Foliumsuz)

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
                             QGroupBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer, QUrl # QUrl, QThread eksik import edilmişti
from PyQt5.QtGui import QFont, QImage, QPixmap, QIcon

# PyQtWebEngineWidgets gerekli
try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    PYQTWEBENGINE_INSTALLED = True
except ImportError:
    print("PyQtWebEngine kütüphanesi kurulu değil.")
    print("Lütfen 'pip install PyQtWebEngine' komutunu çalıştırın.")
    PYQTWEBENGINE_INSTALLED = False


# Dronekit ve Pymavlink
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
    from pymavlink import mavutil
    DRONEKIT_INSTALLED = True
except ImportError:
    print("DroneKit kütüphanesi kurulu değil.")
    print("Lütfen 'pip install dronekit' komutunu çalıştırın.")
    DRONEKIT_INSTALLED = False

# OpenCV
try:
    import cv2
    OPENCV_INSTALLED = True
except ImportError:
    print("OpenCV (cv2) kütüphanesi kurulu değil.")
    print("Video akışı için 'pip install opencv-python' komutunu çalıştırın.")
    OPENCV_INSTALLED = False


# ArduPilot'un VTOL modları listesi (yaygın olanlar)
VTOL_MODES = ["AUTO", "GUIDED", "LOITER", "QLOITER", "QHOVER", "QLAND", "QSTABILIZE", "RTL", "LAND", "TAKEOFF"]

# --- Foliumsuz Harita Widget'ı ---

class MapWidget(QWidget):
    """
    Harita görselleştirme widget'ı.
    Basit HTML/JS Leaflet haritası oluşturup QWebEngineView içinde gösterir.
    Marker ekleme ve konum güncelleme gibi temel harita fonksiyonlarını içerir.
    """
    # Harita tıklandığında gönderilecek sinyal (latitude, longitude)
    # QWebChannel kurulumu olmadan bu sinyal harita tıklamasından otomatik gelmeyecektir.
    map_clicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.web_view = QWebEngineView(self)
        layout = QVBoxLayout(self)
        layout.addWidget(self.web_view)
        layout.setContentsMargins(0,0,0,0)

        self.map_file = os.path.join(os.path.dirname(__file__), "map.html") # Harita dosyası
        self._vehicle_lat = None
        self._vehicle_lon = None
        self._mission_waypoints = [] # [(lat, lon, index, command_type), ...]

        if PYQTWEBENGINE_INSTALLED:
             self._init_map() # Başlangıç haritasını oluştur
        else:
             placeholder_label = QLabel("Map requires PyQtWebEngine. Please install it.")
             placeholder_label.setAlignment(Qt.AlignCenter)
             layout.addWidget(placeholder_label)
             self.web_view.hide() # Web view'ı gizle

    def _init_map(self, center_lat=41.0082, center_lon=28.9784, zoom=13):
        """Başlangıç HTML haritasını oluşturur ve yükler."""
        if not PYQTWEBENGINE_INSTALLED: return

        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
        <title>Map</title>
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
        <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
        <style> #mapid {{ height: 100%; }} </style>
        </head>
        <body>
        <div id="mapid"></div>
        <script>
            var mymap = L.map('mapid').setView([{center_lat}, {center_lon}], {zoom});
            L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                maxZoom: 19,
                attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            }}).addTo(mymap);

            // Optional: Add a scale control
            L.control.scale().addTo(mymmap);

            // Store markers keyed by a unique ID (e.g., 'vehicle', 'wp_0', 'wp_1')
            var markers = {{}};

            // Function to add or update a marker
            window.updateMarker = function(id, lat, lon, popupText, iconUrl, iconSize) {{
                var iconOptions = null;
                if (iconUrl) {{
                    iconOptions = {{
                        icon: L.icon({{
                            iconUrl: iconUrl,
                            iconSize: iconSize || [25, 41], // default Leaflet icon size
                            iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
                            popupAnchor: [1, -34] // point from which the popup should open relative to the iconAnchor
                        }})
                    }};
                }}

                if (markers[id]) {{
                    markers[id].setLatLng([lat, lon]);
                    if(popupText) markers[id].setPopupContent(popupText);
                }} else {{
                    var marker = L.marker([lat, lon], iconOptions).addTo(mymap);
                     if(popupText) marker.bindPopup(popupText);
                    markers[id] = marker;
                }}
                 // Optionally open the popup immediately
                 // if(popupText) markers[id].openPopup();
            }};

             // Function to remove a marker
            window.removeMarker = function(id) {{
                if (markers[id]) {{
                    mymap.removeLayer(markers[id]);
                    delete markers[id];
                }}
            }};

            // Function to clear all dynamic markers (keeps base map layers)
            window.clearAllDynamicMarkers = function() {{
                 for (var id in markers) {{
                     if (markers.hasOwnProperty(id)) {{
                          mymap.removeLayer(markers[id]);
                     }}
                 }}
                 markers = {{}};
            }};

            // Basic click handling (requires QWebChannel to send data to Python)
            // mymap.on('click', function(e) {
            //    var lat = e.latlng.lat;
            //    var lon = e.latlng.lng;
            //    // Example using QWebChannel (requires setup):
            //    // if (window.py_backend && window.py_backend.mapClicked) {
            //    //     window.py_backend.mapClicked(lat, lon);
            //    // }
            //    console.log('Map clicked at: ' + lat + ', ' + lon);
            // });

        </script>
        </body>
        </html>
        """

        # Haritayı HTML dosyasına kaydet
        try:
            with open(self.map_file, "w") as f:
                f.write(html_content)
            # QWebEngineView'da dosyayı yükle
            self.web_view.setUrl(QUrl.fromLocalFile(os.path.abspath(self.map_file)))
        except Exception as e:
             print(f"MapWidget file save/load error: {e}")
             # Hata durumunda hata mesajı gösterilebilir
             error_label = QLabel(f"Error loading map file: {e}")
             error_label.setAlignment(Qt.AlignCenter)
             self.layout().addWidget(error_label)
             self.web_view.hide()


    def _redraw_map(self):
        """
        Mevcut araç konumu ve görev noktalarını kullanarak haritayı yeniden çizer.
        Her veri değiştiğinde haritayı yeniden yüklemek performanslı değildir,
        ancak QWebChannel kullanmadan marker güncellemenin en basit yoludur.
        """
        if not PYQTWEBENGINE_INSTALLED or not os.path.exists(self.map_file):
             # PyQtWebEngine yoksa veya harita dosyası oluşturulamadıysa çizme
             return

        # Mevcut zoom seviyesini korumaya çalış
        current_zoom = 13
        current_center_lat = 41.0082
        current_center_lon = 28.9784

        try:
            if self.web_view.page() and self.web_view.page().runJavaScript("typeof mymap !== 'undefined';").results():
                # Harita yüklendi ve mymap objesi varsa zoom ve center al
                zoom_result = self.web_view.page().runJavaScript("mymap.getZoom();").results()
                if zoom_result is not None:
                     current_zoom = int(zoom_result)

                center_result = self.web_view.page().runJavaScript("mymap.getCenter();").results()
                if center_result is not None:
                     current_center_lat, current_center_lon = center_result['lat'], center_result['lng']

        except Exception as e:
            # print(f"Error getting current map state: {e}") # Debugging
            pass # Hata olursa varsayılan değerler kullanılır

        # Araç konumu geçerliyse haritayı araç üzerinde merkezle
        if self._vehicle_lat is not None and self._vehicle_lon is not None:
             current_center_lat, current_center_lon = self._vehicle_lat, self._vehicle_lon
             # Eğer harita daha önce yüklenmediyse (zoom=13), ilk kez araç konumuna zoom yapabiliriz
             # if abs(current_center_lat - 41.0082) > 0.001 or abs(current_center_lon - 28.9784) > 0.001:
             #     current_zoom = max(current_zoom, 15) # Daha yakın zoom

        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
        <title>Map</title>
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
        <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
        <style> #mapid {{ height: 100%; }} </style>
        </head>
        <body>
        <div id="mapid"></div>
        <script>
            var mymap = L.map('mapid').setView([{current_center_lat}, {current_center_lon}], {current_zoom});
            L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                maxZoom: 19,
                attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            }}).addTo(mymap);

            // Optional: Add a scale control
            L.control.scale().addTo(mymap);

            // Add Markers
        """

        # Araç marker'ını ekle
        if self._vehicle_lat is not None and self._vehicle_lon is not None:
            html_content += f"""
            L.marker([{self._vehicle_lat}, {self._vehicle_lon}]).addTo(mymap)
                .bindPopup("Vehicle"); // Araç marker'ı

            mymap.setView([{self._vehicle_lat}, {self._vehicle_lon}]); // Aracı haritanın ortasına al
            """

        # Görev noktası markerlarını ekle
        for lat, lon, index, command_type in self._mission_waypoints:
            popup_text = f"WP {index}"
            # MAV_CMD.NAV_WAYPOINT = 16
            # MAV_CMD.NAV_TAKEOFF = 22
            # MAV_CMD.NAV_LAND = 21
            color = 'blue' # Varsayılan mavi
            if command_type == mavutil.mavlink.MAV_CMD.NAV_TAKEOFF:
                 popup_text = f"Takeoff {index}"
                 color = 'green'
            elif command_type == mavutil.mavlink.MAV_CMD.NAV_LAND:
                 popup_text = f"Land {index}"
                 color = 'red'


            html_content += f"""
            L.marker([{lat}, {lon}], {{icon: L.icon({{
                 iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-{color}.png',
                 shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
                 iconSize: [25, 41],
                 iconAnchor: [12, 41],
                 popupAnchor: [1, -34],
                 shadowSize: [41, 41]
            }})}}).addTo(mymap)
                .bindPopup("{popup_text}"); // Görev Noktası {index}
            """
            # Görev noktaları arasına çizgi çizebiliriz (isteğe bağlı)
            # Bu, tüm görev noktaları eklendikten sonra JS tarafında yapılmalıdır.

        # Görev noktaları arasına çizgi ekleme (basit bir örnek)
        if len(self._mission_waypoints) > 1:
             coords = [(lat, lon) for lat, lon, _, _ in self._mission_waypoints]
             # Çizgi için gerekli JS
             coords_js = str(coords).replace("(", "[").replace(")", "]") # [(lat, lon), ...] formatına çevir
             html_content += f"""
             var polyline = L.polyline({coords_js}, {{color: 'blue'}}).addTo(mymap);
             """


        html_content += """
        </script>
        </body>
        </html>
        """

        # Haritayı HTML dosyasına kaydet ve yeniden yükle
        try:
            with open(self.map_file, "w") as f:
                f.write(html_content)
            self.web_view.setUrl(QUrl.fromLocalFile(os.path.abspath(self.map_file)))
        except Exception as e:
             print(f"MapWidget redraw_map error: {e}")


    def update_vehicle_location(self, lat, lon):
        """Araç marker'ının konumunu günceller ve haritayı yeniden çizer."""
        if not PYQTWEBENGINE_INSTALLED: return
        self._vehicle_lat = lat
        self._vehicle_lon = lon
        self._redraw_map() # Konum değiştiğinde haritayı yeniden çiz


    def set_mission_waypoints(self, waypoints_data):
        """Görev noktası listesini günceller ve haritayı yeniden çizer."""
        if not PYQTWEBENGINE_INSTALLED: return
        self._mission_waypoints = waypoints_data
        self._redraw_map() # Görev noktaları değiştiğinde haritayı yeniden çiz


    def clear_mission_markers(self):
        """Haritadaki tüm görev markerlarını temizler (ve haritayı yeniden çizer)."""
        if not PYQTWEBENGINE_INSTALLED: return
        self._mission_waypoints = []
        self._redraw_map() # Görev noktaları temizlendiğinde haritayı yeniden çiz


# VideoThread sınıfı (Önceki koddan alınmıştır)
class VideoThread(QThread):
    frame_signal = pyqtSignal(QImage)
    stream_status_signal = pyqtSignal(str)
    log_message_signal = pyqtSignal(str)

    def __init__(self, rtsp_url):
        super().__init__()
        self.rtsp_url = rtsp_url
        self._stop_event = threading.Event()
        self.cap = None

    def run(self):
        if not OPENCV_INSTALLED:
            self.stream_status_signal.emit("Failed: OpenCV not installed")
            self.log_message_signal.emit("Cannot start video stream: OpenCV (cv2) is not installed.")
            return

        self.stream_status_signal.emit("Connecting...")
        self.log_message_signal.emit(f"Attempting to connect to RTSP stream: {self.rtsp_url}")

        # OpenCV VideoCapture ile akışı aç
        # Arduç kez denemek ve kısa bekleme eklemek daha sağlam olabilir
        max_attempts = 3
        attempt = 0
        while attempt < max_attempts and not self.cap or not self.cap.isOpened():
             if self.cap: self.cap.release() # Önceki denemeden kalmışsa temizle
             self.cap = cv2.VideoCapture(self.rtsp_url)
             if not self.cap.isOpened():
                 attempt += 1
                 self.log_message_signal.emit(f"Attempt {attempt} failed. Retrying in 2 seconds...")
                 time.sleep(2)

        if not self.cap or not self.cap.isOpened():
            self.stream_status_signal.emit("Connection Failed")
            self.log_message_signal.emit(f"Failed to open video stream from {self.rtsp_url} after {max_attempts} attempts.")
            self.cap = None
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
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            q_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # QImage'i GUI thread'ine sinyal ile gönder
            self.frame_signal.emit(q_image)

            # time.sleep(0.01) # Çok kısa bir gecikme (isteğe bağlı)

        # Döngü bittiğinde veya thread durdurulduğunda
        if self.cap:
            self.cap.release() # Akışı serbest bırak
            self.cap = None
            self.log_message_signal.emit("Video stream released.")

        if self._stop_event.is_set():
             self.stream_status_signal.emit("Stopped")
        else:
             self.stream_status_signal.emit("Stream Error")


    def stop(self):
        self._stop_event.set()


# DronekitThread sınıfı (Önceki koddan alınmıştır, Folium'a bağımlılığı yoktur)
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
        if not DRONEKIT_INSTALLED:
            self.connection_status_signal.emit("Connection Failed")
            self.log_message_signal.emit("Cannot connect: DroneKit is not installed.")
            return

        self.connection_status_signal.emit(f"Connecting to {self.connection_string}...")
        try:
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
            if self._stop_event.is_set():
                 if self.vehicle: self.vehicle.close()
                 self.connection_status_signal.emit("Connection Cancelled")
                 self.log_message_signal.emit("Connection attempt cancelled.")
                 return

            self.connection_status_signal.emit("Connected")
            self.log_message_signal.emit("Vehicle connected successfully.")

            # listener'ları ekle (GUI threadine sinyal gönderecekler)
            # Listener'lar DroneKit'in kendi thread havuzunda çalışır
            self.vehicle.add_attribute_listener('location.global_frame', self._location_callback)
            self.vehicle.add_attribute_listener('armed', self._armed_callback)
            self.vehicle.add_attribute_listener('mode', self._mode_callback)
            self.vehicle.add_attribute_listener('system_status', self._system_status_callback)
            self.vehicle.add_attribute_listener('gps_0', self._gps_info_callback)
            # self.vehicle.add_attribute_listener('battery', self._battery_callback)


            # İlk veriyi hemen al ve gönder
            self._send_current_vehicle_data()


            # Ana veri döngüsü (polling yapmak yerine listenerları kullanıyoruz)
            # Bu döngü sadece thread'in canlı kalmasını sağlar veya ek polling yapabilir.
            while not self._stop_event.is_set():
                if self.vehicle and self.vehicle.is_connected:
                    # İhtiyaç duyulursa burada ek polling veya komut takibi yapılabilir
                    # Şu an için listenerlar yeterli. Kısa bir bekleme koyalım.
                    time.sleep(0.5) # Daha yavaş polling veya sadece thread'in durmasını bekleme

                else:
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
                # Listener'ları kaldırmak iyi uygulama olabilir, ancak vehicle.close() da bunları serbest bırakır.
                # self.vehicle.remove_attribute_listener('location.global_frame', self._location_callback) # vb.
                if self.vehicle.is_connected: # Check if connection is still active before closing
                    self.vehicle.close()
                    self.log_message_signal.emit("Vehicle connection closed.")
            except Exception as e:
                 self.log_message_signal.emit(f"Error closing vehicle connection: {e}")
            self.vehicle = None

        self.connection_status_signal.emit("Disconnected")


    def stop(self):
        self._stop_event.set()
        # Thread'in run metodu bitene kadar beklemesi gereklidir (wait() GUI tarafında)

    # --- Listener Callback Fonksiyonları (Dronekit thread havuzunda çalışır) ---
    # Bu fonksiyonlar veri geldiğinde çağrılır ve GUI threadine sinyal gönderir.

    def _send_current_vehicle_data(self):
        """Mevcut araç verilerini toplayıp GUI'ye sinyal olarak gönderir."""
        if not self.vehicle: return

        try:
             location = self.vehicle.location.global_frame
             mode = self.vehicle.mode.name
             is_armable = self.vehicle.is_armable
             armed = self.vehicle.armed
             system_status = self.vehicle.system_status.state
             gps_info = self.vehicle.gps_0
             # battery = self.vehicle.battery # Pil bilgisi

             self.vehicle_data_signal.emit({
                 "location": {"lat": location.lat, "lon": location.lon, "alt": location.alt} if location and location.lat is not None else None,
                 "mode": mode,
                 "is_armable": is_armable,
                 "armed": armed,
                 "system_status": system_status,
                 "gps_info": {"fix_type": gps_info.fix_type, "satellites_visible": gps_info.satellites_visible} if gps_info else None,
                 # "battery": {"voltage": battery.voltage, "current": battery.current, "level": battery.level} if battery else None
             })
        except Exception as e:
             # Veri okuma sırasında hata oluşursa (örn: bağlantı koptu)
             # print(f"Error sending current vehicle data: {e}") # Debugging
             pass # Hata durumunda sinyal göndermeyiz veya None göndeririz.


    def _location_callback(self, vehicle, name, location):
        """location.global_frame güncellendiğinde çağrılır."""
        self._send_current_vehicle_data() # Tüm veriyi yeniden göndermek basit yol


    def _armed_callback(self, vehicle, name, armed):
         """armed durumu değiştiğinde çağrılır."""
         self._send_current_vehicle_data()


    def _mode_callback(self, vehicle, name, mode):
         """mode değiştiğinde çağrılır."""
         self._send_current_vehicle_data()

    def _system_status_callback(self, vehicle, name, system_status):
         """system_status değiştiğinde çağrılır."""
         self._send_current_vehicle_data()

    def _gps_info_callback(self, vehicle, name, gps_info):
         """gps_0 bilgisi değiştiğinde çağrılır."""
         self._send_current_vehicle_data()

    # def _battery_callback(self, vehicle, name, battery):
    #      """battery bilgisi değiştiğinde çağrılır."""
    #      self._send_current_vehicle_data()


    # --- Araç Kontrol Metodları (GUI thread'inden çağrılır, thread bağlamında çalışır) ---

    def arm_vehicle(self):
         if self.vehicle and self.vehicle.is_armable:
            if not self.vehicle.armed:
                self.log_message_signal.emit("Arming vehicle...")
                try:
                    # Pre-arm kontrollerini pas geçmek isterseniz force_arm=True kullanabilirsiniz (dikkatli olun)
                    self.vehicle.arm(True, wait=False) # wait=False GUI'yi dondurmaz
                    self.log_message_signal.emit("Arming command sent. Waiting for confirmation...")
                except APIException as e:
                    self.log_message_signal.emit(f"Arming failed: {e}")
                except Exception as e:
                     self.log_message_signal.emit(f"Arming error: {e}")
            else:
                self.log_message_signal.emit("Vehicle is already armed.")
         elif self.vehicle and not self.vehicle.is_armable:
              # is_armable False ise sebebini loglamak faydalı olabilir (örn: GPS fix yok)
              if self.vehicle.system_status.state != 'ACTIVE': # ACTIVE genellikle armable demektir
                   self.log_message_signal.emit(f"Vehicle not armable. System status: {self.vehicle.system_status.state}. Check pre-arm requirements (GPS, Calibrations etc.).")
              else:
                   self.log_message_signal.emit("Vehicle not armable. Check pre-arm requirements.")
         else:
            self.log_message_signal.emit("Cannot arm: Vehicle not connected.")


    def disarm_vehicle(self):
        if self.vehicle and self.vehicle.armed:
            self.log_message_signal.emit("Disarming vehicle...")
            try:
                self.vehicle.armed = False # wait=False gibi davranır genellikle
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
                mode_name = mode_name.upper()
                if mode_name != self.vehicle.mode.name:
                    self.log_message_signal.emit(f"Attempting to set mode to {mode_name}...")
                    # Mod değiştirmek genellikle bloklayıcıdır. DronekitThread içinde sorun olmaz.
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
                cmds.download()
                cmds.wait_ready()

                mission_list = []
                for cmd in cmds:
                    mission_list.append(cmd)

                self.log_message_signal.emit(f"Downloaded {len(mission_list)} mission commands.")
                self.mission_downloaded_signal.emit(mission_list)

            except APIException as e:
                self.log_message_signal.emit(f"Mission download failed: {e}")
                self.mission_downloaded_signal.emit([])
            except Exception as e:
                 self.log_message_signal.emit(f"Mission download error: {e}")
                 self.mission_downloaded_signal.emit([])
        else:
            self.log_message_signal.emit("Cannot download mission: Vehicle not connected.")
            self.mission_downloaded_signal.emit([])


    def upload_mission(self, mission_commands):
        if self.vehicle:
            self.log_message_signal.emit(f"Uploading {len(mission_commands)} mission commands to vehicle...")
            try:
                cmds = self.vehicle.commands
                cmds.clear()
                # Komutları ekle
                for command in mission_commands:
                    cmds.add(command)

                cmds.upload() # Upload'ı başlat
                self.log_message_signal.emit("Mission upload command sent.")
                # Başarılı upload onayı için ek MAVLink mesajı dinlenmesi gerekebilir.

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
                cmds.upload()
                self.log_message_signal.emit("Mission clear command sent.")
            except APIException as e:
                self.log_message_signal.emit(f"Clearing mission failed: {e}")
            except Exception as e:
                 self.log_message_signal.emit(f"Clearing mission error: {e}")
        else:
             self.log_message_signal.emit("Cannot clear mission: Vehicle not connected.")


# MissionEditorDialog sınıfı (Önceki koddan alınmıştır)
class MissionEditorDialog(QDialog):
    def __init__(self, parent=None, default_lat=None, default_lon=None):
        super().__init__(parent)
        self.setWindowTitle("Add Mission Command (Waypoint)")
        self.setModal(True)

        self.layout = QFormLayout(self)

        self.command_label = QLabel("Command:")
        # Farklı komut tipleri için ComboBox eklenebilir
        self.command_value = QLabel("MAV_CMD_NAV_WAYPOINT") # Şimdilik sabit
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
        self.latitude_input.setMinimumWidth(150) # Giriş alanını genişlet
        self.layout.addRow("Latitude:", self.latitude_input)

        self.longitude_input = QDoubleSpinBox(self)
        self.longitude_input.setRange(-180, 180)
        self.longitude_input.setDecimals(6)
        self.longitude_input.setValue(default_lon if default_lon is not None else 0.0)
        self.longitude_input.setMinimumWidth(150)
        self.layout.addRow("Longitude:", self.longitude_input)

        self.altitude_input = QDoubleSpinBox(self)
        self.altitude_input.setRange(-1000, 10000) # İrtifa negatif olabilir (deniz seviyesi altı)
        self.altitude_input.setSingleStep(1.0)
        self.altitude_input.setValue(10.0)
        self.layout.addRow("Altitude (m):", self.altitude_input)

        # Altitude tipi seçimi eklenebilir (AMSL, RELATIVE, FRAME_GLOBAL_RELATIVE_ALT)
        # Şimdilik sadece FRAME_GLOBAL_RELATIVE_ALT varsayılıyor (Mavlink frame 3)

        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        self.layout.addRow(self.button_box)

    def get_command(self):
        """Diyalogdan girilen bilgileri kullanarak bir Dronekit Command objesi oluşturur."""
        # MAV_CMD_NAV_WAYPOINT = 16
        # MAV_FRAME.GLOBAL_RELATIVE_ALT = 3
        command = Command(
            0, 0, 0, # target_system, target_component (genellikle 0,0)
            mavutil.mavlink.MAV_FRAME.GLOBAL_RELATIVE_ALT, # Frame (3)
            mavutil.mavlink.MAV_CMD.NAV_WAYPOINT, # Command (16)
            0, # is_current (bu komut yüklenirken mevcut komut olmayacak)
            1, # autocontinue (True)
            self.param1_input.value(), # param1 (Hold Time)
            self.param2_input.value(), # param2 (Acceptance Radius)
            self.param3_input.value(), # param3 (Pass Through)
            self.param4_input.value(), # param4 (Yaw Angle)
            self.latitude_input.value(), # x (Latitude for WAYPOINT)
            self.longitude_input.value(), # y (Longitude for WAYPOINT)
            self.altitude_input.value() # z (Altitude for WAYPOINT)
        )
        return command


# GCSWindow sınıfı (Önceki koddan alınmıştır, Folium bağımlılığı kaldırılmıştır)
class GCSWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Basic VTOL GCS - Map & Video & Mission")
        self.setGeometry(100, 100, 1200, 900)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)

        # Sol Panel (Kontroller, Durum, Bağlantı, Görev, Loglar)
        self.left_panel_layout = QVBoxLayout()
        self.left_panel_layout.setSpacing(10)
        self.left_panel_layout.setContentsMargins(5, 5, 5, 5) # Kenar boşlukları
        self.main_layout.addLayout(self.left_panel_layout, 1) # Sol panel 1 oranında yer kaplasın

        # Sağ Panel (Harita, Video)
        self.right_panel_layout = QVBoxLayout()
        self.right_panel_layout.setSpacing(10)
        self.right_panel_layout.setContentsMargins(5, 5, 5, 5) # Kenar boşlukları
        self.main_layout.addLayout(self.right_panel_layout, 3) # Sağ panel 3 oranında yer kaplasın


        self.dronekit_thread = None
        self.video_thread = None

        self.mission_commands = [] # GUI tarafında tutulan görev komutları listesi (Dronekit Command objeleri)

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

        # Kurulum uyarıları
        if not DRONEKIT_INSTALLED:
            self.log_message("Warning: DroneKit is not installed. Vehicle connection will not work.")
            self.connect_button.setEnabled(False)

        if not PYQTWEBENGINE_INSTALLED:
            self.log_message("Warning: PyQtWebEngine is not installed. Map will not display.")

        if not OPENCV_INSTALLED:
             self.log_message("Warning: OpenCV (cv2) is not installed. Video stream will not work.")


    # --- UI Oluşturma Metodları ---

    def _create_connection_interface(self, parent_layout):
        conn_group = QGroupBox("Bağlantı")
        conn_layout = QVBoxLayout(conn_group)

        conn_layout.addWidget(QLabel("<b>Vehicle Connection</b>"))

        conn_input_layout = QHBoxLayout()
        self.conn_label = QLabel("Conn String:")
        self.conn_input = QLineEdit()
        self.conn_input.setText("udp:127.0.0.1:14550")
        self.conn_input.setPlaceholderText("E.g., udp:127.0.0.1:14550 or /dev/ttyACM0")
        self_size_policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.conn_input.setSizePolicy(self_size_policy) # Giriş alanını genişlet

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
        # self.battery_label = QLabel("Battery: N/A") # Pil etiketi

        status_layout.addWidget(self.location_label)
        status_layout.addWidget(self.mode_label)
        status_layout.addWidget(self.armable_label)
        status_layout.addWidget(self.armed_label)
        status_layout.addWidget(self.system_status_label)
        status_layout.addWidget(self.gps_info_label)
        # status_layout.addWidget(self.battery_label)

        parent_layout.addWidget(status_group)


    def _create_map_widget(self, parent_layout):
        """Harita widget'ını oluşturur ve layout'a ekler."""
        self.map_widget = MapWidget(self)
        self.map_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # HARİTA TIKLAMA ÖZELLİĞİ ŞU AN İÇİN DESTEKLENMİYOR (QWebChannel gerekli)
        # self.map_widget.map_clicked.connect(self.on_map_clicked)

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
        # Map tıklaması çalışmadığı için Add Waypoint butonu diyalog açar
        self.add_wp_button = QPushButton("Add Waypoint") # İsim güncellendi
        self.add_wp_button.clicked.connect(self.add_mission_command_dialog) # Diyalog ile ekle fonksiyonuna bağla

        self.remove_wp_button = QPushButton("Remove Selected")
        self.remove_wp_button.clicked.connect(self.remove_selected_mission_command)

        self.clear_local_mission_button = QPushButton("Clear Local")
        self.clear_local_mission_button.clicked.connect(self.clear_local_mission)

        # self.load_mission_file_button = QPushButton("Load File") # İsteğe bağlı dosya işlemleri
        # self.save_mission_file_button = QPushButton("Save File")

        self.upload_mission_button = QPushButton("Upload to Vehicle")
        self.upload_mission_button.clicked.connect(self.upload_local_mission)

        self.download_mission_button = QPushButton("Download from Vehicle")
        self.download_mission_button.clicked.connect(self.download_vehicle_mission)


        mission_buttons_layout.addWidget(self.add_wp_button)
        mission_buttons_layout.addWidget(self.remove_wp_button)
        mission_buttons_layout.addWidget(self.clear_local_mission_button)
        # mission_buttons_layout.addWidget(self.load_mission_file_button)
        # mission_buttons_layout.addWidget(self.save_mission_file_button)
        mission_buttons_layout.addStretch(1)
        mission_buttons_layout.addWidget(self.download_mission_button)
        mission_buttons_layout.addWidget(self.upload_mission_button)

        mission_layout.addWidget(self.mission_list_widget)
        mission_layout.addLayout(mission_buttons_layout)

        parent_layout.addWidget(mission_group)


    def _create_video_interface(self, parent_layout):
        video_group = QGroupBox("Video Stream")
        video_layout = QVBoxLayout(video_group)

        video_layout.addWidget(QLabel("<b>Video Stream</b>"))

        video_input_layout = QHBoxLayout()
        self.rtsp_label = QLabel("RTSP URL:")
        self.rtsp_input = QLineEdit()
        self.rtsp_input.setText("rtsp://<camera_ip>:<port>/<stream_path>") # Örnek URL
        self.rtsp_input.setPlaceholderText("E.g., rtsp://192.168.1.100:554/stream1")
        self.rtsp_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        self.stream_button = QPushButton("Start Stream")
        self.stream_button.clicked.connect(self.start_stop_stream)

        video_input_layout.addWidget(self.rtsp_label)
        video_input_layout.addWidget(self.rtsp_input)
        video_input_layout.addWidget(self.stream_button)

        self.video_display_label = QLabel("No video stream")
        self.video_display_label.setAlignment(Qt.AlignCenter)
        self.video_display_label.setStyleSheet("border: 1px solid gray; background-color: black; color: white;")
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
       if not DRONEKIT_INSTALLED:
           self.log_message("DroneKit is not installed. Cannot connect.")
           return

       # Eğer thread zaten çalışıyorsa, connect butonu disconnect görevi görsün
       if self.dronekit_thread is not None and self.dronekit_thread.isRunning():
            self.disconnect_vehicle()
            return

       connection_string = self.conn_input.text()
       if not connection_string:
           self.log_message("Please enter a connection string.")
           return

       self.dronekit_thread = DronekitThread(connection_string)
       # Sinyal bağlantıları
       self.dronekit_thread.connection_status_signal.connect(self.update_connection_status)
       self.dronekit_thread.vehicle_data_signal.connect(self.update_vehicle_data)
       self.dronekit_thread.log_message_signal.connect(self.log_message)
       self.dronekit_thread.mission_downloaded_signal.connect(self.on_mission_downloaded)

       # UI durumunu güncelle
       self.connect_button.setEnabled(False)
       self.connect_button.setText("Connecting...")
       self.connection_status_label.setStyleSheet("color: orange;")

       # Thread'i başlat
       self.dronekit_thread.start()


    def disconnect_vehicle(self):
        if self.dronekit_thread and self.dronekit_thread.isRunning():
            self.log_message("Disconnecting vehicle...")
            self.dronekit_thread.stop()
            # Thread'in durmasını beklemeden UI'ı güncelle
            self.connect_button.setEnabled(False)
            self.connect_button.setText("Disconnecting...")
            # Thread'in durduğunda update_connection_status sinyali gelecek.
        elif self.dronekit_thread is not None:
             # Thread zaten durmuş ama obje hala var
             self.log_message("Dronekit thread exists but is not running. Cleaning up.")
             if not self.dronekit_thread.isFinished():
                 self.dronekit_thread.wait(1000) # Kısa bekleme
             self.dronekit_thread = None
             self.update_connection_status("Disconnected") # UI'ı güncelle
        else:
            self.log_message("Vehicle not connected.")
            self.update_connection_status("Disconnected") # UI'ı "Disconnected" yap


    @pyqtSlot(str)
    def update_connection_status(self, status):
        self.connection_status_label.setText(f"Status: {status}")
        if "Connected" in status:
            self.connection_status_label.setStyleSheet("color: green;")
            self.connect_button.setEnabled(True)
            self.connect_button.setText("Disconnect")
            # Bağlantı kurulduğunda kontrol butonlarını varsayılan olarak etkinleştir
            self._update_button_states(connected=True, armed=False, armable=False)
            # Görev butonlarını bağlı durumuna göre güncelle (yerel liste boş olabilir)
            self._update_mission_planning_button_states(connected=True, mission_loaded=len(self.mission_commands) > 0)

        elif "Connecting" in status:
             self.connection_status_label.setStyleSheet("color: orange;")
             self.connect_button.setEnabled(False) # Bağlantı devam ederken buton pasif
             self.connect_button.setText("Connecting...")
             # Bağlanırken kontrol ve görev butonlarını devre dışı bırak
             self._update_button_states(connected=False, armed=False, armable=False)
             self._update_mission_planning_button_states(connected=False, mission_loaded=False)

        else: # Disconnected, Connection Failed, Connection Cancelled, Stream Error vb.
            self.connection_status_label.setStyleSheet("color: red;")
            self.connect_button.setEnabled(True)
            self.connect_button.setText("Connect")
            # Eğer thread hala varsa ama durduysa temizle
            if self.dronekit_thread and not self.dronekit_thread.isRunning():
                 if not self.dronekit_thread.isFinished():
                     self.dronekit_thread.wait(1000) # Kısa bekleme
                 self.dronekit_thread = None # Thread objesini serbest bırak

            # Bağlantı kesildiğinde tüm kontrol ve görev butonlarını devre dışı bırak
            self._update_button_states(connected=False, armed=False, armable=False)
            self._update_mission_planning_button_states(connected=False, mission_loaded=False)


    @pyqtSlot(dict)
    def update_vehicle_data(self, data):
        """Dronekit thread'inden gelen araç verileri sinyalini işler (GUI thread'inde çalışır)."""
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
             # Harita üzerindeki araç marker'ını güncelle (Foliumsuz MapWidget metodunu çağır)
             if PYQTWEBENGINE_INSTALLED:
                  self.map_widget.update_vehicle_location(location['lat'], location['lon'])
        else:
            self.location_label.setText("Location: Waiting for GPS...")
            # GPS yoksa harita üzerindeki araç marker'ını da temizleyebiliriz
            if PYQTWEBENGINE_INSTALLED:
                 self.map_widget.update_vehicle_location(None, None) # Marker'ı temizle

        self.mode_label.setText(f"Mode: {mode}")
        self.armable_label.setText(f"Armable: {'Yes' if is_armable else 'No'}")
        self.armed_label.setText(f"Armed: {'Yes' if armed else 'No'}")
        self.system_status_label.setText(f"System Status: {system_status}")

        if gps_info:
             self.gps_info_label.setText(f"GPS: Fix Type={gps_info['fix_type']}, Satellites={gps_info['satellites_visible']}")
             # GPS fix type > 1 (2D Fix veya daha iyisi) ise konum genellikle güvenilirdir
             if gps_info['fix_type'] < 2:
                  # Konum gelmiş olsa bile Fix Type düşükse uyarı logla veya etiketi güncelle
                   if location and location['lat'] is not None: # Konum verisi var ama fix düşük
                        self.log_message(f"Warning: GPS Fix Type is {gps_info['fix_type']}. Location data might be inaccurate.")
                   else: # Konum verisi de yok
                        self.location_label.setText("Location: Waiting for GPS Fix...")
        else:
             self.gps_info_label.setText("GPS: N/A")

        # if battery:
        #      self.battery_label.setText(f"Battery: {battery['voltage']:.2f}V ({battery['level']}%)")
        # else:
        #      self.battery_label.setText("Battery: N/A")


        # Kontrol ve Görev butonlarının durumunu güncelle
        # Bağlantı varsa (update_connection_status 'Connected' yaptıysa), armed/armable durumuna göre ayarla
        connection_active = self.dronekit_thread is not None and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle is not None
        self._update_button_states(connected=connection_active, armed=armed, armable=is_armable)
        self._update_mission_planning_button_states(connected=connection_active, mission_loaded=len(self.mission_commands) > 0)


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
                else:
                     button.setStyleSheet("") # Bağlı değilse temizle


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
                  self.disarm_button.setStyleSheet("background-color: salmon;") # Disarm her zaman kırmızı kalabilir
        else: # Bağlı değilse renkleri sıfırla ve devre dışı bırak
             self.arm_button.setStyleSheet("")
             self.disarm_button.setStyleSheet("")


    def _update_mission_planning_button_states(self, connected, mission_loaded):
        """Görev planlama butonlarının enabled/disabled durumlarını ayarlar."""
        # Görev ekleme/kaldırma/temizleme butonları
        self.add_wp_button.setEnabled(connected) # Bağlıyken yeni waypoint ekleyebiliriz
        self.remove_wp_button.setEnabled(connected and mission_loaded) # Yüklü görev varsa kaldırabiliriz
        self.clear_local_mission_button.setEnabled(connected and mission_loaded) # Yüklü görev varsa temizleyebiliriz

        # Upload/Download butonları
        self.upload_mission_button.setEnabled(connected and mission_loaded) # Yüklü görev varsa yükleyebiliriz
        self.download_mission_button.setEnabled(connected) # Bağlıyken her zaman indirebiliriz


    # --- Kontrol Butonu İşleyicileri ---
    # Bu fonksiyonlar GUI thread'inde çalışır, DronekitThread metotlarını çağırırlar.

    def on_arm_button_clicked(self):
         if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
            self.dronekit_thread.arm_vehicle() # Komutu Dronekit thread'ine gönder
         else:
            self.log_message("Cannot send ARM command: Vehicle not connected or thread not running.")

    def on_disarm_button_clicked(self):
         if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
            self.dronekit_thread.disarm_vehicle() # Komutu Dronekit thread'ine gönder
         else:
            self.log_message("Cannot send DISARM command: Vehicle not connected or thread not running.")

    def on_mode_button_clicked(self, mode_name):
        if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
             self.dronekit_thread.set_mode(mode_name) # Komutu Dronekit thread'ine gönder
        else:
            self.log_message(f"Cannot set mode to {mode_name}: Vehicle not connected or thread not running.")


    # --- Görev Planlama İşleyicileri (GUI thread'inde çalışır) ---

    # @pyqtSlot(float, float) # QWebChannel kurulursa bu sinyal MapWidget'tan gelebilir
    # def on_map_clicked(self, lat, lon):
    #    """Harita tıklandığında çağrılır (QWebChannel ile). Yeni görev noktası ekleme diyaloğu açar."""
    #    # NOT: Bu fonksiyon şu an QWebChannel kurulumu yapılmadığı için harita tıklaması ile otomatik ÇALIŞMAYACAKTIR.
    #    self.log_message(f"Map clicked at: Lat={lat:.6f}, Lon={lon:.6f}")
    #    if not (self.dronekit_thread and self.dronekit_thread.vehicle):
    #         self.log_message("Cannot add waypoint from map: Vehicle not connected.")
    #         return
    #    self.add_mission_command_dialog(default_lat=lat, default_lon=lon) # Diyalog açma fonksiyonunu çağır


    def add_mission_command_dialog(self, default_lat=None, default_lon=None):
        """'Add Waypoint' butonuna basıldığında veya harita tıklandığında çağrılır. Diyalog açar."""
        if not (self.dronekit_thread and self.dronekit_thread.vehicle):
             self.log_message("Cannot add waypoint: Vehicle not connected.")
             return

        # Eğer haritadan gelmiyorsa ve aracın konumu varsa, onu varsayılan yap
        if default_lat is None and self.dronekit_thread.vehicle.location.global_frame and \
           self.dronekit_thread.vehicle.location.global_frame.lat is not None:
             loc = self.dronekit_thread.vehicle.location.global_frame
             default_lat, default_lon = loc.lat, loc.lon
        elif default_lat is None: # Bağlı ama konumu yok veya geçerli değilse
            default_lat, default_lon = 0.0, 0.0


        dialog = MissionEditorDialog(self, default_lat=default_lat, default_lon=default_lon)
        if dialog.exec_(): # Diyalog modal olarak açılır ve kullanıcı OK'e basarsa True döner
            new_command = dialog.get_command()
            if new_command:
                # Komutu yerel listeye ekle (Genellikle index 0, HOME komutudur. Yeni WP'ler sona eklenir.)
                # Görev listesine eklerken sırasına dikkat etmek önemlidir.
                # DroneKit'te 0. komut genellikle HOME komutudur. Yeni komutlar 1'den başlayarak eklenir.
                # Basitlik için şimdilik listenin sonuna ekleyelim.
                self.mission_commands.append(new_command)
                self._update_mission_list_widget() # GUI listesini ve haritayı güncelle
                self.log_message(f"Waypoint added (dialog) - Lat={new_command.x:.6f}, Lon={new_command.y:.6f}, Alt={new_command.z:.2f}m")


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
            self._update_mission_list_widget() # GUI listesini ve haritayı güncelle
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

        # MapWidget'taki görev noktası listesini güncelle ve haritayı çizdir
        waypoints_for_map = []
        if not self.mission_commands:
             self.log_message("Mission list is empty.")
             if PYQTWEBENGINE_INSTALLED:
                  self.map_widget.set_mission_waypoints([]) # Haritayı temizle
             self._update_mission_planning_button_states(connected=True, mission_loaded=False) # Liste boşsa butonları devre dışı bırak
             return

        self.log_message(f"Displaying {len(self.mission_commands)} mission commands.")
        for i, command in enumerate(self.mission_commands):
            # Görev komutlarını liste widget'ında göster
            # Komut türünü ve önemli parametrelerini göster
            # NOTE: HOME komutu (genellikle index 0) bazen parametreleri (x, y, z) boş dönebilir.
            item_text = f"{i}: CMD={command.command} FRAME={command.frame} p1={command.param1:.2f} p2={command.param2:.2f} p3={command.param3:.2f} p4={command.param4:.2f} x={command.x:.6f} y={command.y:.6f} z={command.z:.2f}"
            self.mission_list_widget.addItem(item_text)

            # Eğer komut bir navigasyon komutuysa ve konumu geçerliyse haritaya eklemek için listeye al
            if command.command in [mavutil.mavlink.MAV_CMD.NAV_WAYPOINT,
                                   mavutil.mavlink.MAV_CMD.NAV_TAKEOFF,
                                   mavutil.mavlink.MAV_CMD.NAV_LAND,
                                   mavutil.mavlink.MAV_CMD.NAV_LOITER_UNLIM, # vb loiter komutları
                                   mavutil.mavlink.MAV_CMD.NAV_LOITER_TIME,
                                   mavutil.mavlink.MAV_CMD.NAV_LOITER_TO_ALT,
                                   mavutil.mavlink.MAV_CMD.NAV_RETURN_TO_LAUNCH]: # RTL komutu
                 # Konum geçerliyse (genellikle x=lat, y=lon)
                 # HOME komutu için 0.0, 0.0 olabilir, bunu çizmek istemeyebiliriz
                 if abs(command.x) > 0.000001 or abs(command.y) > 0.000001:
                      waypoints_for_map.append((command.x, command.y, i, command.command)) # lat, lon, index, command_type


        # MapWidget'a güncellenmiş görev noktaları listesini gönder ve haritayı çizdir
        if PYQTWEBENGINE_INSTALLED:
             self.map_widget.set_mission_waypoints(waypoints_for_map)

        self._update_mission_planning_button_states(connected=True, mission_loaded=True) # Liste doluysa butonları aktif et


    # --- Video Stream İşleyicileri (GUI thread'inde çalışır) ---

    def start_stop_stream(self):
        """Video stream başlat/durdur butonuna basıldığında çağrılır."""
        if not OPENCV_INSTALLED:
             self.log_message("OpenCV (cv2) is not installed. Cannot start video stream.")
             return

        if self.video_thread is not None and self.video_thread.isRunning():
            self.stop_stream()
        else:
            rtsp_url = self.rtsp_input.text()
            if not rtsp_url or rtsp_url == "rtsp://<camera_ip>:<port>/<stream_path>":
                self.log_message("Please enter a valid RTSP URL.")
                return

            self.video_thread = VideoThread(rtsp_url)
            self.video_thread.frame_signal.connect(self.update_video_frame)
            self.video_thread.stream_status_signal.connect(self.update_stream_status)
            self.video_thread.log_message_signal.connect(self.log_message)

            # UI durumunu güncelle
            self.stream_button.setEnabled(False)
            self.stream_button.setText("Connecting...")
            self.stream_status_label.setStyleSheet("color: orange;")

            # Thread'i başlat
            self.video_thread.start()


    def stop_stream(self):
        """Video stream durdurma işlemi."""
        if self.video_thread and self.video_thread.isRunning():
            self.log_message("Stopping video stream...")
            self.video_thread.stop()
            # Thread'in durmasını beklemeden UI'ı güncelle
            self.stream_button.setEnabled(False)
            self.stream_button.setText("Stopping...")
        elif self.video_thread is not None:
             # Thread zaten durmuş ama obje hala var
             self.log_message("Video thread exists but is not running. Cleaning up.")
             if not self.video_thread.isFinished():
                 self.video_thread.wait(1000) # Kısa bekleme
             self.video_thread = None
             self.update_stream_status("Stopped") # UI'ı güncelle
        else:
            self.log_message("Video stream is not active.")
            self.update_stream_status("Stopped")


    @pyqtSlot(QImage)
    def update_video_frame(self, q_image):
        """Video thread'inden gelen QImage karesini gösterir (GUI thread'inde çalışır)."""
        if q_image is None or q_image.isNull():
             # Hata durumunda veya boş kare geldiğinde
             self.video_display_label.setText("Error: No frame received")
             self.video_display_label.setPixmap(QPixmap()) # Resmi temizle
             self.video_display_label.setStyleSheet("border: 1px solid gray; background-color: black; color: red;")
             return

        pixmap = QPixmap.fromImage(q_image)
        # Video display label'ının boyutuna uydurmak için ölçeklendir
        # Qt.KeepAspectRatio oranları korur, Qt.SmoothTransformation kaliteyi artırır
        scaled_pixmap = pixmap.scaled(self.video_display_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.video_display_label.setPixmap(scaled_pixmap)
        self.video_display_label.setText("") # Resim geldiğinde placeholder yazısını gizle
        self.video_display_label.setStyleSheet("border: 1px solid gray; background-color: black;") # Arkaplanı siyaha çek


    @pyqtSlot(str)
    def update_stream_status(self, status):
        """Video thread'inden gelen stream durumu sinyalini işler (GUI thread'inde çalışır)."""
        self.stream_status_label.setText(f"Status: {status}")
        if "Streaming" in status:
            self.stream_status_label.setStyleSheet("color: green;")
            self.stream_button.setEnabled(True)
            self.stream_button.setText("Stop Stream")
            # Video display label'ının stilini normal hale getir (hata stili varsa)
            self.video_display_label.setStyleSheet("border: 1px solid gray; background-color: black;")

        elif "Failed" in status or "Disconnected" in status or "Error" in status or "Stopped" in status:
             self.stream_status_label.setStyleSheet("color: red;")
             self.stream_button.setEnabled(True)
             self.stream_button.setText("Start Stream")
             self.video_display_label.setText(f"No video stream ({status})") # Hata/Kesilme durumunda placeholder yazısı
             self.video_display_label.setPixmap(QPixmap()) # Resmi temizle
             self.video_display_label.setStyleSheet("border: 1px solid gray; background-color: black; color: red;") # Hata stili

             # Thread'i temizle (eğer kendiliğinden durduysa veya durdurma komutu tamamlandıysa)
             if self.video_thread and not self.video_thread.isRunning():
                  if not self.video_thread.isFinished():
                      self.video_thread.wait(1000) # Kısa bekleme
                  self.video_thread = None

        elif "Connecting" in status:
            self.stream_status_label.setStyleSheet("color: orange;")
            self.stream_button.setEnabled(False) # Bağlantı devam ederken buton pasif
            self.video_display_label.setStyleSheet("border: 1px solid gray; background-color: black; color: orange;") # Bağlanıyor stili


        else: # Idle vs.
             self.stream_status_label.setStyleSheet("color: gray;")
             self.stream_button.setEnabled(True)
             self.stream_button.setText("Start Stream")
             self.video_display_label.setStyleSheet("border: 1px solid gray; background-color: black; color: white;")


    # --- Genel Yardımcı Metotlar ---
    @pyqtSlot(str)
    def log_message(self, message):
        """Log alanına zaman damgalı mesaj ekler (GUI thread'inde çalışır)."""
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
            if not self.dronekit_thread.wait(5000): # 5 saniye bekle
                self.log_message("Dronekit thread did not stop gracefully within 5s.")
            else:
                 self.log_message("Dronekit thread stopped.")
            self.dronekit_thread = None


        # Video thread'i durdur
        if self.video_thread and self.video_thread.isRunning():
            self.log_message("Stopping Video thread...")
            self.video_thread.stop()
            if not self.video_thread.wait(5000): # 5 saniye bekle
                 self.log_message("Video thread did not stop gracefully within 5s.")
            else:
                 self.log_message("Video thread stopped.")
            self.video_thread = None


        self.log_message("Application closed.")
        event.accept() # Pencere kapatma olayına izin ver


# --- Uygulamanın Başlangıç Noktası ---
if __name__ == "__main__":

    app = QApplication(sys.argv)
    main_window = GCSWindow()
    main_window.show()
    sys.exit(app.exec_())
