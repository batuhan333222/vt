import sys
import time
import threading
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QPushButton, QLineEdit, QTextEdit,
                             QMessageBox, QGridLayout)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont

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
                        location = self.vehicle.location.global_frame
                        mode = self.vehicle.mode.name
                        is_armable = self.vehicle.is_armable
                        armed = self.vehicle.armed
                        system_status = self.vehicle.system_status.state # NOT: Bu doğrudan bağlantı kalitesi değil, sistemin genel durumu

                        # Verileri ana GUI thread'ine sinyal ile gönder
                        self.vehicle_data_signal.emit({
                            "location": {"lat": location.lat, "lon": location.lon, "alt": location.alt} if location else None,
                            "mode": mode,
                            "is_armable": is_armable,
                            "armed": armed,
                            "system_status": system_status
                        })

                        # Daha sık veri almak isterseniz buradaki sleep süresini kısaltın
                        time.sleep(0.5) # 500 ms bekle

                    except APIException as e:
                        self.log_message_signal.emit(f"API Exception during data fetch: {e}")
                    except Exception as e:
                         self.log_message_signal.emit(f"Error during data fetch: {e}")
                         # Olası bağlantı kopması durumunda thread'i durdurabiliriz
                         # self.stop() # İsteğe bağlı: Hata olursa thread'i durdur

                else:
                     # Vehicle nesnesi yoksa, bir sorun var demektir.
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
        # Vehicle nesnesi varsa, bağlantıyı kapatmayı dene (güvenli kapatma için run metodunun sonu beklenir)
        # Alternatif olarak burada da kapatılabilir ama thread race condition'a dikkat edilmeli
        # if self.vehicle:
        #     try:
        #         self.vehicle.close()
        #         self.log_message_signal.emit("Vehicle connection requested close.")
        #     except Exception as e:
        #          self.log_message_signal.emit(f"Error during vehicle close request: {e}")
        #     self.vehicle = None


    # Aşağıdaki metodlar GUI thread'inden çağrılır ve thread güvenliği için
    # doğrudan MAVLink komutlarını gönderecek şekilde tasarlanmalıdır.
    # Dronekit'in kendi thread'inde olması MAVLink çağrılarının senkronizasyonunu kolaylaştırır.

    def arm_vehicle(self):
        """Aracı arm etme komutunu gönderir."""
        if self.vehicle and self.vehicle.is_armable:
            if not self.vehicle.armed:
                self.log_message_signal.emit("Arming vehicle...")
                try:
                    self.vehicle.arm()
                    self.log_message_signal.emit("Vehicle armed.")
                    # ARM durumunu kontrol etmek için kısa bir bekleme eklenebilir
                    # self.vehicle.wait_for_armable(True)
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
                self.vehicle.armed = False
                # DISARM durumunu kontrol etmek için kısa bir bekleme eklenebilir
                # self.vehicle.wait_for_armable(False) # Wait for disarmed state
                self.log_message_signal.emit("Vehicle disarmed.")
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
                self.log_message_signal.emit(f"Attempting to set mode to {mode_name}...")
                self.vehicle.mode = VehicleMode(mode_name)
                # Mod değişimini doğrulamak için kısa bir bekleme veya durum kontrolü eklenebilir
                # self.vehicle.wait_for_mode(mode_name)
                self.log_message_signal.emit(f"Vehicle mode set to {self.vehicle.mode.name}.") # Güncel modu teyit et
            except APIException as e:
                self.log_message_signal.emit(f"Setting mode failed: {e}")
            except Exception as e:
                self.log_message_signal.emit(f"Setting mode error: {e}")
        else:
            self.log_message_signal.emit("Cannot set mode: Vehicle not connected.")

class GCSWindow(QMainWindow):
    """
    Ana yer kontrol istasyonu penceresi.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Basic VTOL GCS")
        self.setGeometry(100, 100, 800, 600) # Pencere boyutunu ayarla

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.dronekit_thread = None # Dronekit thread nesnesi burada tutulacak

        self._create_connection_interface()
        self._create_status_display()
        self._create_map_placeholder()
        self._create_control_buttons()
        self._create_log_area()

        self._update_button_states(connected=False, armed=False, armable=False) # Başlangıçta butonları devre dışı bırak

    def _create_connection_interface(self):
        """Bağlantı arayüzü widget'larını oluşturur."""
        conn_layout = QHBoxLayout()

        self.conn_label = QLabel("Connection String:")
        self.conn_input = QLineEdit()
        self.conn_input.setText("udp:127.0.0.1:14550") # Varsayılan bağlantı stringi (SITL için)
        self.conn_input.setPlaceholderText("E.g., udp:127.0.0.1:14550 or /dev/ttyACM0 or COMx")

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_vehicle)

        self.connection_status_label = QLabel("Status: Disconnected")
        self.connection_status_label.setStyleSheet("color: red;") # Bağlantı yoksa kırmızı

        conn_layout.addWidget(self.conn_label)
        conn_layout.addWidget(self.conn_input)
        conn_layout.addWidget(self.connect_button)
        conn_layout.addWidget(self.connection_status_label)
        conn_layout.addStretch(1) # Boş alanı doldur

        self.layout.addLayout(conn_layout)

    def _create_status_display(self):
        """Araç durumu (konum, mod vb.) gösteren widget'ları oluşturur."""
        status_layout = QVBoxLayout()
        status_layout.addWidget(QLabel("<b>Vehicle Status</b>"))

        self.location_label = QLabel("Location: N/A")
        self.mode_label = QLabel("Mode: N/A")
        self.armable_label = QLabel("Armable: N/A")
        self.armed_label = QLabel("Armed: N/A")
        self.system_status_label = QLabel("System Status: N/A") # Bağlantı kalitesi placeholder

        status_layout.addWidget(self.location_label)
        status_layout.addWidget(self.mode_label)
        status_layout.addWidget(self.armable_label)
        status_layout.addWidget(self.armed_label)
        status_layout.addWidget(self.system_status_label)

        self.layout.addLayout(status_layout)

    def _create_map_placeholder(self):
        """Harita görselleştirme için bir yer tutucu (placeholder) oluşturur."""
        map_layout = QVBoxLayout()
        map_layout.addWidget(QLabel("<b>Map Placeholder</b>"))
        # Gerçek bir harita widget'ı buraya entegre edilebilir (örn: QtWebEngineWidgets ile web haritası)
        # Şimdilik sadece konum bilgisini göstereceğiz.
        self.map_placeholder_label = QLabel("Vehicle Position will be shown here or coordinates displayed above.")
        self.map_placeholder_label.setAlignment(Qt.AlignCenter)
        self.map_placeholder_label.setStyleSheet("border: 1px solid gray; min-height: 200px;") # Görsel bir sınır ekle
        map_layout.addWidget(self.map_placeholder_label)
        self.layout.addLayout(map_layout)


    def _create_control_buttons(self):
        """Kontrol butonlarını (ARM/DISARM, Modlar) oluşturur."""
        control_layout = QVBoxLayout()
        control_layout.addWidget(QLabel("<b>Vehicle Control</b>"))

        # ARM/DISARM Butonları
        arm_disarm_layout = QHBoxLayout()
        self.arm_button = QPushButton("ARM")
        self.arm_button.setStyleSheet("background-color: green;")
        self.arm_button.clicked.connect(self.on_arm_button_clicked)

        self.disarm_button = QPushButton("DISARM")
        self.disarm_button.setStyleSheet("background-color: red;")
        self.disarm_button.clicked.connect(self.on_disarm_button_clicked)

        arm_disarm_layout.addWidget(self.arm_button)
        arm_disarm_layout.addWidget(self.disarm_button)
        control_layout.addLayout(arm_disarm_layout)

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
        control_layout.addLayout(mode_layout)

        self.layout.addLayout(control_layout)

    def _create_log_area(self):
        """Log mesajlarını gösteren alanı oluşturur."""
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setFont(QFont("Courier", 10)) # Monospace font loglar için iyi olabilir
        self.layout.addWidget(QLabel("<b>Log / Status Messages</b>"))
        self.layout.addWidget(self.log_text_edit)


    def connect_vehicle(self):
        """Bağlantı butonuna basıldığında çağrılır."""
        if self.dronekit_thread is not None and self.dronekit_thread.isRunning():
            # Bağlantı zaten aktif veya kuruluyor
            self.log_message(f"Connection thread is already running. Status: {self.connection_status_label.text()}")
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
        self.dronekit_thread.start() # Thread'i başlat

    def update_connection_status(self, status):
        """Dronekit thread'inden gelen bağlantı durumu sinyalini işler."""
        self.connection_status_label.setText(f"Status: {status}")
        if "Connected" in status:
            self.connection_status_label.setStyleSheet("color: green;")
            self.connect_button.setText("Disconnect") # İleride disconnect butonu eklemek isterseniz
            # self.connect_button.clicked.disconnect(self.connect_vehicle) # Bağlantıyı kesme slotuna bağla
            # self.connect_button.clicked.connect(self.disconnect_vehicle)
            self._update_button_states(connected=True, armed=False, armable=False) # Veri gelene kadar varsayılan
        else:
            self.connection_status_label.setStyleSheet("color: red;")
            self.connect_button.setEnabled(True) # Tekrar bağlanmayı etkinleştir
            self.connect_button.setText("Connect")
            if self.dronekit_thread and not self.dronekit_thread.isRunning():
                 # Eğer thread durduysa ve bağlantı yoksa, thread referansını temizle
                 self.dronekit_thread = None
            self._update_button_states(connected=False, armed=False, armable=False)


    def update_vehicle_data(self, data):
        """Dronekit thread'inden gelen araç verileri sinyalini işler."""
        location = data.get("location")
        mode = data.get("mode", "N/A")
        is_armable = data.get("is_armable", False)
        armed = data.get("armed", False)
        system_status = data.get("system_status", "N/A")

        if location:
            self.location_label.setText(f"Location: Lat={location['lat']:.6f}, Lon={location['lon']:.6f}, Alt={location['alt']:.2f}m")
            # Harita placeholder'ı yerine burada koordinatları da gösterebilirsiniz
            # self.map_placeholder_label.setText(f"Current Location:\nLat: {location['lat']:.6f}\nLon: {location['lon']:.6f}\nAlt: {location['alt']:.2f}m")
        else:
            self.location_label.setText("Location: N/A")
            # self.map_placeholder_label.setText("Waiting for location data...")


        self.mode_label.setText(f"Mode: {mode}")
        self.armable_label.setText(f"Armable: {'Yes' if is_armable else 'No'}")
        self.armed_label.setText(f"Armed: {'Yes' if armed else 'No'}")
        # System Status: Bağlantı kalitesini doğrudan göstermese de sistemin genel sağlığını belirtir
        self.system_status_label.setText(f"System Status: {system_status}")


        # ARM/DISARM butonlarının durumunu güncelle
        self._update_button_states(connected=True, armed=armed, armable=is_armable)


    def _update_button_states(self, connected, armed, armable):
        """Butonların enabled/disabled durumlarını ayarlar."""
        # Mod butonları sadece bağlıyken aktif olsun
        for button in self.findChildren(QPushButton):
            if button.text() in VTOL_MODES:
                button.setEnabled(connected and not armed) # Mod değişimi genellikle disarm veya arm sonrası (havada) yapılır

        # ARM/DISARM butonları
        self.arm_button.setEnabled(connected and armable and not armed)
        self.disarm_button.setEnabled(connected and armed)

        # Bağlantı butonu durumu update_connection_status içinde yönetiliyor


    def on_arm_button_clicked(self):
        """ARM butonuna basıldığında çağrılır."""
        if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
            # Komutu thread'e gönder
            # Thread güvenliği için doğrudan komut çağırmak yerine,
            # thread'in içinden çağrılacak bir method kullanmak daha iyidir.
            # Veya bir kuyruk sistemi kullanılabilir.
            # Basitlik için, thread'in içindeki methodu doğrudan çağırıyoruz,
            # ama thread'in kendisinin MAVLink komutlarını göndermesi gerektiğini unutmayın.
            self.dronekit_thread.arm_vehicle()
        else:
            self.log_message("Cannot arm: Vehicle not connected or thread not running.")


    def on_disarm_button_clicked(self):
        """DISARM butonuna basıldığında çağrılır."""
        if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
            self.dronekit_thread.disarm_vehicle()
        else:
            self.log_message("Cannot disarm: Vehicle not connected or thread not running.")

    def on_mode_button_clicked(self, mode_name):
        """Mod butonlarından birine basıldığında çağrılır."""
        if self.dronekit_thread and self.dronekit_thread.isRunning() and self.dronekit_thread.vehicle:
            self.dronekit_thread.set_mode(mode_name)
        else:
            self.log_message(f"Cannot set mode {mode_name}: Vehicle not connected or thread not running.")

    def log_message(self, message):
        """Log alanına mesaj ekler."""
        self.log_text_edit.append(f"[{time.strftime('%H:%M:%S')}] {message}")

    def closeEvent(self, event):
        """Pencere kapatıldığında çağrılır. Thread'i güvenli bir şekilde durdurur."""
        if self.dronekit_thread and self.dronekit_thread.isRunning():
            self.log_message("Stopping Dronekit thread...")
            self.dronekit_thread.stop() # Durdurma sinyali ver
            self.dronekit_thread.wait() # Thread'in bitmesini bekle
            self.log_message("Dronekit thread stopped.")
        event.accept() # Pencereyi kapatma olayını kabul et


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = GCSWindow()
    main_window.show()
    sys.exit(app.exec_())
