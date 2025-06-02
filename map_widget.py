# map_widget.py (Ayrı bir dosya olarak kaydedebilirsiniz, veya ana kodun üzerine ekleyebilirsiniz)

import os
import folium
import io
import json
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import Qt, QUrl, pyqtSignal, QObject

class MapWidget(QWidget):
    """
    Folium ile HTML harita oluşturan ve QtWebEngineView içinde gösteren widget.
    Python'dan JavaScript'e ve tersine iletişim yeteneği eklenmiştir.
    """
    # Tıklanan konum bilgisini göndermek için sinyal
    map_clicked = pyqtSignal(float, float) # latitude, longitude

    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0) # Kenar boşluklarını sıfırla

        self.web_view = QWebEngineView(self)
        self.layout.addWidget(self.web_view)

        self.map = None
        self.vehicle_marker = None # Araç marker'ı
        self.mission_markers = [] # Görev noktası marker'ları

        self._create_base_map()

        # Python'dan JavaScript'e komut göndermek için Kanal
        self.channel = QObject()
        # JavaScript'ten Python'a gelen mesajları almak için Slot
        self.channel.map_clicked.connect(self._handle_map_click) # JavaScript'ten gelen tıklama sinyalini bağla
        self.web_view.page().currentFrame().addToJavaScriptWindowObject("channel", self.channel)

        self._add_javascript_interface() # JavaScript iletişim arayüzünü ekle


    def _create_base_map(self, initial_coords=(39.9207, 32.8541), zoom_start=12):
        """Başlangıç haritasını oluşturur."""
        self.map = folium.Map(
            location=initial_coords,
            zoom_start=zoom_start,
            tiles='OpenStreetMap' # Farklı tile'lar kullanılabilir (Google Maps vb.)
        )

        # Haritayı bir HTML dosyası gibi belleğe kaydet
        data = io.BytesIO()
        self.map.save(data, close_html=True)
        html_content = data.getvalue().decode()

        # HTML içeriğini QWebEngineView'e yükle
        self.web_view.setHtml(html_content, QUrl("about:blank")) # base_url'i about:blank yapmak güvenlik/referans sorunlarını azaltır

    def _add_javascript_interface(self):
        """JavaScript tarafında Python ile iletişim kuracak fonksiyonları ekler."""
        # Harita yüklendikten sonra çalışacak JavaScript kodu ekleyelim
        # Bu kod, harita tıklama event'ini yakalar ve Python'daki 'channel' objesine sinyal gönderir.
        # Ayrıca, marker ekleme/güncelleme fonksiyonlarını da JavaScript'e ekler.

        js_code = """
        var map = {}; // Folium map object placeholder

        // Folium haritası yüklendikten sonra harita objesine eriş
        window.onload = function() {
            // Harita objesini bul (Folium'un nasıl oluşturduğuna bağlı)
            // Genellikle harita objesi 'map' id'li div içinde veya globalde bulunur.
            // Folium'un varsayılan id'si genellikle rastgele bir stringdir.
            // Folium map'i HTML içine yerleştirirken `name="map"` veya `id="map"`
            // gibi ek bilgiler ekleyerek erişimi kolaylaştırabiliriz,
            // veya Folium'un ürettiği HTML'i parse etmemiz gerekir.
            // Basitlik için, Folium'un oluşturduğu map objesini bulmaya çalışalım:
            for (var key in window) {
                if (window[key] && typeof window[key].addLayer === 'function' && typeof window[key].on === 'function') {
                    map = window[key];
                    break;
                }
            }

            if (map && typeof map.on === 'function') {
                console.log("Folium map object found.");

                // Harita tıklama olayını dinle
                map.on('click', function(e) {
                    console.log("Map clicked:", e.latlng.lat, e.latlng.lng);
                    // Python'daki channel objesine sinyal gönder
                    // addToJavaScriptWindowObject("channel", self.channel) ile 'channel' globalde erişilebilir.
                    if (window.channel) {
                        window.channel.map_clicked(e.latlng.lat, e.latlng.lng);
                    } else {
                        console.error("Python channel object not found.");
                    }
                });

                // Python'dan çağrılacak marker ekleme/güncelleme fonksiyonları
                window.addOrUpdateVehicleMarker = function(lat, lon) {
                    // Mevcut araç marker'ını bul ve kaldır
                    if (map.vehicle_marker) {
                        map.removeLayer(map.vehicle_marker);
                    }
                    // Yeni marker oluştur
                    map.vehicle_marker = L.marker([lat, lon]).addTo(map);
                    // Pop-up ekleyebiliriz
                    map.vehicle_marker.bindPopup("<b>Vehicle</b><br>Lat: " + lat.toFixed(6) + "<br>Lon: " + lon.toFixed(6)).openPopup();
                    // Haritayı yeni konuma merkezleyebiliriz (isteğe bağlı)
                    // map.setView([lat, lon], map.getZoom());
                };

                window.addMissionMarker = function(lat, lon, label) {
                     // Folium marker'ı oluştur
                     var marker = L.marker([lat, lon]).addTo(map);
                     marker.bindPopup("<b>Waypoint " + label + "</b><br>Lat: " + lat.toFixed(6) + "<br>Lon: " + lon.toFixed(6));
                     // Marker objesini sakla (ileride silmek/güncellemek için)
                     if (!map.mission_markers) map.mission_markers = [];
                     map.mission_markers.push({lat: lat, lon: lon, label: label, marker_obj: marker});
                     return map.mission_markers.length - 1; // listenin indexini döndür
                };

                window.removeMissionMarker = function(index) {
                    if (map.mission_markers && index >= 0 && index < map.mission_markers.length) {
                         var marker_info = map.mission_markers[index];
                         if (marker_info && marker_info.marker_obj) {
                              map.removeLayer(marker_info.marker_obj);
                              map.mission_markers.splice(index, 1); // Listeden kaldır
                              // Index'ler değiştiği için görev listesini de güncellemek gerekebilir
                         }
                    }
                };

                window.clearMissionMarkers = function() {
                    if (map.mission_markers) {
                        map.mission_markers.forEach(function(marker_info) {
                            if (marker_info && marker_info.marker_obj) {
                                map.removeLayer(marker_info.marker_obj);
                            }
                        });
                        map.mission_markers = [];
                    }
                };

                console.log("Map JavaScript interface added.");

            } else {
                console.error("Folium map object not found after window load.");
                 // Harita objesi bulunamazsa tekrar deneme veya farklı bir bulma yöntemi gerekebilir
            }
        };
        """
        # JavaScript kodunu web sayfasına enjekte et
        # QWebEngineView'in sayfayı yüklemesinden sonra enjekte etmek önemlidir.
        # Bu, QWebEngineView'in `loadFinished` sinyali ile yapılabilir.
        self.web_view.page().loadFinished.connect(lambda: self.web_view.page().runJavaScript(js_code))


    def _handle_map_click(self, lat, lon):
        """JavaScript'ten gelen harita tıklama sinyalini işler."""
        print(f"Received map click from JS: {lat}, {lon}")
        self.map_clicked.emit(lat, lon) # Bu sinyali ana GCSWindow'a ilet


    def update_vehicle_location(self, lat, lon):
        """Haritadaki araç marker'ını günceller veya ekler."""
        if self.map:
             # JavaScript fonksiyonunu çağır
             self.web_view.page().runJavaScript(f"addOrUpdateVehicleMarker({lat}, {lon});")

    def add_mission_waypoint(self, lat, lon, label):
        """Haritaya bir görev noktası marker'ı ekler."""
        if self.map:
             # JavaScript fonksiyonunu çağır
             self.web_view.page().runJavaScript(f"addMissionMarker({lat}, {lon}, '{label}');")
             # NOT: Şu an için Python tarafında marker bilgisini tutmuyoruz, sadece haritada gösteriyoruz.
             # Tam bir görev planlama için, Python tarafında da görev noktalarını bir listede tutmak gerekir.

    def clear_mission_markers(self):
         """Haritadaki tüm görev noktası marker'larını temizler."""
         if self.map:
             self.web_view.page().runJavaScript("clearMissionMarkers();")

    # Diğer harita işlemleri (zoom, merkezleme vb.) için JavaScript fonksiyonları eklenebilir.
