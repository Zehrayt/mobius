# Sahne assetleri

Step6 için dosyaları aşağıdaki adlarla yerleştirip scripti yeniden çalıştırın.
Kod değişikliği gerekmez. Eksik/bozuk dosyada bir uyarı ve isimli placeholder
üretilir; gerçek dosyanın üzerine placeholder yazılmaz.

| Klasör | Dosyalar | Sahnedeki görünür boyut |
|---|---|---|
| `backgrounds/` | `living_room.png` | 1280×720 salon |
| `characters/` | `bilge_smile.png`, `bilge_happy.png` | 350 px yükseklik |
| `characters/` | `bilge_brother_happy.png` | 264 px yükseklik |
| `characters/` | `guest_1.png` (dede) | 330 px yükseklik |
| `characters/` | `guest_2.png` (nine) | 315 px yükseklik |
| `props/` | `tea_tray.png`, `dessert_tray.png` | 136 / 112 px genişlik |
| `audio/` | `turkuz_biz.mp3` | 10–14. saniyeler |

Karakter ve prop PNG'leri şeffaf olmalıdır. Boyutlar `alpha > 16` figür
sınırlarına göre hesaplanır; dıştaki şeffaf boşluklar kırpılır, en-boy oranı
korunur. Kaynak PNG'lere yazılmaz; ışık düzeltmesi de yalnızca render kopyasına
uygulanır. Genel `SpriteManager` doğal boyutta BGRA yüklemeye devam eder.

## Ayarlanabilir yerleşim

`layouts/step6_misafiri_severiz.json` içindeki alanlar:

- `visible_height`: her karakterin bağımsız görünür yüksekliği.
- `contact_world`: 1280×720 dünya koordinatlarında minder/zemin noktası.
- `contact_uv`: figürün **görünür alfa kutusu** içinde 0–1 arası kalça/ayak referansı.
- `feet_uv`, `hands_uv`: ayak tabanları ve tepsinin bağlandığı avuç noktası.
- `seat_shadow_radius`, `foot_shadow_radius`: yumuşak temas gölgeleri.
- `light`: BGR renk çarpanı ve soldan gelen küçük ışık farkı.
- `props.support_uv`: tepsi tutamağı; avuç bu noktaya bağlanır.
- `occluders`: sehpa ve koltukların dünya piksel poligonları, boşlukları ve katman sırası.

Dedenin kalça teması `(657, 327)`, nineninki `(994, 348)` olarak kalibre edildi.
Bilge'nin zemin referansı `(465, 583)`, kardeşininki `(857, 564)`.
Kamera hareketi tüm katmanlara aynı dönüşümü uyguladığı için bu ilişkiler korunur.
Dede/nine sabit kalır; çocukların girişinde gölgeler ayaklarla birlikte hareket eder.

```bash
python3 demo/step6_scenario_timeline.py --preview-only
python3 demo/step6_scenario_timeline.py --layout assets/layouts/step6_misafiri_severiz.json
```

İlk komut statik PNG üretir. İkinci komut önce önizlemeyi, ardından aynı
mizansenle MP4'ü üretir. `outputs/layout_debug/contacts.png` temas işaretlerini,
`*_mask.png` dosyaları örtücü maskeleri, `placement.json` kaynak alfa kutuları,
gerçek ölçek oranları ve ayak noktalarını gösterir.

Maskeler düz arka plandan **aynı pikselleri** yeniden öne çizer; sehpanın
ayakları arasındaki boşluklar karakteri göstermeye devam eder. Soldaki yakın
koltuk giriş sırasında çocukları örter. Yeni salon görseli konduğunda maskeler
ve minder noktaları da yeniden kalibre edilmelidir.

Ses için PATH üzerinde veya proje içindeki `.venv/bin/ffmpeg` konumunda
`ffmpeg` gerekir. Bu makinede proje içine kuruldu. Örneğin:

```bash
python3 demo/step6_scenario_timeline.py --audio-start 12.5
```

Ses veya ffmpeg yoksa ya da mux başarısız olursa sessiz MP4 üretilir. Kısa
ses sessizlikle tamamlanır. Otomatik şarkı bölümü bulma ve lip-sync yoktur.

## Eklenen görseller (30 Eylül 2026)

Kullanıcının sağladığı PNG'ler içerikleri değiştirilmeden kopyalandı:

| Kaynak dosya | Projedeki dosya |
|---|---|
| `living_room.png` | `backgrounds/living_room.png` |
| `bilge_smile.png` | `characters/bilge_smile.png` |
| `bilge_happy.png` | `characters/bilge_happy.png` |
| `bilge_brother_happy.png` | `characters/bilge_brother_happy.png` |
| `dede.png` | `characters/guest_1.png` |
| `nene.png` | `characters/guest_2.png` |
| `cay.png` | `props/tea_tray.png` |
| `lokum_baklava.png` | `props/dessert_tray.png` |

Diğer uzun adlı salon PNG'si `living_room.png` ile birebir aynı olduğundan
ayrı kopyalanmadı. Downloads altındaki kaynak dosyalar korundu.

Yerleşim güncel ortası kanepeli, sağı yeşil koltuklu salon için kalibre edildi.
Bilge'nin iki PNG'sinde yüzle birlikte kol pozu da değişir; dolayısıyla
expression geçişi tüm pozu değiştirir. Bu yerleşimde Bilge happy pozunda
kalır; elleri cepte olan smile pozuna geçilmez. Tek PNG modu elleri tepsiye göre bükmez.

## Şarkı zamanlaması

`Türküz Biz.mp3`, `audio/turkuz_biz.mp3` olarak içerik değiştirilmeden eklendi.
Kullanıcının belirttiği başlangıç: “Misafiri severiz” **10. saniye**.
Step6 varsayılan olarak şarkının **10–14 saniye** aralığını kullanır.
Başka bir bölüm için `--audio-start` seçeneği kullanılabilir.

Sahne 4 saniyedir: çocukların girişi, hafif tepsi sunumu ve kamera yaklaşması bu süreye sığdırıldı.
Son 0.75 saniyede tamamlanan poz korunur; şarkının 14. saniyesinde sonraki
sahneye doğrudan kesme yapılabilir. Ses yavaşlatılmaz veya hızlandırılmaz.
