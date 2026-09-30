# Dizin — yol haritası maddesi ↔ dosya eşlemesi

Bu dosya, kullanıcı geri bildirimi üzerine eklendi: yol haritasındaki
(README'deki "Yol haritası" bölümü) madde numaraları ile `demo/` altındaki
dosya adları BİREBİR örtüşmüyor (ör. `step8_collision_friction.py` hem
madde 8'i hem madde 11'i kapsıyor). Bu tablo, hangi maddenin hangi
dosya(lar)da yaşadığını ve neden bazı numaraların "kendi dosyası" olmadığını
açıkça gösteriyor.

## Madde → dosya tablosu

| Yol haritası maddesi | Demo dosyası | Durum |
|---|---|---|
| 1. Verlet zinciri (organik vs. robotik) | `demo/step1_verlet_chain.py` | ✅ |
| 2. FABRIK + foot-planting yürüyüş | `demo/step2_leg_reach.py` | ✅ |
| 3. Tam iskelet (gövde+baş+kol+bacak) | `demo/step3_full_skeleton.py` | ✅ |
| 4. Yürüyüş ince ayarı (karşı-kol, gravity/friction) | `demo/step4_gait_tuning.py` | ✅ |
| 5. Parallax arka plan + bacak erişilebilirlik düzeltmesi | `demo/step5_parallax.py` | ✅ |
| 6. Senaryo/diyalog sistemi | `demo/step6_scenario_timeline.py` | ✅ Saniye bazlı timeline, sprite/prop, kamera, opsiyonel ses; diyalog/lip-sync kapsam dışı |
| 7. Momentum ve esneme (squash & stretch) | `demo/step7_squash_stretch.py` | 🔶 |
| 8. Çoklu nokta çarpışması (zemin) | `demo/step8_collision_friction.py` | 🔶 (11 ile birleşik, aşağıya bkz.) |
| 9. Aktif/pasif ragdoll harmanı | `demo/step9_ragdoll_blend.py` | 🔶 |
| 10. İkincil fizik nesneleri (rüzgar/pelerin) | `demo/step10_secondary_wind.py` | 🔶 |
| 11. Dinamik sürtünme/tutunma | `demo/step8_collision_friction.py` | 🔶 **madde 8 ile AYNI dosyada** — bilinçli: ikisi de aynı `collide_ground()` mekanizmasını kullanıyor, biri olmadan diğerini anlamlı göstermek zordu (bkz. o dosyanın kendi docstring'i) |
| 12. Kütle merkezi (center of mass) dengesi | `demo/step12_balance.py` | 🔶 |
| 13. Tam entegrasyon stres testi | `demo/step13_full_integration_test.py` | 🔶 (yol haritasında numaralı bir "madde" değil, ayrı bir doğrulama/stres testi — bkz. aşağıdaki not) |
| 14. Serbest kalçalı dinamik biped (capture-point) | `demo/step14_active_biped.py` (`ActiveBipedSim`) | 🔶 Adım 17'de döngü sınıfa taşındı (bit-bit aynı), diz dalı düzeltildi |
| 15–16. Segment ayak laboratuvarları | `demo/step15_segment_foot.py`, `demo/step16_segment_foot_lab.py` | 🔶 izole ölçüm |
| 17. Bilge derisi fizik iskeletinde + Faz A kontak-faz sensörü | `demo/step17_bilge_physics_skin.py`, `physics/active_gait.py` | 🔶 Faz B başlatılmadı (README "Adım 17") |
| — Bilge kinematik yürüyüş (REFERANS) | `demo/bilge_walk_validation.py`, `demo/bilge_walk_skinned.py` | render referansı: kalça kinematik eğriyle sürülür, ana motor DEĞİL |

`step6` artık `scene/` üretim katmanının ilk sahnesidir. `step11` yok
çünkü 11 numaralı iş `step8` ile aynı dosyada yaşıyor. Hiçbir demo dosyası
silinmedi/kaybolmadı.

## `step13_full_integration_test.py` neden farklı

Diğer tüm `stepN` dosyaları yol haritasındaki NUMARALANMIŞ bir maddeye
karşılık geliyor (yeni BİR mekanik, izole bir demoda). `step13` farklı bir
amaca hizmet ediyor: yeni bir mekanik TANITMIYOR, bunun yerine 7, 8, 9, 10,
11, 12 numaralı maddelerin HEPSİNİ AYNI ANDA, aynı karakter üzerinde
çalıştırıp birbirleriyle çakışıp çakışmadığını (ör. ragdoll blend'in
squash&stretch ile, rüzgarın denge mekanizmasıyla) ve farklı `dt`/FPS
değerlerinde stabil kalıp kalmadığını sınayan bir ENTEGRASYON/stres testi.
Bu yüzden ayrı bir yol haritası maddesi numarası almadı; bkz. README'deki
"Master entegrasyon testi" notu.

## Çekirdek ve sahne modülleri (mekanik ↔ modül)

Kullanıcının "sorumlulukların ayrılması" (separation of concerns) geri
bildirimi üzerine, daha önce demo dosyalarında hardcode olan bazı mantık
çekirdek modüllere çıkarıldı:

| Modül | İçerik | Kullanan demo(lar) |
|---|---|---|
| `physics/verlet.py` | `VerletSystem` (nokta/çubuk motoru, gravity/wind/friction, `add_stick(compliance=...)`), `clamp_direction()` | hepsi |
| `physics/fabrik.py` | `FabrikChain2D`, `clamp_joint_angles()`, `clamp_joint_angle_points()` (pasif/ragdoll temsili için) | step2-9, 12, 13 |
| `physics/gait.py` | `FootPlantingLeg` (`last_overrun_px` ile erişim-aşımı dışa açık) | step2-9, 12, 13 |
| `physics/collision.py` | `collide_ground(body, floor_fn, friction_fn)` — VerletSystem'den bağımsız zemin çarpışması | step7, 8, 9, 13 |
| `physics/self_collision.py` | `push_points_off_segment()`, `apply_drag()` — ikincil zincirlerin (pelerin) ana gövdeden itilmesi | step13 |
| `physics/environment.py` | `Terrain` (zemin + bölgesel sürtünme), `GustWind` (düzensiz rüzgar) | step7, 8, 9, 10, 13 |
| `physics/ragdoll.py` | aktif/pasif blend yardımcıları (`blend_point`, `blended_max_angle`, `blended_friction`, `driver_follow_target`, `transition_impulse_vector`, `apply_impulse`) | step9, 13 |
| `physics/balance.py` | CoM-destek farkı + orantılı kol tepkisi (`upper_body_com_x`, `support_x`, `counter_balance_offset`, `reach_pulldown_offset`) | step9, 12, 13 |
| `scene/actions.py` | Eylemler, parametre doğrulama ve bağlama | step6 |
| `scene/export.py` | MP4 doğrulama ve isteğe bağlı ffmpeg ses mux | step6 |
| `scene/timeline.py` | Zaman-tabanlı action scheduler + tween/easing | step6 |
| `scene/actor.py` | On-screen aktor (position, scale, rotation, expr, props) | step6 |
| `scene/staging.py` | Alfa sınırına göre boyut, kalça/ayak teması, gölge ve örtücü maskeler | step6 |
| `scene/sprite.py` | PNG yükleme + fallback placeholder | step6 |
| `scene/camera.py` | 2D camera (pan, zoom, world-to-screen) | step6 |
| `scene/compositing.py` | Alpha blending, transform, Z-order comp | step6 |

Bu refactor SIRASINDA hiçbir sayısal davranış değişmedi -- her taşınan
mantık, taşınmadan önceki/sonraki demo çalıştırmalarının BİREBİR aynı
sayısal çıktıyı ürettiği doğrulanarak (regresyon) yapıldı (bkz. git commit
mesajı).

## 2. tur kullanıcı geri bildirimi (zemin sırası, eklem açısı, momentum,
## pelerin self-collision, kemik esnemesi)

Kullanıcının videoları izledikten sonra bildirdiği 5 maddelik ikinci bir
geri bildirim turu üzerine `step9`/`step13`'e 5 hedefli düzeltme eklendi
(hepsi önce sayısal olarak doğrulanıp sonra düzeltildi) -- ayrıntılı
madde madde döküm için `README.md`'deki "2. tur kullanıcı geri bildirimi
ve düzeltmeler" bölümüne bakın. Özet: zemin çarpışma sıra hatası
düzeltildi, pasif bacağa eklem açı kısıtı eklendi, geçişte momentum
enjeksiyonu eklendi, pelerin self-collision'ı eklendi, bacak erişim
aşımına kalça/CoM tepkisi eklendi -- ve normal yürüyüşte dizin her karede
neredeyse tam düz kalması (FABRIK'in "tembel" çözümü) ayrı, daha temel
bir bulgu olarak DÜZELTİLMEDEN dürüstçe belgelendi (bkz. README).

## 3. tur: Production scene layer (senaryo/timeline sistemi)

Kullanıcının "çocuk şarkıları için tekrar kullanılabilir bir 2D animasyon
üretim sistemi" talebine yanıt olarak, yeni `scene/` katmanı oluşturuldu.
Bu katman physics motorunun üzerine inşa edilerek:

- **Zaman-tabanlı (seconds) scenario scheduling** — karelere değil, saniye
  bazında kontrol, FPS-independent
- **Reusable timeline system** — `ActionType.MOVE_ACTOR`, `FADE_IN`,
  `SET_EXPRESSION`, `CAMERA_ZOOM` vb. eylemler, automatic tweening
- **Asset management + PNG fallback** — dosya yoksa renkli placeholder,
  koyduktan sonra otomatik yüklenir
- **Multi-actor scene composition** — prop attachment, Z-order, alpha blending
- **Simple 2D camera** — pan, zoom, world-to-screen transforms

İLK SENARYO: "Misafiri Severiz" (Türk çocuk şarkısı, aile salonu sahnesi,
4 saniye) — `demo/step6_scenario_timeline.py` ve `scene/` modülleri.

Mimari tasarım: `physics/` karakterleri değiştirmedi. `scene/` physics
motorunun ÜZERINE kuruldu, fakat katı bir bağımlılık değil — fizik kullanmayan sprite sahneleri de çalışır; gerçek rig adaptörü daha sonra eklenebilir.

## Step6 doğrulama ve sınırlar

`python3 demo/step6_scenario_timeline.py` eksik assetlerle de
`outputs/step6_misafiri_severiz.mp4` üretir: 1280×720, 30 FPS, 4 saniye.
`python3 -m unittest discover -s tests -v` zamanlama, PNG/compositing, prop,
rig arayüzü ve çıktı kontrollerini çalıştırır.
`python3 tests/smoke_demos.py` önceki demoları geçici çıktılarla çalıştırır.

Timeline ileri yönlüdür; geri sarma için sahne yeniden kurulur.
PROCEDURAL_RIG adaptör sözleşmesi ve parça çizimi hazırdır; gerçek gait/IK
adaptörünün genel Actor/timeline entegrasyonu henüz yapılmadı. Bilge'nin
parçalı karakteri ve gait/IK bağlantısı aşağıdaki bağımsız demoda bulunur.
Ses için ffmpeg gerekir; yokluğunda sessiz çıktı geçerlidir.

Step6 yerleşim ayarları: `assets/layouts/step6_misafiri_severiz.json`.
`--preview-only` ile 1280×720 hareketsiz yerleşim kontrol edilir; normal komut
aynı temaslarla 4 saniyelik MP4 üretir. Maskeler mevcut salon görseline özeldir.

## Bilge iskelet yürüyüş doğrulaması (bağımsız demo)

- `demo/bilge_walk_validation.py`: `FootPlantingLeg` + FABRIK, çocuk oranları,
  sabit kamera, 0–1 bekleme / 1–6 yürüyüş / 6–8 duruş; ayarlar `WalkConfig` içinde.
- `outputs/bilge_walk_validation.mp4`: 1280×720, 30 FPS, 8 saniye.
- `outputs/bilge_walk_validation_preview.png`: videodan dört duruş karesi.
- `outputs/bilge_walk_validation_report.json`: çözülmüş eklemler üzerinden
  ayak kayması, zemin ihlali, kemik uzunluğu ve diz yönü ölçümleri.
- `tests/test_bilge_walk_validation.py`: tüm yürüyüş boyunca fiziksel kısıtlar,
  karşı bacak/kol uyumu, başlangıç-duruş ve ölçüm regresyonları.

Temel step2/3/4 demoları çalıştırılarak mevcut altyapı doğrulandı.
Step15/16 ayak laboratuvarı ve step6 misafirlik sahnesi bu teste dahil değildir.

## Bilge parçalı karakter yürüyüşü

İnceleme rehberi ve görseller: [Zehra'ya değişiklik notları](docs/ZEHRA_DEGISIKLIK_NOTLARI.md).

- `demo/bilge_walk_skinned.py`: aynı doğrulama iskeletini görselle kaplar;
  `--debug` ile ayrı iskelet/temas kontrolü, `--preview-only` ile PNG.
- `scene/skinning.py`: iki noktalı affine parça bağlama ve alfa çizimi.
- `assets/characters/bilge_rig/`: 16 parça, `rig.json` bağlantıları,
  özgün yüz pikselleri ve referanstan tamamlanan gövde/uzuv atlası.
- `demo/prepare_bilge_rig.py`: atlas paketleme ve özgün başı ayırma.
- `outputs/bilge_walk_skinned.mp4`, `bilge_walk_skinned_preview.png`,
  `bilge_walk_skinned_report.json`: 8 saniyelik video, dört poz ve ölçümler.
- `tests/test_bilge_skinning.py`: kimlik, eklem bağlantıları, taban teması,
  iskelet verisinin korunması ve debug regresyonları.
