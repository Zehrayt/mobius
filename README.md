# Möbius — Video Modu (Prosedürel 2D Animasyon Motoru)

Karakterleri elle kare kare çizmek yerine; iskeletleri **Verlet integration**
(yerçekimi + mesafe kısıtlamalı nokta-çubuk fiziği) ve **FABRIK** (Inverse
Kinematics) ile prosedürel olarak hareket ettirip, sahneyi kare kare
`OpenCV` ile `.mp4` olarak render eden bir motor. Fizik demolarında hareket fizik ve hedef-tabanlı matematikten doğar.
Yeni `scene/` katmanı ise üretim sahnelerine saniye tabanlı mizansen ve
tween kontrolü ekler; fizik çekirdeğinin davranışını değiştirmez.

**Yol haritası maddesi ↔ dosya eşlemesi netleşmemişse (ör. neden
`step11` diye bir dosya yok) önce [`INDEX.md`](INDEX.md)'e bakın.**

Yeni sahne ve Bilge yürüyüşü için [Zehra'ya inceleme notları](docs/ZEHRA_DEGISIKLIK_NOTLARI.md):
dosya bazında açıklamalar, önizlemeler, çalıştırma komutları ve mevcut sınırlar.

## Kurulum

```bash
pip install numpy opencv-python
```

GPU gerekmez — bütün hesaplama numpy tabanlı skaler/vektör matematiği,
normal bir CPU'da gerçek zamanlıdan çok daha hızlı çalışır.

## Çalıştırma

### Physics Demos (steps 1–5, 7–16)

```bash
python3 demo/step1_verlet_chain.py   # -> outputs/step1_verlet_vs_robotic.mp4
python3 demo/step2_leg_reach.py      # -> outputs/step2_leg_reach.mp4
python3 demo/step3_full_skeleton.py  # -> outputs/step3_full_skeleton.mp4
python3 demo/step4_gait_tuning.py    # -> outputs/step4_gait_tuning.mp4
python3 demo/step5_parallax.py        # -> outputs/step5_parallax.mp4
python3 demo/step10_secondary_wind.py # -> outputs/step10_secondary_wind.mp4
python3 demo/step8_collision_friction.py # -> outputs/step8_collision_friction.mp4
python3 demo/step7_squash_stretch.py  # -> outputs/step7_squash_stretch.mp4
python3 demo/step9_ragdoll_blend.py   # -> outputs/step9_ragdoll_blend.mp4
python3 demo/step12_balance.py        # -> outputs/step12_balance.mp4
python3 demo/step13_full_integration_test.py # -> outputs/step13_full_integration_test.mp4
python3 demo/step14_active_biped.py # -> outputs/step14_active_biped.mp4
python3 demo/step15_segment_foot.py # -> outputs/step15_segment_foot.mp4
python3 demo/step16_segment_foot_lab.py # -> outputs/step16_segment_foot_lab.mp4
```

### Bilge: bağımsız iskelet yürüyüş testi

```bash
python3 demo/bilge_walk_validation.py
python3 -m unittest discover -s tests -p 'test_bilge_walk_validation.py' -v
```

`outputs/bilge_walk_validation.mp4`: 1280×720, 30 FPS, 8 saniye, sessiz.
0–1 saniye bekleme; 1–6 saniye hızlanarak yürüyüş ve yavaşlama;
6–8 saniye son basışın tamamlanması ve bekleme. Sabit kamera, sade zemin,
çocuk oranlarında çizgi/eklem iskeleti. `bilge_walk_validation_preview.png`
videodan çözülen bekleme, sol adım, sağ adım ve son duruş karelerini birleştirir.
`bilge_walk_validation_report.json` tüm 240 karenin sayısal ölçümlerini içerir.

Adımlar mevcut `physics.gait.FootPlantingLeg`, kol ve bacak eklemleri mevcut
`FabrikChain2D` ile çözülür. Kalça sürücüsü step3/step4 yaklaşımındaki gibi
kinematiktir; bu test dinamik denge simülasyonu değildir. Kol hedefleri karşı
bacağın gerçek konumundan türetilir. `WalkConfig` oranları, adım mesafesini,
kaldırma yüksekliğini, başlangıç/duruş zamanlarını ve IK toleransını toplar.
Ayak basma mekanizması kare tabanlı olduğundan test 30 Hz için ayarlanmıştır.

Kayma, her basışın ilk karesine göre **çözülmüş ayak ekleminin dünya x
koordinatından** ölçülür. Zemin ihlali aynı eklemin zemin altına uzaklığıdır;
ayaklar nokta temasıdır, ayakkabı/taban geometrisi yoktur. Kemik uzunlukları,
diz yönü, erişilebilirlik, tek ayak desteği ve kadraj da denetlenir. Diz dalı
başlangıçta seçilir; clamp sonrası ayağı zorla yerine koymak yerine FABRIK
hassas yakınsatılır. Motor çekirdeği değişmez. Step15/16 kullanılmaz ve
misafirlik sahnesine bağlanmaz.

### Bilge: parçalı görselle yürüyüş

```bash
python3 demo/bilge_walk_skinned.py
python3 demo/bilge_walk_skinned.py --debug
```

İlk komut `outputs/bilge_walk_skinned.mp4` ve
`outputs/bilge_walk_skinned_preview.png` üretir: 1280×720, 30 FPS,
8 saniye, sade zemin, normal videoda iskelet çizgileri yok.
`--debug`, orijinal eklemleri, görsel bilekleri ve taban temaslarını ayrı
`bilge_walk_skinned_debug.*` çıktılarında gösterir. `--preview-only`
yalnızca dört pozluk PNG ve ölçüm raporu üretir.

`bilge_walk_validation.simulate(WalkConfig())` aynı haliyle kullanılır;
zamanlama ve eklem koordinatları değişmez. 16 parçanın bağlantıları
`assets/characters/bilge_rig/rig.json` içindedir. Yüz, orijinal Bilge PNG'sinin
pikselleridir. Diğer parçalar ve gizli eklem yüzeyleri referansa göre
imagegen ile tamamlanmıştır; ayrıntılar [rig açıklamasında](assets/characters/bilge_rig/README.md).
`scene/skinning.py` iki bağlantılı dönüşümleri ve alfa çizimini yapar;
fizik motoru ve misafirlik sahnesi değiştirilmez.

Çözülmüş eklemlerin ölçümleriyle birlikte parça örtüşmeleri, ayakkabı
kayması ve görünür taban/zemin farkı `bilge_walk_skinned_report.json`
dosyasına yazılır. Önizleme dört video karesinin büyütülmüş kesitidir;
videoda kamera sabittir. Testler:
`python3 -m unittest discover -s tests -p 'test_bilge_skinning.py' -v`.

### Production Scene (Timeline + Assets)

```bash
python3 demo/step6_scenario_timeline.py # -> outputs/step6_misafiri_severiz.mp4
```

**İlk production scene: "Misafiri Severiz" (Türk çocuk şarkısı sahnesi, placeholder
assets ile başlayıp gerçek PNG'ler konabilir).**

## Mimari: physics, scene ve demo katmanları

### Katman 1: Physics (`physics/`)

**Jenerik, sahneden bağımsız.**

Belirli karakterleri, şarkıları ve salon sahnesini BİLMEZ. Sadece nokta/çubuk
fiziği, ters kinematik, çarpışma, ragdoll, denge gibi mekanizmalar sağlar.

- `physics/verlet.py` — genel amaçlı nokta/çubuk Verlet integration sistemi
  (`VerletSystem`) + `clamp_direction()` yardımcı fonksiyonu (bir segmentin
  yönünü sabit bir referansa göre sınırlar — bkz. aşağıdaki "double
  pendulum" notu) + `add_stick(..., compliance=...)` (esnek çubuklar,
  squash & stretch). Zincir, ağaç ya da kapalı iskelet (torso + kollar +
  bacaklar) gibi keyfi graf yapılarını destekler. **Zemin/çarpışma
  kavramından tamamen habersizdir** — bkz. `physics/collision.py`.
- `physics/fabrik.py` — 2D FABRIK IK çözücü (`FabrikChain2D`) +
  `clamp_joint_angles()` (bir eklemin büküm açısını ve yönünü sınırlar —
  ör. dizin tersine bükülmemesi). Bir zincirin (ör. kalça→diz→ayak bileği)
  ucunu bir hedef noktaya ulaştırır. Ayrıca `clamp_joint_angle_points()` —
  AYNI matematik ama bir `FabrikChain2D` nesnesi üzerinde değil, doğrudan
  `VerletSystem.points`/`prev_points` üzerinde çalışır; **pasif (ragdoll)**
  bacak temsiline de eklem açı sınırı uygulamak için eklendi (kullanıcı
  geri bildirimi — bkz. aşağıdaki "2. tur düzeltmeler" notu).
- `physics/gait.py` — `FootPlantingLeg`: FABRIK + "ayak basma" (foot-
  planting) state machine'i. Bu projede sıfırdan yazılmıştır. Artık her
  `update()` çağrısından sonra `last_overrun_px` (hedefin bacak menzilini
  ne kadar aştığı, 0 = erişilebilir) dışa açıyor — bkz. Adım 9/13 CoM
  tepkisi.
- `physics/collision.py` — `collide_ground(body, floor_fn, friction_fn)`:
  `VerletSystem`'den bağımsız bir zemin çarpışma fonksiyonu (önceden
  `VerletSystem.collide_ground()` metoduydu, sorumlulukların ayrılması
  için buraya taşındı — davranış birebir aynı, bkz. `INDEX.md`). ÖNEMLİ:
  bu fonksiyon zaten TÜM pinned-olmayan noktalara (el, pelerin, baş dahil)
  uygulanıyor — "sadece ayak" diye bir kısıtlama YOK, bkz. "2. tur
  düzeltmeler" notu.
- `physics/self_collision.py` — **YENİ** (2. tur kullanıcı geri bildirimi):
  `push_points_off_segment()` bir noktalar grubunu (ör. pelerin) bir
  gövde segmentinden (ör. kalça-omuz) en az `min_dist` uzakta tutacak
  şekilde iter; `apply_drag()` belirli noktalara `VerletSystem.friction`'dan
  BAĞIMSIZ ek bir hava direnci uygular. `collide_ground()` ile aynı ruhta
  (bağımsız, jenerik) minimal bir kendi-kendine-çarpışma mekanizması.
- `physics/environment.py` — sahneye özgü ama karakterden bağımsız iki
  yardımcı: `Terrain` (zemin yüksekliği + x aralığına göre bölgesel
  sürtünme — buz/normal zemin gibi) ve `GustWind` (birkaç uyumsuz sinüs
  frekansının toplamı + ramp ile düzensiz "gust" rüzgarı — tek bir
  `sin(t)` DEĞİL).
- `physics/ragdoll.py` — aktif (IK) / pasif (ragdoll) fizik harmanı için
  paylaşılan yardımcılar (`blend_point`, `blended_max_angle`,
  `blended_friction`, `driver_follow_target`) — bkz. Adım 9. Ayrıca
  `transition_impulse_vector()` + `apply_impulse()` — aktif→pasif geçiş
  ANINDA gövdeye karakterin kendi hızına orantılı tek seferlik bir darbe
  enjekte eder (kullanıcı geri bildirimi — "momentum aktarılmıyor").
- `physics/balance.py` — kütle merkezi (basitleştirilmiş üst-gövde vekili)
  ile destek tabanı farkına orantılı kol tepkisi yardımcıları
  (`upper_body_com_x`, `support_x`, `counter_balance_offset`) — bkz. Adım 12.
  Ayrıca `reach_pulldown_offset()` — bir bacak hedefine erişemediğinde
  (`FootPlantingLeg.last_overrun_px > 0`) kalçayı/gövdeyi aşağı+ileri
  eğen mimari tepki (kullanıcı geri bildirimi — "kemik esnemesi/kütle
  merkezi bağlantısızlığı").
- `demo/step1_verlet_chain.py` — **Adım 1**: Tek bir verlet zincirinin
  (kuyruk/kol) sabit bir anchor'dan sarkışını, aynı anchor hareketiyle
  sürülen saf `sin()` tabanlı "robotik" bir zincirle yan yana karşılaştırır.
- `demo/step2_leg_reach.py` — **Adım 2**: Verlet ile hafifçe sallanan bir
  kalça noktası + iki bacağın FABRIK ile "ayak basma" (foot-planting) state
  machine'iyle yürüme döngüsü. Bacaklar sırayla yerde sabit durur (stance)
  ve ileri taşınır (swing); tüm yürüyüş hiçbir keyframe olmadan bu
  mantıktan doğar.
- `demo/step3_full_skeleton.py` — **Adım 3**: Gövde + baş + 2 kol + 2 bacak.
  Gövde/boyun/kollar tek bir `VerletSystem` grafiği (pasif fizik), bacaklar
  ayrı `FootPlantingLeg`'ler (hedef güdümlü FABRIK + diz açı sınırı).
- `demo/step4_gait_tuning.py` — **Adım 4**: İnce ayar. Her kol artık ortak
  bir omuz noktasından değil kendi PINNED omuz tutamağından (`l_anchor` /
  `r_anchor`) sarkıyor; bu tutamaklar karşı bacağın swing ilerlemesine
  (`FootPlantingLeg.state` / `swing_t`) bağlı olarak öne/geriye kayıyor —
  gerçek bir yürüyüşteki karşı-bacak (contralateral) kol sallanmasının
  basit bir yaklaşıklaması. Ayrıca `VerletSystem.gravity` / `.friction`
  bu karakter için görsel olarak kalibre edildi (bkz. dosyanın başındaki
  yorum).
- `demo/step5_parallax.py` — **Adım 5**: Parallax arka plan katmanları.
  Her katmanın bir `depth` (derinlik) çarpanı var: `screen_x = world_x *
  depth + (W/2 - hip_x * depth)`. depth<1 uzak katmanlar (dağlar, oba)
  karakterden yavaş, depth>1 ön katman (çalılar) karakterden hızlı kayar.
  Şekiller gerçek sprite yerine basit geometrik ilkellerle (üçgen/daire)
  prosedürel üretilip sonsuza tekrarlanıyor — mimari aynı kalmak kaydıyla
  bunların yerine sanat ekibinin .png'leri konabilir.
- `demo/step10_secondary_wind.py` — **Adım 10 (ön çalışma)**: İkincil fizik
  nesneleri. Omuzdan sarkan bir pelerin (8 segmentlik saf verlet zinciri,
  **hiçbir hedefe bağlı değil, hiçbir açı kısıtlaması yok** — kollar/
  bacaklardan temel farkı bu) artık `physics/verlet.py`'ye eklenen `wind`/
  `wind_scale` ile hem karakterin kendi hareketinden (kaldıraç/ivme yoluyla)
  hem de düzensiz bir rüzgar/gust fonksiyonundan etkileniyor. İlk 3 saniye
  rüzgar kapalı (pelerin sadece kendi ağırlığı + karakterin yürüyüşüyle
  sallanıyor), sonra 1.5 saniyede rüzgar açılıp pelerin arkaya doğru
  bir bayrak gibi düzleşiyor — bkz. aşağıdaki "ikincil fizik" notu.
- `demo/step8_collision_friction.py` — **Adım 8 + 11 (başlangıç,
  birleşik)**: zemin çarpışması + bölgesel temas sürtünmesi. Pelerin 16
  segmente uzatıldı (karakter kısa süre durunca serbestçe yere kadar
  sarkıyor), yeni `VerletSystem.collide_ground()` bunun zeminin altına
  gömülmeden üzerinde sürüklenmesini sağlıyor. Ayrıca hiçbir çubuğa bağlı
  olmayan iki serbest "taş" (biri normal zeminde biri buzlu bir şeritte,
  aynı anda aynı hızla fırlatılıyor) bölgesel sürtünmenin GERÇEK etkisini
  gösteriyor — bkz. aşağıdaki yol haritası maddesi 11'deki dürüst sınır
  notu (aynı sürtünme, katı bir çubuk zincirinde neredeyse hiçbir şey
  ifade etmiyor).
- `demo/step7_squash_stretch.py` — **Adım 7 (başlangıç)**: momentum ve
  esneme (squash & stretch). `physics/verlet.py`'ye eklenen
  `add_stick(..., compliance=...)` ile halka + çapraz "jant" topolojisinde
  kurulmuş, esnek çubuklu bir "yumuşak top" zeminine düşüp çarpıyor.
  Tekdüze yerçekimi altında serbest düşüşte gerçek bir deformasyon
  OLMAZ (bütün noktalar aynı ivmeyi alır) — asıl squash/stretch SADECE
  zeminle çarpışma anında ortaya çıkıyor (alt noktalar aniden durur, üst/
  yan noktalar bir-iki kare daha düşmeye devam edip halkayı yassıltıyor),
  ardından esnek çubuklar topu kısmen geri yuvarlaklaştırıyor (rebound) ve
  sönümlü bir salınımla dinleniyor. Bkz. aşağıdaki yol haritası maddesi 7.
- `demo/step9_ragdoll_blend.py` — **Adım 9 (başlangıç)**: aktif/pasif
  ragdoll harmanı. Bacakların diz/ayak noktaları artık HEM `FabrikChain2D`
  ile IK-çözülüyor HEM DE aynı anda ana `VerletSystem`'in sıradan (rijit)
  noktaları — her karede ikisi arasında bir `blend` katsayısıyla (1=tam
  IK/aktif, 0=tam fizik/pasif) geçiş yapılıyor. Gövde/boyun
  `clamp_direction()` sınırı da aynı `blend` ile gevşetiliyor. Karakter
  3 saniye normal yürüyor, sonra bir "darbe" 0.6 saniyede kontrolü tamamen
  bırakıp karaktere önceden `clamp_direction()` ile BİLEREK engellenmiş
  olan kaotik çift-sarkaç (double pendulum) davranışını geri veriyor —
  bkz. aşağıdaki yol haritası maddesi 9. **2. tur düzeltmeler (kullanıcı
  geri bildirimi):** (1) `collide_ground()` artık diz/ayak blend-
  overwrite'ından SONRA bir kez daha çağrılıyor (17 kare ~7px zemin
  ihlali → 0); (2) pasif diz/ayak temsiline `clamp_joint_angle_points()`
  ile aktifle AYNI `KNEE_LIMITS` uygulanıyor (önce 0.3°-142.6° arası
  tamamen serbestti); (3) geçiş anında `transition_impulse_vector()` ile
  kalçaya/gövdeye tek seferlik bir darbe enjekte ediliyor (kalça vx
  1.85→3.14'e sıçrıyor, sonra doğal sönümleniyor) — artık "olduğu yerde
  çökme" değil, mevcut hareket yönünde fırlatılmış gibi başlıyor.
- `demo/step12_balance.py` — **Adım 12 (başlangıç)**: kütle merkezi (center
  of mass) dengesi. Her karede gövdenin (kalça+omuz+baş ortalaması) yatay
  konumu ile o an zeminde duran ayağın/ayakların yatay konumu arasındaki
  fark (`error`) hesaplanıp kollara bu farkla orantılı bir "dengeleme"
  ofseti (geriye/yukarı) uygulanıyor. t=3s'te bir tokezleme itkisi bu
  farkı aniden büyütüp kolların tepkisini net şekilde gösteriyor — bkz.
  aşağıdaki yol haritası maddesi 12.
- `demo/step13_full_integration_test.py` — **Master entegrasyon/stres
  testi** (yol haritasında numaralı bir madde değil — bkz. `INDEX.md`).
  Adım 7 (bağımsız sekiçen yumuşak top), 8+11 (buz/normal zemin
  çarpışması+sürtünmesi), 9 (aktif/pasif ragdoll geçişi), 10 (rüzgarlı
  pelerin) ve 12 (kütle merkezi dengesi) AYNI sahnede, AYNI karakterde,
  aynı anda çalışıyor. Potansiyel çakışmalar bilinçli olarak ele alındı:
  denge (12) kol ofseti ragdoll `blend`'i ile sıfıra çekiliyor (pasifte
  "denge" kavramının anlamı yok), pelerin ve gövde/bacaklar aynı
  `collide_ground()` çağrısını paylaşıyor, bağımsız top tamamen ayrı bir
  `VerletSystem`+`Terrain` ile aynı döngüde çalışıyor. Ayrıca AYNI sahne
  24/30/60 FPS'te (headless, sadece sayısal) koşturulup patlama (NaN)
  olup olmadığı kontrol edildi. **Dürüst bulgu:** hiçbir FPS'te
  patlama/NaN yok (sayısal olarak stabil), AMA `VerletSystem.step()` her
  zaman `dt=1.0` ile çağrıldığı için (gerçek `dt=1/FPS` sadece tetikleyici
  zamanlamasında kullanılıyor) fizik KARE SAYISINA bağlı, GERÇEK SANİYEYE
  değil — aynı 12 saniyelik sahnede FPS=24/30/60 için son kalça x'i
  sırasıyla 327.7/375.1/239.4 çıktı, yani şu an FPS'ten BAĞIMSIZ değil.
  Bu bir çökme değil ama dokümante edilmiş bir sınırlama; sonraki olası
  iş: `gravity`/`friction`/`wind` terimlerini gerçek `dt`'ye göre ölçekleyip
  motoru kare hızından tamamen bağımsız hale getirmek. **2. tur
  düzeltmeler:** step9 ile aynı zemin-sırası/eklem-açısı/momentum
  düzeltmeleri burada da uygulandı, ARTI: pelerin artık
  `physics/self_collision.py` ile gövde (kalça-omuz, omuz-kafa)
  segmentlerinden itiliyor + ekstra hava direnci (drag) alıyor (önce 360
  karenin birkaçında gövdeyi kesiyordu, min mesafe ~0.18px → ~2.2px'e
  çıktı — DÜRÜST SINIR: hedef 9px korunması HER ZAMAN tutmuyor, nokta-
  bazlı itme + çubuk gevşetmesinin etkileşimi yüzünden ara sıra daha
  yakın kalabiliyor); ve bacak hedefi menzili aştığında (`last_overrun_px`)
  `reach_pulldown_offset()` ile kalça/gövde aşağı+ileri eğiliyor (bu
  sahnede ölçülen maks. aşım ~23.4px, kalçayı orantılı olarak aşağı/ileri
  çekiyor).

### Önemli bir tasarım notu: "double pendulum" tuzağı

İlk denemede gövde (kalça→omuz) ve boyun (omuz→baş) tamamen serbest
(pinned olmayan) verlet noktaları olarak bağlanmıştı. Sonuç: klasik bir
fizik problemi olan **kaotik çift sarkaç (double pendulum)** davranışı —
karakter birkaç saniye içinde takla atıp gövdesi tersine dönüyordu. Çözüm,
`physics/fabrik.py`'deki diz açı sınırlamasıyla aynı fikri verlet
zincirine de uygulamak oldu: `clamp_direction()` ile gövdenin yönü sabit
bir global "yukarı" vektörüne, boynun yönü de (artık stabilize edilmiş)
gövde yönüne göre küçük bir açı aralığıyla sınırlandı. Bu, karakteri
yarı-rijit tutarken yine de hıza tepki veren hafif bir sallanma/eğilme
bırakıyor. **Referans olarak çok kısa bir segmentin (ör. driver→hip,
3px) yönünü kullanmayın** — sayısal olarak gürültülü olur ve açı
hesaplaması kararsızlaşır; sabit bir global yön veya zaten stabilize
edilmiş bir segmentin yönü kullanılmalı.

### Gelecek yön: "organik kütle" fiziği (Rain World'den ilham)

Temel yürüyüş (IK + Verlet) çözüldükten sonra, karakteri sadece eklemli bir
iskelet değil, esneyebilen/sıkışabilen ve momentumu hisseden organik bir
"kütle" gibi hissettirmek için hedeflenen ek mekanikler:

- **Momentum ve esneme (squash & stretch):** Karakter hızlanınca ya da
  yüksekten düşünce iskelet sabit kalmaz; gövdeyi oluşturan verlet
  zincirleri kinetik enerjiye tepki verip uzar (esner), yere çarpınca
  büzüşür — klasik animasyon ilkesi tamamen fizik/kod üzerinden.
- **Çoklu nokta çarpışması (multi-node collision & raycasting):** Karakter
  tek bir kaba kutu (hitbox) yerine baş/omuz/kalça/ayak gibi birden fazla
  noktadan zemin ve engellere ışın (raycast) yollayarak sahneyle etkileşir
  — dar bir geçitten geçerken vücut o geometriye göre sıkışıp adapte olur.
- **Aktif/pasif ragdoll harmanı:** Karakter koşarken/hedefe uzanırken kas
  gücü uygulayan "aktif" (IK güdümlü) durumda; sert bir çarpmada, denge
  kaybında ya da hasarda IK anlık olarak devre dışı kalıp karakter tamamen
  yerçekimine teslim "pasif ragdoll" durumuna geçiyor. Organik his, bu
  ikisi arasındaki pürüzsüz geçişten geliyor.
- **İkincil fizik nesneleri (secondary animation):** Kuyruk/kulak/anten/
  pelerin gibi parçalar hiçbir zaman bir hedefe ulaşmaya çalışmaz (IK
  kullanmazlar) — sadece ana gövdenin hareketinden, yerçekiminden ve hava
  sürtünmesinden (drag) etkilenen saf verlet zincirleridir. Ana karakter
  durduktan sonra bile kuyruğun bir süre daha salınmaya devam etmesi
  canlılık hissi veriyor.
- **Dinamik sürtünme/tutunma:** Eklemlerdeki sürtünme katsayısı zemin
  materyaline ve eyleme göre değişir — buzda ayak ucu sürtünmesi sıfıra
  yaklaşır, bir kenara/direğe tutunurken el sürtünmesi maksimuma çıkıp
  vücudu havada kilitler.
- **Kütle merkezi (center of mass) dengesi:** Kütle merkezi bir boşluğa
  (ör. uçurum kenarı) kayarsa sistem bunu algılayıp kolları ters yöne
  savurarak (counter-balance) dengeyi düzeltmeye çalışır.

Bu mekanikler tek seferde değil, modüler olarak eklenecek — örn. önce
`physics/verlet.py`'ye hava direnci (drag) eklenip rüzgarda salınan bir
kuyruk/kumaş simülasyonu ile başlanıp, ardından çarpışma/sürtünme
katsayılarına geçilmesi planlanıyor. Aşağıdaki yol haritasında 7-12
numaralı adımlar bunlara karşılık geliyor.

**İlk somut adım (Adım 10'un başlangıcı) atıldı** — bkz.
`demo/step10_secondary_wind.py` ve `physics/verlet.py`'deki yeni `wind`/
`wind_scale` alanları. Tasarım/ayar notu: ilk denemede rüzgar kuvveti
yerçekiminden (0.065) çok daha büyüktü (taban 0.55) ve pelerin gerçekte
"savrulmuyor", sadece anında dümdüz gerilip öyle kalıyordu (rüzgar tüm iç
fiziği/gust dalgalanmasını bastırıyordu). Sayısal olarak doğrulanan
düzeltme: rüzgar büyüklüğü yerçekimiyle aynı mertebeye indirildi (taban
0.12 + ~0.16 gust) ve pelerin 5'ten 8 segmente çıkarılıp daha fazla
"sarkma payı" verildi -- bu ikisi birlikte pelerinin hem rüzgarsızken
yürüyüş ritmine bağlı organik bir sallanma (uç noktanın göreli x'i:
ort. -81px, std 30) hem de rüzgar açıldığında arkaya doğru düzleşip bir
bayrak gibi gerilme (ort. -118px, std 2.5 -- yani neredeyse tam gerili
ve stabil) göstermesini sağladı. Görsel doğrulama: ffmpeg ile çıkarılan
karelerde geçiş (rüzgar 0 -> tam güç) pürüzsüz, ani bir "sıçrama" yok.

---

## Adım 6: Production scene / timeline

`scene/`, genel fizik motorunun yanında çalışan tekrar kullanılabilir üretim
katmanıdır. `demo/step6_scenario_timeline.py` yalnızca sahneye özgü oyuncuları,
asset eşlemelerini, placeholder çizimlerini ve saniye tabanlı mizanseni tanımlar.
`physics/` ve önceki demolar bu eklemede değiştirilmedi.

| Dosya | Sorumluluk |
|---|---|
| `scene/actions.py` | Eylem türleri, parametre doğrulama ve property bağlama |
| `scene/timeline.py` | Saniye bazlı olay yürütme, sabit başlangıçlı tween ve easing |
| `scene/actor.py` | Actor, expression, yerel anchor/prop ağacı, `RigAdapter` sözleşmesi |
| `scene/sprite.py` | PNG/BGRA yükleme, önbellek, eksik/bozuk asset uyarısı ve placeholder |
| `scene/camera.py` | Dünya/ekran koordinat dönüşümü, kamera merkezi ve zoom |
| `scene/compositing.py` | Alpha compositing, kırpılmayan döndürme, global z-index sırası |
| `scene/staging.py` | Görünür alfa sınırları, temas noktaları, ışık, yumuşak gölge ve mobilya maskeleri |
| `scene/export.py` | Kontrol edilen MP4 yazımı, isteğe bağlı ffmpeg ses birleştirme |

### Çalıştırma ve assetler

```bash
python3 demo/step6_scenario_timeline.py
# outputs/step6_misafiri_severiz.mp4 — 1280×720, 30 FPS, 4 saniye
python3 demo/step6_scenario_timeline.py --fps 24 --output outputs/step6_24fps.mp4
```

Varsayılan asset ve çıktı yolları dosyanın konumundan çözülür; komut başka
çalışma dizininden de çalışır. `--assets-dir` ve `--output` ile değiştirilebilir.

Beklenen dosyalar:

```text
assets/backgrounds/living_room.png
assets/characters/bilge_smile.png
assets/characters/bilge_happy.png
assets/characters/bilge_brother_happy.png
assets/characters/guest_1.png
assets/characters/guest_2.png
assets/props/tea_tray.png
assets/props/dessert_tray.png
assets/audio/turkuz_biz.mp3             # isteğe bağlı
```

PNG yoksa veya okunamıyorsa dosya başına bir uyarı ve isimli renkli placeholder
kullanılır. Step6 için salon, insan ve tepsi biçimli geometrik placeholder'lar
vardır. Gerçek PNG'leri aynı adlarla koyup yeniden çalıştırmak yeterlidir.
Expression eşlemesi açık bir sözlüktür; oyuncu adından dosya adı tahmin edilmez.
Kardeşin `smile` ifadesi ilk MVP'de mevcut `bilge_brother_happy.png` dosyasını
kullanır. Kardeş Mete değildir; Mete bu sahnede yer almaz.

Arka plan 1280×720 dünya tuvaline sığdırılır. `scene/staging.py` karakter ve
tepsilerin alfa kanalındaki görünür sınırlarını bulur; şeffaf kenar boşlukları
boyut hesabına katılmaz, figürler en-boy oranı korunarak ölçeklenir.
Bilge 350, kardeşi 264, dede 330, nine 315 piksel görünür yüksekliğindedir.
Kalça, ayak ve avuç temas noktaları; bağımsız boyutlar; sıcak ışık düzeltmesi;
sehpa/koltuk örtüşme poligonları
`assets/layouts/step6_misafiri_severiz.json` içinde ayarlanır.

```bash
python3 demo/step6_scenario_timeline.py --preview-only
# outputs/step6_misafiri_severiz_preview.png — hareketsiz, 1280×720
python3 demo/step6_scenario_timeline.py
# Aynı yerleşimle 4 saniyelik, sesli MP4
```

`outputs/layout_debug/` altında temas işaretli görsel, alfa maskeleri ve
ölçek/temas raporu üretilir. `--layout` ile başka bir ayar dosyası seçilebilir.
Örtücü katmanlar gerçek arka plan piksellerinden çıkarılır; sehpa boşlukları
şeffaf kalır. Kamera zoom'u arka plana, maskelere ve gölgelere birlikte uygulanır.
Oturma temasını bozmamak için dede/nineye tüm-gövde dönüşü veya bob uygulanmaz.
Çocuklar yerleşimdeki son konumlarına girer; ayak gölgeleri onları takip eder.
Kaynak PNG'ler değiştirilmez. Maskeler mevcut salon için kalibre edilmiştir;
arka plan değişirse poligonlar ve minder noktaları yeniden ayarlanmalıdır.
Ayrıntılar: [assets/README.md](assets/README.md).

Ses dosyası ve PATH üzerinde veya `.venv/bin/ffmpeg` konumunda `ffmpeg`
varsa video render edildikten sonra şarkının 10–14. saniyeleri AAC sesle
mux edilir. Başlangıç, kullanıcının verdiği “Misafiri severiz” zamanıdır. `--audio-start 12.5` gibi bir seçenekle
şarkının başka bir bölümünü seçebilirsiniz. Kısa ses sessizlikle tamamlanır;
video süresi 4 saniye kalır. Ses/ffmpeg yoksa veya mux başarısız olursa
anlaşılır mesajla sessiz MP4 korunur. Speech recognition ve lip-sync yoktur.

### Timeline kullanımı ve zaman sözleşmesi

```python
from scene import Actor, Camera, Timeline, ActionType as A

bilge = Actor("bilge", position=(-100, 400), visible=False)
camera = Camera(640, 360)
timeline = Timeline(camera)
timeline.at(0.7, A.SHOW_ACTOR, actor=bilge)
timeline.tween(0.7, 2.5, A.MOVE_ACTOR, actor=bilge,
               target=(480, 400), easing="ease_in_out")
timeline.tween(0, 4, A.CAMERA_ZOOM, zoom_target=1.045)
# Render döngüsünde timeline.update(frame_index / fps)
```

`at` ve `add_action` eşdeğerdir; `tween(start, end, ...)` saniye aralığı alır.
`update(t)` eylemleri kendisi uygular, ayrıca o çağrıda başlayan eylemleri
döndürür. Bunları render döngüsünde tekrar uygulamayın. Desteklenen eylemler:
show/hide, move/scale/rotate, expression, attach/detach, look_at, camera
move/zoom, fade_in/fade_out ve callback. Easing: `linear`, `ease_in_out`,
`ease_out` veya bir fonksiyon. Fade mevcut alpha'dan hedefe gider; tam bir
fade-in için actor'u `alpha=0` ile başlatın.

Olaylar kare aralarına düşse de atlanmaz, callback bir kez çalışır, tween
bitiş değeri tam uygulanır. Başlangıç değeri olayın gerçek başlangıç anında
alınır; her karede yeniden alınmaz. Aynı property'ye gelen sonraki eylem
öncekini keser; aynı zamandaki eylemlerde ekleme sırası geçerlidir.
Tüm eylemleri oynatmadan önce ekleyin. Saat ileri yönlüdür; geri sarma için
yeni bir sahne kurun. Callback'lerin dış yan etkileri otomatik geri alınmaz.

### Koordinatlar, prop ve fizik adaptörü

Dünya eksenleri sağa/aşağı, actor pivotu PNG merkezidir; dönüş derecedir ve
pozitif yön ekranda saat yönünün tersidir. Kamera konumu ekranın baktığı
dünya merkezidir. Zoom hem konum hem sprite boyutunu etkiler. Actor'un
`parallax_depth` değeri step5 ile aynı uzak/yakın kayma mantığını kullanır;
step6 katmanları sabit bir salon için varsayılan `1.0` kullanır.

`actor.attach_prop(prop, anchor="hands", offset=(20, -4))` bir yerel bağlantı
kurar. Bağlı prop'un position değeri ek yerel ofsettir; scale, rotation,
alpha ve visible üst actor'dan miras alınır. Prop'lar global z-index ile
bir kez çizilir. Detach dünya dönüşümünü korur; sonrasında görünmeye devam
etmesi için prop'u sahnenin actor listesinde tutun. Döngüsel veya birden
fazla ebeveynli bağlantılar reddedilir. `render_offset`/`render_rotation`
ikincil hareket içindir; timeline konumuna kümülatif sapma eklemez.

`SIMPLE_SPRITE` ilk sahnenin çalışan modudur. `walk_to(..., timeline=..., at=...)`
sprite hareketini zamanlar; `look_at` tüm PNG'ye küçük bir eğim verir.
`PROCEDURAL_RIG` için `RigAdapter` sözleşmesi hazırdır: `update(dt_seconds)`,
`walk_to`, `reach`, `set_pose`, `look_at`, `anchor` ve `parts`. Renderer
adaptörün yerel actor parçalarını çizebilir, prop bone anchor'ını kullanabilir.
Henüz gerçek Verlet/FABRIK/active-gait adaptörü yazılmadı; adaptör gerektiren
çağrılar sessizce hiçbir şey yapmak yerine açıklayıcı hata verir.
Gelecekteki adaptör fizik adımlarını sabit adım biriktiricisiyle yönetmelidir.
Mevcut fizik motorunun frame-count bağımlılığı bu görevde değiştirilmedi.

### Doğrulama

```bash
python3 -m compileall -q scene demo physics tests
python3 -m unittest discover -s tests -v
python3 tests/smoke_demos.py
```

Smoke kontrolü önceki demoların tam simülasyon/render döngülerini çalıştırır;
MP4'leri geçici klasöre yönlendirip son karelerinin okunabildiğini doğrular.
Step16 sayısal laboratuvarını da çalıştırır. Mevcut çıktıların üzerine yazmaz.

## Yol haritası

1. ~~Verlet zinciri (kuyruk/kol) — organik vs. robotik karşılaştırma~~ ✅
2. ~~FABRIK ile hedefe uzanma / foot-planting yürüyüş~~ ✅
3. ~~Gövde + 2 kol + 2 bacaktan oluşan tam iskelet, eklem açı sınırları~~ ✅
4. ~~Yürüyüş döngüsü ince ayarı: karşı-bacak kol sallanması, gravity/friction
   kalibrasyonu~~ ✅
   - **Ek düzeltme:** Pinned noktalar (`driver`, `l_anchor`, `r_anchor`)
     `_integrate()` içinde yanlışlıkla hâlâ hız/yerçekimiyle "entegre"
     ediliyordu; bu, hedefi karesel değişen (ör. itki profili ters
     yönlü) pinned noktalarda gözle görülür bir titremeye (jitter) yol
     açıyordu — omuz/kol titremesi buradan geliyordu. `_integrate()`
     artık pinned noktaları tamamen muaf tutuyor. Sayısal doğrulama: el
     noktasının kare-başı yer değiştirmesi ort. 6.5px/max 46px (34 yön
     değişimi) → düzeltme sonrası ort. 2.2px/max 8px (9 yön değişimi,
     doğal adım döngüsüyle uyumlu).
5. ~~Parallax arka plan katmanları (Z-index'e göre farklı kayma hızı)~~ ✅
   - **Ek düzeltme: "robotik" bacak yürüyüşü.** Kullanıcı geri bildirimi:
     dirsekler doğal ama bacaklar robot gibi hareket ediyordu. Kök neden
     sayısal olarak doğrulandı: `LEG_SEGMENT_LEN=75` ile bacağın toplam
     erişimi (`arm_length = 2*75 = 150px`) kalça-zemin dikey mesafesinden
     (`GROUND_Y - HIP_Y = 160px`) **azdı** -- yani stance fazının neredeyse
     tamamında ayak hedefi geometrik olarak erişilemezdi
     (`FabrikChain2D.is_reachable()` → `False`), bu da `solve()`'u sürekli
     "erişilemez hedefe doğru dümdüz ger" fallback'ine düşürüp dizin hemen
     hiç bükülmeden kalmasına (rijit "sopa bacak" görünümü) yol açıyordu.
     İki parça halinde düzeltildi:
     - `LEG_SEGMENT_LEN` 75 → 92 (`arm_length` 150 → 184, 160'ı rahatça
       aşıyor). Sayısal doğrulama (8 saniyelik tam yürüyüş simülasyonu,
       diz açısı = kalça→diz ve diz→ayak segmentleri arası sapma):
       düzeltme öncesi ort. 17.5°/maks 49° (std 15.2) → düzeltme sonrası
       ort. 37.8°/maks 84° (std 30.5), yani diz artık gerçek bir yürüyüşteki
       gibi belirgin şekilde bükülüyor.
     - `physics/gait.py`'de `FootPlantingLeg.update()`'in swing fazındaki
       x enterpolasyonu lineer yerine ease-in/ease-out (`smoothstep`,
       `3t²-2t³`) eğrisine çevrildi -- ayak kalkışta/inişte yumuşak
       hızlanıp yavaşlıyor, sabit hızlı bir "süpürme" yerine gerçek bir
       uzvun ataletine daha yakın. Bu değişiklik `step2`-`step5` arası
       `FootPlantingLeg` kullanan tüm demoları etkiler (paylaşılan modül).
     - Bu bug `step3`/`step4`'te de aynı sabitlerle mevcuttu (step2'de
       yoktu, çünkü o adımın `HIP_Y`/`GROUND_Y`/`LEG_SEGMENT_LEN` oranı
       zaten erişilebilirdi) -- üçü de aynı düzeltmeyle güncellendi.
     - Diz açısının asla tamamen 0'a kilitlenmemesi zaten `KNEE_LIMITS`
       (min 8°) ile garanti altındaydı; bu, düzeltmeden önce de "tam düz"
       görünmeyi hafifçe yumuşatıyordu ama erişilemezlik sorununun kendisini
       çözmüyordu.
6. ✅ Senaryo/timeline üretim katmanı: `scene/` +
   `demo/step6_scenario_timeline.py`; sprite, prop, kamera ve opsiyonel ses.
   Diyalog/lip-sync bu MVP'nin kapsamında değil.
7. 🔶 Momentum ve esneme (squash & stretch) — verlet zincirlerinin hıza/
   düşmeye tepki olarak uzayıp büzüşmesi. **Başlandı:**
   `physics/verlet.py`'ye `add_stick(..., compliance=...)` eklendi — bir
   çubuğun `_satisfy_sticks()` düzeltmesinin ne kadarının uygulanacağını
   (0=tam rijit/eski davranış, >0 kısmi) ayarlıyor; sınırlı iterasyon
   sayısında tam düzeltilemeyen çubuk momentum altında geçici esniyor.
   bkz. `demo/step7_squash_stretch.py`: halka+jant topolojili "yumuşak
   top" zemine düşüp çarpıyor. **Dürüst not:** tekdüze yerçekimi altında
   serbest düşüşte hiçbir deformasyon fiziksel olarak GERÇEKLEŞMEZ (tüm
   noktalar aynı ivmeyi alır) — squash/stretch SADECE zeminle çarpışma
   anındaki asimetriden (alt nokta durur, üst nokta düşmeye devam eder)
   ve ardından çubukların kısmi geri-yuvarlaklaşmasından (rebound) ortaya
   çıkıyor. Sayısal doğrulama (bounding-box yükseklik/genişlik oranı):
   dinlenme ~0.95 → çarpma anında min ~0.79 (yassılaşma) → rebound'da
   maks ~0.985 (neredeyse tam yuvarlak) → son 20 karede ort. 0.941 (std
   0.00002, tamamen sönümlenmiş). Sonraki olası genişletme: aynı
   `compliance` mekanizmasının karakterin kendi gövde/uzuv çubuklarına
   (ör. sert bir inişte gövdenin hafifçe sıkışması) uygulanması.
8. 🔶 Çoklu nokta çarpışması (multi-node collision & raycasting) — baş/omuz/
   kalça/ayaktan zemine ve engellere ışın taraması. **Başlandı (düz zemin
   versiyonu):** `VerletSystem.collide_ground(floor_fn, friction_fn)` —
   serbest her nokta kendi x'inde `floor_fn(x)` ile tanımlı zeminin altına
   sızarsa yüzeye geri itiliyor (aşağı doğru tek noktalı bir "raycast").
   bkz. `demo/step8_collision_friction.py`: 16 segmentlik uzun bir pelerin,
   karakter kısa süre durup serbestçe yere sarktıktan sonra, tekrar
   yürüyüşe geçince zeminin üzerinde/ altına gömülmeden sürükleniyor
   (`collide_ground()` olmadan noktalar zeminin altına sızardı — sayısal
   olarak doğrulandı: her karede `cape` noktalarının max y'si kesinlikle
   330'u (GROUND_Y) geçmiyor). Henüz genel raycast/engel geometrisi yok —
   sadece düz zemin; basamak/rampa gibi x'e bağlı yükseklik `floor_fn`
   imzasında zaten destekleniyor ama bu demoda kullanılmadı.
9. 🔶 Aktif/pasif ragdoll harmanı — IK güdümlü "aktif" durum ile tamamen
   yerçekimine teslim "pasif ragdoll" durumu arasında pürüzsüz geçiş.
   **Başlandı:** bacakların diz/ayak noktaları artık HEM ayrı bir
   `FabrikChain2D` ile IK-çözülüyor (aktif hedef) HEM DE ana
   `VerletSystem`'in sıradan rijit noktaları/çubukları olarak var (pasif
   fizik serbestçe evrilebilsin diye). Her karede: önce `body.step()` +
   `collide_ground()`'un DOĞAL sonucu ("pasif" konum) yakalanıyor, sonra
   IK'nin ürettiği ("aktif") konumla `blend*aktif + (1-blend)*pasif`
   olarak karıştırılıp geri yazılıyor. `blend=1`'de sonuç önceki
   adımlardaki yürüyüşle BİREBİR aynı; `blend=0`'da bacaklar/gövde
   tamamen serbest. Gövde/boyun `clamp_direction()`'ın izin verdiği
   maksimum açı da aynı `blend` ile 12°'den (aktif) 180°'ye (pasif,
   pratikte sınırsız) genişletiliyor — yani IK'nın yanı sıra gövde
   stabilizasyonu da birlikte devre dışı kalıyor. `driver` (kalça pin'i)
   pasifte kalçanın kendi bir önceki karedeki fizik-konumunu hedefliyor,
   yani kalça da serbest kalıyor. Senaryo: 3 saniye normal yürüyüş →
   0.6 saniyede "darbe" ile blend 1→0 → kontrol tamamen bırakılıyor.
   **Sayısal doğrulama:** aktif fazda `|aktif-pasif fark|` ort. ~3.9px
   (IK zaten fiziğe yakın bir yolda), pasif fazda ~103px'e çıkıyor (IK
   artık hiç uygulanmıyor, gövde kendi başına hareket ediyor). Gövde açısı
   aktifte sabit -12° (clamp limiti) iken pasifte -115°'ye kadar
   savruluyor. **Dürüst sınır:** pasif modda ek bir eklem sönümlemesi
   (joint damping) yok — sadece genel `friction` (pasifte 0.045'ten
   0.30'a çıkarılıyor) var, bu yüzden çöküş birkaç saniye boyunca
   (`physics/verlet.py`'de daha önce "aktif" modda `clamp_direction` ile
   ÇÖZÜLEN aynı kaotik çift-sarkaç fenomeni, burada pasifte BİLEREK geri
   getiriliyor) salınarak devam ediyor, sonunda dümdüz yatmak yerine
   dağınık/kıvrılmış bir yığın halinde yere yakın kalıyor — daha temiz bir
   yere-yatış için ek eklem sönümlemesi/limit sonraki olası iş. Görsel
   doğrulama ffmpeg kare çıkarımıyla yapıldı: aktif fazda tanıdık kontrollü
   yürüyüş, geçişten sonra net şekilde "kontrolü kaybetme" hissi var.
10. 🔶 İkincil fizik nesneleri (secondary animation) — kuyruk/kulak/pelerin
    gibi hedefsiz, saf verlet + hava direnci (drag) ile sarkan parçalar.
    **Başlandı:** bkz. `demo/step10_secondary_wind.py` (tek pelerin +
    rüzgar). Sonraki olası genişletmeler: kuyruk/kulak gibi başka örnekler,
    karakter durunca "bir süre daha salınmaya devam etme" davranışının
    ayrıca doğrulanması.
11. 🔶 Dinamik sürtünme/tutunma — zemine/eyleme göre değişen temas
    sürtünmesi (buzda kayma, bir kenara tutunma). **Başlandı, ama önemli
    bir dürüst sınır bulundu:** `collide_ground`'un `friction_fn(x)`'i bir
    noktanın HIZINI söndürür; bu, katı çubuklarla (stick) birbirine bağlı
    bir zincirde (ör. pelerin) neredeyse HİÇBİR görsel fark yaratmıyor,
    çünkü `_satisfy_sticks()` her karede noktayı komşusuna göre saf
    GEOMETRİK olarak yeniden konumlandırıyor ve sürtünmenin azalttığı hıza
    hiç bakmıyor (sayısal olarak ölçüldü: buzlu/normal bölgede pelerin
    ucu-kalça mesafesi ~91.4px'e karşı ~91.8px — pratikte ayırt edilemez;
    relaksasyon döngüsünün her iterasyonuna gömülmesi bile bunu
    değiştirmedi). Sürtünme, ANCAK hiçbir çubuğa bağlı olmayan gerçekten
    serbest bir parçacıkta (`demo/step8_collision_friction.py`'deki iki
    "taş" — biri normal zeminde, biri buzda, aynı anda aynı hızla
    fırlatılıyor) net bir şekilde görülüyor: normal zeminde ~13px kayıp
    duruyor, buzda ~117px kayıp çok daha yavaş duruyor (~9 kat fark,
    sayısal olarak doğrulandı). Sonraki olası iş: bacak/ayak temasına da
    (şu an sadece FABRIK hedefi olarak ele alınıyor) gerçek bir temas
    sürtünmesi kavramı eklemek isteniyorsa, muhtemelen `collide_ground`'u
    genişletmek yerine ayrı bir yaklaşım gerekecek.
12. 🔶 Kütle merkezi (center of mass) dengesi — denge kaybında kollarla
    counter-balance. **Başlandı:** bkz. `demo/step12_balance.py`. Her
    karede gövdenin (kalça+omuz+baş ortalaması — basitleştirilmiş bir
    "üst gövde" kütle merkezi vekili, tam kütle-ağırlıklı bir hesap
    DEĞİL) yatay konumu ile o an zeminde duran (stance) ayağın/ayakların
    yatay konumu arasındaki fark (`error`) hesaplanıp kollara bu farkla
    orantılı (negatif geri besleme) bir "dengeleme" ofseti uygulanıyor —
    hem yatay (geriye) hem dikey (yukarı kaldırma). t=3s'te bir tokezleme
    itkisi (`driver`'a ani bir ileri sıçrama) bu farkı büyütüp kolların
    tepkisini net şekilde gösteriyor. **Sayısal doğrulama:** tokezleme
    öncesi doğal yürüyüş salınımı ort. |error| ~8.65px (bu, her adımda
    destek ayağının değişmesinden kaynaklanan normal bir gidiş-gelme —
    tokezlemeye özgü değil), tokezleme sonrası PİK |error| ~38.8px'e
    çıkıyor (kol tepkisi ~21.3px), kol-ofseti ile `-error` arasındaki
    korelasyon 1.0000 (tam orantılı, uygulama doğru çalışıyor).
    **Dürüst sınır:** bu basit bir sezgisel geri besleme — gerçek bir
    ters-dinamik (inverse dynamics)/rigid-body coupling hesaplamıyor;
    kollar "doğru yöne" hareket ediyor ama bu iskelette kolların kütlesi
    gövdenin gerçek kütle merkezini fiziksel olarak geri ÇEKECEK kadar
    büyük/bağlı değil — yani dengesizliği düzeltmiyor, sadece görsel
    olarak DOĞRU YÖNDE ifade ediyor (oyun animasyonlarında yaygın bir
    "ucuz tepki jesti" tekniği). Sonraki olası iş: gerçek bir kütle-
    ağırlıklı CoM hesabı (kol/bacak kütleleri dahil) ve/veya ayak
    yerleşimini (capture point) buna göre ayarlayan bir denge kurtarma
    mekanizması.

## 2. tur kullanıcı geri bildirimi ve düzeltmeler

Kullanıcı, `step9`/`step13` videolarını izledikten sonra 5 ayrı sorun
bildirdi. Her biri önce sayısal bir tanılama script'iyle DOĞRULANDI, sonra
düzeltildi, sonra tekrar ölçüldü — hiçbiri "muhtemelen doğrudur" varsayılıp
körlemesine düzeltilmedi.

1. **"Zemin çarpışması sadece ayakta çalışıyor, el/pelerin zemine
   batıyor."** Kısmen yanlış bir öncülle doğru bir gözlem: `collide_ground()`
   MİMARİ OLARAK zaten TÜM pinned-olmayan noktalara uygulanıyordu (el/
   pelerin/kafa dahil — "sadece ayak" diye özel bir kod YOK, grep ile
   doğrulandı). Ölçülen ihlaller (step9: 17 kare/~7px, step13: 34 kare/
   ~17px) SADECE `r_foot`'ta çıktı, gerçek neden ise bir SIRALAMA
   hatasıydı: diz/ayağın aktif/pasif blend-overwrite'ı `collide_ground()`
   çağrısından SONRA yapılıyordu, yani o karenin zemin kontrolü bir kare
   geç kalıyordu. **Düzeltme:** `collide_ground()` blend-overwrite'tan
   SONRA bir kez daha çağrılıyor → iki demoda da ihlal sayısı 0'a indi.
   El/pelerin/kafa bu iki demoda zaten hiç ihlal etmiyordu (geometrik
   olarak zemine hiç yaklaşmıyorlar) — kullanıcının "el batıyor" gözlemi
   muhtemelen farklı bir sahne/anın yanlış hatırlanması ya da pelerinin
   gövdeyi kesmesiyle (madde 4) karıştırılmış olabilir.
2. **"Pasif (ragdoll) modda eklem açı sınırı yok, iskelet kırılıyor."**
   TAMAMEN DOĞRU, kod incelemesiyle doğrulandı: `clamp_joint_angles()`
   SADECE aktif `FabrikChain2D` üzerinde çağrılıyordu (`physics/gait.py`);
   pasif temsil (rijit `stick`'lerle bağlı serbest Verlet noktaları) hiç
   açı kısıtı almıyordu — ölçülen: tam pasif karelerde diz iç açısı
   0.3°-142.6° arasında (0°=dümdüz, 180°=tamamen katlanmış) TAMAMEN
   serbestti. **Düzeltme:** yeni `clamp_joint_angle_points()` ile pasif
   temsile de aktifle AYNI `KNEE_LIMITS` her zaman (blend'den bağımsız)
   uygulanıyor → ölçülen aralık artık tam olarak [8°, 150°].
3. **"Aktiften pasife geçişte momentum aktarılmıyor, karakter turuncu
   cisme çarpıp olduğu yerde çöküyor."** Öncül DÜZELTİLMELİ: kodda
   karakter ile bağımsız "yumuşak top" arasında HİÇBİR çarpışma/mesafe
   kontrolü YOK (grep ile doğrulandı, sadece `collide_ground` var) —
   `step13`'teki "darbe" `KNOCKDOWN_T=7.0`'da SAATE göre tetiklenen
   BAĞIMSIZ bir zamanlayıcı, topun konumuyla hiç ilgisi yok. Görsel
   çakışma (top o sırada yakında duruyor) muhtemelen bu izlenimi
   yaratmış. Ama alttaki genel iddia (momentum aktarılmıyor) DOĞRUYDU:
   ölçülen kalça vx'i geçişte 1.85'ten friction ile yumuşakça 0.34'e
   düşüyordu (darbe hissi yok). **Düzeltme:** geçişin başladığı karede
   `transition_impulse_vector()` ile kalça/omuz/kafaya karakterin KENDİ
   o anki hızının 4 katı + küçük bir yukarı kalkış enjekte ediliyor →
   vx artık 1.85'ten 3.14'e SIÇRIYOR, sonra doğal sönümleniyor.
4. **"İkincil animasyon (pelerin) gövdeyle çarpışmıyor, dengesiz."**
   TAMAMEN DOĞRU: projede HİÇBİR self-collision kodu yoktu (grep ile
   doğrulandı) — pelerin gövdeyi (`step13`'te 360 karenin 4-14'ünde
   ölçüldü) serbestçe kesiyordu. **Düzeltme:** yeni `physics/
   self_collision.py` ile pelerin gövde segmentlerinden itiliyor + ekstra
   hava direnci. **Dürüst sınır:** nokta-bazlı itme tam bir segment-segment
   kesişmeme GARANTİSİ vermiyor (min mesafe ~0.18px'ten ~2.2px'e çıktı,
   hedef 9px'e her zaman ulaşamıyor) — ince tek-zincirli nesneler için
   yeterli ama hacimli/kalın bir çarpışma modeli değil.
5. **"Kemik esnemesi: bacak lastik gibi geriliyor, diz tam düz kilitleniyor,
   kütle merkezi tepki vermiyor."** DOĞRU, iki katmanlı bir bulgu:
   - **(a) Beklenen edge-case:** hedef bacağın menzilini gerçekten aştığında
     (`FabrikChain2D.is_reachable()==False`) zincir düz bir çizgiye
     gerilip dizin bükülmesi neredeyse sıfıra iniyor — ölçülen: en kötü
     karede menzilin %99.76'sı. `KNEE_LIMITS` (min 8°) burada zaten devrede
     ama 8° görsel olarak hâlâ "kilitli" gibi görünüyor. **Düzeltme
     (kullanıcının önerdiği mimari):** `FootPlantingLeg.last_overrun_px`
     artık aşım miktarını dışa açıyor; `reach_pulldown_offset()` bacak
     yetişemediğinde kalçayı/gövdeyi aşağı+ileri eğerek hedefi bir sonraki
     karede GERÇEKTEN erişilebilir yapmaya çalışıyor (bacağı germek
     yerine) — bu sahnede ölçülen maks. aşım 23.4px, kalça buna orantılı
     tepki veriyor.
   - **(b) Yeni, daha temel bir bulgu (bu turda keşfedildi, DÜZELTİLMEDİ):**
     diz bükülmesi sadece "erişilemez" anlarda değil, NORMAL yürüyüşün
     HER karesinde de neredeyse tamamen düz (~8°) çıkıyor — ölçülen: saf
     aktif yürüyüşte (t<4s, hiç tokezleme/darbe yokken) her iki bacağın da
     iç açısı sabit 8.0° (KNEE_LIMITS'in minimum sınırı). Bunun nedeni,
     bacak segmentlerinin (92px×2=184px menzil) tipik adım genişliğine
     (~20-45px) göre ÇOK uzun olması: FABRIK, hedefe ulaşan en "tembel"
     (neredeyse düz) çözümü buluyor, çünkü açı için doğal bir tercih/önyargı
     yok. `KNEE_LIMITS`'in min açısını basitçe büyütmek (denendi: 8°→35°
     arası) DENENDİ ama YAN ETKİSİ ölçüldü: stance ayağının zeminden
     sapması 12.4px'ten (mevcut 8°'de bile zaten var) 47px'e kadar
     büyüyor (ayak havada süzülür gibi görünüyor) — yani bu basit
     değişiklik "lastik bacak"ı "havada yüzen ayak"la değiştiriyor,
     daha iyi değil. Gerçek çözüm muhtemelen FABRIK'in çözümüne bir
     "tercih edilen büküm açısı" önyargısı eklemek ya da bacak/adım
     oranlarını yeniden ayarlamak, ki bu TÜM `step2`-`step12` demolarının
     görünümünü etkiler — bu yüzden bu turda YAPILMADI, sadece dürüstçe
     belgeleniyor (projenin FPS/dt bağımlılığı bulgusuyla aynı desende).

**Doğrulama yöntemi:** her madde için önce/sonra sayısal bir tanılama
script'i (ground-violation sayacı, diz iç açısı istatistiği, kalça hız
logu, pelerin-gövde en yakın mesafe ölçümü, erişim aşımı) yazıldı, hem
`step9` hem `step13` üzerinde koşturuldu, videolar yeniden render edilip
ffmpeg ile kare kare görsel QA yapıldı. Hiçbir demo NaN/patlama üretmedi.

## 3. tur kullanıcı geri bildirimi ve düzeltmeler

Kullanıcı "bilgisayar mühendisliği perspektifinden" (nodelar/vektörler var
ama aralarında çalışacak kural setleri eksik) 4 sorun bildirdi, `fabrik.py`/
`gait.py`/ragdoll omurgası için somut kod önerileriyle birlikte. Aynı
yöntem: `demo/step13_full_integration_test.py` sahnesi üzerinde çalışan bir
tanılama script'i (`diag_round3.py` — bir kerelik, commit'e dahil değil)
yazıldı, her iddia önce/sonra sayısal olarak ölçüldü.

1. **"Flamingo bacağı: özellikle 3.2s ve 4.2s'de diz tersine bükülüyor."**
   İDDİA EDİLEN HALİYLE DOĞRULANMADI, ama daha temel bir bulguya çıktı:
   t=3.2s ve 4.2s'deki ham (clamp öncesi) diz açıları her iki bacakta da
   beklenen işaret aralığındaydı (sol: -74.6°..-50.4°, sağ bu iki anda
   -9°..-12° civarı) — tam bu iki karede görünür bir ters bükülme YOK.
   Ama tüm 12 saniyelik sahne taranınca gerçek bir kırılganlık bulundu:
   sağ bacağın ham açısı 360 karenin 128'inde (%36) POZİTİF çıkıyordu
   (FABRIK'in doğal çözümü dizi anatomik olarak yanlış tarafa koyuyordu)
   ve eski kod bunu `clip(açı, -max, -min)` ile en yakın SINIRA (-8°,
   neredeyse düz) sıçratıyordu — süreksiz bir "pop". Bu, projenin daha
   önce ERTELENMİŞ "tembel FABRIK diz" bulgusuyla (bkz. madde 5b, 2. tur)
   AYNI kök nedene çıktı. **Düzeltme:** `clamp_joint_angles()` /
   `clamp_joint_angle_points()` artık yanlış-taraf açısını sınıra
   kenetlemek yerine, büküm BÜYÜKLÜĞÜNÜ koruyarak doğru tarafa YANSITIYOR
   (`magnitude = clip(|açı|, min, max); clamped = ±magnitude`) — kullanıcının
   "açı 180'i geçtiği an kodun dizi zorla doğru tarafa katlaması" önerisinin
   süreklilik-koruyan (reflect) hali. **Beklenmeyen ama sayısal olarak
   doğrulanmış yan etki:** bu tek değişiklik, 2. turda ERTELENMİŞ "diz her
   karede sabit ~8°" sorununu da düzeltti — aktif yürüyüşte nihai (clamp
   SONRASI) diz açısı artık -8.0°'den -84.7°'ye kadar doğal bir dağılımla
   değişiyor (ortalama ≈-62°, std≈9-10°; önceden 210 karenin 209'u ~8°
   civarındaydı, şimdi sadece 1'i).
2. **"Omuz ve dirsekler göğüs kafesinin içinden geçiyor, kol 360° dönebiliyor."**
   TAMAMEN DOĞRU: kollar sadece sabit uzunluklu çubuklarla omuza bağlıydı,
   HİÇBİR açısal sınır yoktu (grep ile doğrulandı — pelerin için
   `self_collision.py` vardı, kollar için hiçbir mekanizma yoktu). Ölçülen:
   AKTİF yürüyüşte bile sağ kol/gövde en yakın mesafesi 210 karenin
   151'inde 10px'in altına (bazı karelerde tam 0px'e, yani gövdenin
   üzerine) düşüyordu. **Düzeltme (kullanıcının önerdiği iki katman
   birlikte uygulandı):** (a) "Ulaşım Konisi" — omuz→dirsek ve dirsek→el
   yönleri gövde eksenine göre `clamp_direction()` ile sınırlanıyor
   (yeni kod değil, gövde/boyun için zaten var olan AYNI jenerik
   fonksiyonun tekrar kullanımı); aktifte dar (45°/55°), pasifte gevşek
   (100°/120°) — bkz. `blended_max_angle()`. (b) Nokta-vs-segment itme —
   pelerinde kullanılan `push_points_off_segment()` artık kol için de
   (gövdenin iki segmentine karşı) çağrılıyor, min. mesafe 11px.
   **Sonuç:** aynı sahnede kol/gövde en yakın mesafesi artık HİÇBİR karede
   11px'in altına inmiyor (önce: 88/360 kare <5px, şimdi: 0/360).
3. **"Ağırlık transferi ve ayak sürüklenmesi: adım atarken ayak buz pateni
   gibi düz bir çizgide kayıyor."** İDDİA EDİLEN HALİYLE DOĞRULANMADI:
   swing fazı zaten `sin(πt)` ile ayağı yerden açıkça kaldırıyordu ve
   STANCE fazında sol ayak zaten tam sabitti (std=0.0000px — hiç
   kaymıyordu). Ama sağ ayakta GERÇEK, farklı bir kusur bulundu:
   `clamp_joint_angles()`'ın diz açısını düzeltirken uç-efektörü (ayağı)
   yan etki olarak kaydırması yüzünden (bkz. fonksiyonun kendi dokstring'i
   — "uç-efektör hedefe tam ulaşamayabilir"), STANCE sırasında bile sağ
   ayak std=4.31px titriyordu (bazı karelerde zeminden ~12.5px sapma) —
   "kayma" değil ama gözle benzer bir izlenim verebilecek bir titreme.
   **Düzeltme:** (a) kullanıcının önerdiği gibi swing artık bağımsız
   ease(x)+sin(y) yerine TEK bir ikinci-derece Bezier eğrisiyle
   (`_quadratic_bezier()`) hesaplanıyor — kontrol noktası düz çizginin
   ortasının `2×lift_height` üstünde; (b) STANCE fazında diz-açısı kısıtı
   uygulandıktan HEMEN SONRA ayak (`chain.points[-1]`) her zaman tam
   olarak `self.planted` hedefine yeniden kenetleniyor. **Sonuç:** iki
   ayağın da stance std sapması artık tam 0.0000px.
4. **"Ragdoll'da omurga 8.2s sonrası pasife geçince anında 90° kırılıp
   kendi içine çöküyor."** YÖN OLARAK DOĞRU, rakam biraz farklı ama ÖZÜNDE
   doğrulandı: `blended_max_angle()` pasif modda varsayılan
   `passive_max_deg=180.0` (pratikte sınırsız) kullanıyordu — ölçülen:
   knockdown sonrası gövde açısı t=7.6s'de 24.4°'den t=10.0s'de 120.7°'ye
   savruluyordu (gerçek bir omurganın yapamayacağı bir katlanma miktarı;
   7.6→8.2s arası tek başına +16°). **Düzeltme:** kullanıcının "Verlet
   yaylarına sertlik atanması" önerisinin bu mimarideki (point-stick,
   gerçek açısal yay YOK) en doğrudan karşılığı — demo'larda artık
   `blended_max_angle()`'a pasif için sonsuz (180°) yerine SONLU, gevşek
   bir tavan veriliyor (gövde 75°, boyun 85°) — hâlâ aktiften çok daha
   serbest (ragdoll hissi korunuyor) ama fiziksel olarak imkânsız
   tam-katlanmaya izin vermiyor. **Sonuç:** aynı sahnede gövde açısı
   artık ±75°'yi hiç aşmıyor. **Dürüst sınır:** bu gerçek bir açısal
   yay/sönümleme (spring/damping) DEĞİL, hâlâ projenin `clamp_direction()`
   tabanlı sert-sınır (hard-clamp) yaklaşımı — sadece tavanı 180'den
   düşürüyor; iki nokta arasında gerçek bir tork/geri-çağırma kuvveti
   uygulayan bir "Verlet açısal yayı" bu basit motor için henüz yok
   (olası bir sonraki iş).

**Doğrulama:** `diag_round3.py`, `step13` sahnesini önce/sonra çalıştırıp
4 maddeyi de sayısal olarak ölçtü (yukarıdaki rakamlar oradan). Ayrıca
TÜM demo dosyaları (`step1`-`step13`) yeniden çalıştırılıp (a) hiçbirinin
hata/NaN üretmediği, (b) bu turun değişikliklerinden ETKİLENMEMESİ
gereken step7/step12'nin sayısal çıktılarının (0.794/0.985/0.941 squash
oranları; 8.65px/38.80px/1.0000 denge korelasyonu) ÖNCEKİ turla BİREBİR
AYNI kaldığı doğrulandı. t=3.2s/4.2s/8.2s kareleri ayrıca ffmpeg ile
görsel olarak da incelendi (diz artık doğal bükülüyor, kol gövdeye
11px'den yakın durmuyor, çöküş sonrası gövde ~-63° civarında kalıp 90°+
katlanmıyor).

## 3. tur eki: omurgaya tork ve yay-sönümleme (Hooke Yasası)

Kullanıcının 3. tur bulgularını değerlendirdiği takip mesajında (capsule
collision / kütle dağılımı yerine) açıkça önceliklendirdiği tek madde:
**"mevcut yapıyı kırmadan"** omurga/boyun için gerçek bir açısal
yay-sönümleme (spring-damping, $F=-kx-cv$) mekanizması. Motor yeniden
yazılmadı; `physics/verlet.py`'a tek bir fonksiyon (`apply_angular_spring`)
eklendi ve mevcut `clamp_direction()` sert-tavan çağrılarının HEMEN
ÖNCESİNE, aynı iki nokta üzerinde ek bir adım olarak eklendi (var olan
`blended_max_angle()`/`lerp_blend()` harman mantığı AYNEN korunuyor — sert
tavan hâlâ orada, sadece artık yay ÖNCE yumuşakça direnç gösteriyor).

**Nasıl çalışıyor (dürüst sınırlarıyla):** `apply_angular_spring()`,
`clamp_direction()` ile AYNI imzalı (points/prev_points, iki nokta, bir
referans yön) ama pozisyonu SINIRLAMAK yerine bir açısal tork hesaplayıp
bunu `apply_impulse()`'taki gibi `prev_points`'i kaydırarak enjekte
ediyor (konuma asla dokunmuyor, sadece bir sonraki karenin hızını
etkiliyor — ani pozisyon sıçraması riski yok). Açısal hız, persistan bir
durum (state) TUTMADAN, o karedeki göreli Verlet hızının teğetsel
izdüşümünden türetiliyor. **Dürüst sınır:** bu gerçek bir rijit-cisim
torku/eylemsizliği DEĞİL — kütle/eylemsizlik momenti yok, sadece "bu iki
nokta arasındaki açı sapması ve açısal hıza orantılı bir düzeltici ivme"
uyguluyor; gerçek bir fizik motorundaki tork'un basitleştirilmiş bir
yaklaşıklaması.

**Sabitler** (`demo/step9_ragdoll_blend.py` ve `step13_full_integration_test.py`'de
BİREBİR aynı, aktifte ikisi de 0 — yay SADECE pasif/ragdoll'da devrede):
`PASSIVE_TORSO_STIFFNESS=0.025`, `PASSIVE_TORSO_DAMPING=0.40`,
`PASSIVE_NECK_STIFFNESS=0.015`, `PASSIVE_NECK_DAMPING=0.30`. İlk
denemede (stiffness=0.15/damping=0.35) karakter neredeyse dikey knockdown
duruşuna geri çekilip düşemiyordu (görsel olarak havada asılı kalıp
bacaklar çapraz kilitleniyordu) — **sönüm-ağırlıklı** bir ayara geçildi
(düşük stiffness, yüksek damping), çünkü bilinçsiz bir bedenin gerçek
kas tonusu yerçekimine karşı AKTİF geri-çağırma kuvveti değil, ÇOĞUNLUKLA
hız-sönümleme (damping) sağlar.

**Sonuç (rest açısı knockdown anına kilitlenip ölçüldü):** gövde açısı
knockdown'da -12.0°'den başlayıp t=8.2s'de +16.1°'ye YUMUŞAKÇA yükseliyor,
t=10.2s'de +23.8°'lik bir tepe yapıp yön değiştiriyor, t=11.0s civarında
tekrar 0°'ı geçip t=13.0s'de -58.8°'ye iniyor — yani gerçek bir SÖNÜMLÜ
SALINIM (damped oscillation): yükselip tepe yapıp geri dönme davranışı,
2. tur'un sert-tavan-SADECE yaklaşımında YOKTU (orada açı tek yönde
sürekli artıp doğrudan ±75° duvarına ÇARPIP orada DÜZ kalıyordu, bkz. bir
önceki bölümdeki "-12.0'den 120.7'ye" ölçümü). Tüm süreç boyunca ±75°
güvenlik duvarının dışına HİÇ çıkmadı (yay zaten duvara çarpmadan önce
yönü çeviriyor).

### Yan bulgu: yay çalışmasını doğrularken bulunan GERÇEK bir süreklilik hatası (ve düzeltmesi)

Yayı görsel olarak doğrularken (ffmpeg kareleri), ragdoll'un bacaklarının
zaman zaman havada "asılı/donmuş" göründüğü fark edildi — sayısal tanı
(`clamp_joint_angle_points` çağrısından hemen önce ham açıyı loglayan
geçici bir script) şunu ortaya çıkardı: knockdown darbesinden hemen sonra
sağ dizin ham (kısıtlanmamış) açısı fiziksel olarak sürekli +82°'den
+174°'ye yükseliyordu (bacağın momentumla savrulması — GERÇEK bir fizik
olayı). Ama `fabrik.py`'deki 3. tur "yansıtma" (reflect) düzeltmesi --
`magnitude = clip(|açı|, min, max); clamped = ±magnitude (işaret
`bend_sign`'a ZORLA sabitlenerek)` -- HER KAREDE bu büyük açıyı ZORLA ters
işarete çeviriyordu (ör. +142.83° → -142.83°), bu da dizin etrafında
TEK KAREDE ~74°-164°'lik GÖRÜNÜR bir sıçramaya yol açıyordu. Düzeltme
ayrıca `prev_points`'i yeni konuma eşitlediği için (mevcut kod
kuralı, bkz. `clamp_direction()` docstring'i) düzeltilen ayağın hızı da
sıfırlanıyor, yani ayak birkaç kare boyunca "donmuş" kalıp yavaşça
yeniden düşmeye başlıyordu (ölçüldü: sağ ayak y=304px'ten y=138px'e
~0.4s'de sıçrayıp oradan ~1.1s boyunca düşmedi).

Bu, 3. tur'un kendi "süreklilik her koşulda korunur" iddiasını ihlal eden
GERÇEK bir kenar-durum hatasıydı (küçük yanlış-taraf açılarında —
docstring'in kendi örneği +5°→-8°'de — sorunsuzdu, sadece BÜYÜK
yanlış-taraf açılarında/yüksek açısal hızda ortaya çıkıyordu — normal
yürüyüşte nadiren, güçlü bir darbeden sonra sıklıkla). **Düzeltme:**
"büyüklüğü koru, işareti zorla" yerine, geçerli açı aralığının (`[min,
max]` büyüklüğünde, `bend_sign` işaretli bir yay/arc) ÇEMBER ÜZERİNDEKİ
EN YAKIN UÇ NOKTASINA katlama (`_nearest_valid_bend_deg()`,
`physics/fabrik.py`) — bir aralığın dışındaki bir noktaya en yakın geçerli
nokta her zaman o aralığın uçlarından biridir (temel geometri), bu da
küçük açılarda ESKİ davranışla BİREBİR aynı sonucu verirken (+5°→-8°
değişmedi) büyük/yüksek-hızlı durumda düzeltmeyi ~67°'ye indiriyor (164°
yerine) ve gerçek bir sıçrama üretmiyor.

**Doğrulama:** aynı diagnostic script'in düzeltme sonrası çıktısı --
sağ diz artık aynı pencerede pürüzsüz, tek yönlü bir düşüş sergiliyor
(304px → 330px'e yumuşakça, sıçramasız), ham açı -6° civarında sabitleniyor
(sert sıçrama yok, ardışık kareler arası fark <1°). Yan etki olarak AKTİF
(IK ile yürüyen) bacağın kendi ham diz açısı aralığı da düzeldi: aynı
`diag_round3.py` testinde eskiden [-173°, +174°] (neredeyse tam çember,
daha önce fark edilmemiş bir kararsızlık) iken düzeltmeden sonra
[-58°, +58°]'e daraldı -- yani bu hem ragdoll'u hem de orijinal round-3
"flamingo bacağı" düzeltmesinin kendisini daha sağlam hale getirdi. Tüm
`step1`-`step13` regresyon paketi (3 FPS'te step13 dahil) yeniden
çalıştırılıp hiçbir NaN/patlama üretmediği, step7/step12'nin sayısal
çıktılarının (0.794/0.985/0.941; 8.65px/38.80px/1.0000) değişmediği
doğrulandı.

### Dürüst sınır (yeni bulunan, henüz DÜZELTİLMEMİŞ bir sorun)

Sıçrama hatası düzeldikten SONRA bile, tam pasif (ragdoll) düşüş
sırasında bacaklardan biri zaman zaman neredeyse düz (min. büküm ~8°
sınırına yakın) bir pozda "donup" yere düşmüyor -- ekranda bacakların
geniş bir "V" (cimnastik splits) şeklinde açık kaldığı görülebiliyor
(hem `step9` hem `step13`'te gözlemlendi, bu yüzden yeni omurga yayından
DEĞİL, paylaşılan pasif-bacak mimarisinden kaynaklanıyor). **Kök neden:**
`clamp_joint_angle_points()`'in düzeltme uyguladığı HER karede
`prev_points`'i de yeni konuma eşitlemesi (hızı sıfırlaması) --
`clamp_direction()`'la paylaşılan, KASITLI bir tasarım kuralı ("sahte hız
sıçraması yaratmasın" diye) -- bacağın doğal açısı sınırın (8°) hemen
dışında SABİT kalırsa (yani düzeltme NEREDEYSE HER karede tetiklenirse),
yerçekiminin bacağa kazandırmaya çalıştığı açısal hız her karede
sıfırlanıp bacak fiilen "frenleniyor". Bu, `gait.py`'de zaten dokümante
edilmiş küçük ölçekli bir sınırlamanın (aktif stance ayağında birkaç
piksel "titreme", bkz. yukarıdaki yorum) ragdoll'da çok daha büyük
ölçekte ortaya çıkan hâli. **Neden şimdi düzeltilmedi:** gerçek düzeltme
ya kalıcı durum (state) tutan bir süreklilik-farkındalı yumuşatma ya da
tam bir eklem-kısıtı fizik motoru (yay-tabanlı joint limit + restitution)
gerektiriyor -- kullanıcının kendi önceliklendirmesiyle (bu tur SADECE
omurga yay-sönümleme, capsule collision/kütle dağılımı GELECEK tur)
tutarlı şekilde, motoru bu turda daha fazla büyütmek yerine burada
DÜRÜSTÇE bir sınır olarak bırakılıyor -- olası bir sonraki iş (capsule
collision ile aynı pakette ele alınabilir, ikisi de "gerçek eklem
kısıtı/çarpışma fiziği" kategorisine giriyor).

## 4. tur kullanıcı geri bildirimi ve düzeltmeler

Kullanıcı, 3. tur eki (omurga yay-sönümleme) sonrası entegrasyon videosunu
izleyip 4 yapısal sorun bildirdi: (1) `_nearest_valid_bend_deg()`'in (3.
tur eki, bkz. yukarıdaki "Dürüst sınır") aktif yürüyüş bacağının ham diz
açı aralığını da `[-58°,+58°]`'e daraltmasının "kazık yutmuş gibi" robotik/
peg-leg bir yürüyüşe yol açması; (2) `clamp_joint_angle_points()`'in HER
düzeltmede `prev_points`'i sıfırlamasının (yukarıdaki "Dürüst sınır"da
zaten tespit edilmiş V-splits/donmuş-bacak sorunu) yerçekimini fiilen
iptal etmesi; (3) tüm Verlet noktalarının eşit kütleymiş gibi davranması
("kağıt bebek etkisi" -- ağır bir gövdenin hafif bir bacağı gerçekçi
şekilde sürükleyememesi); (4) nokta-tabanlı öz-çarpışmanın yüksek momentumlu
ragdoll flailing'inde kırılganlaşması (bu turda ele ALINMADI -- capsule
collision hâlâ gelecek iş). Kullanıcı (2) ve (1)'in BİRLİKTE çözülmesi
için somut bir teknik reçete verdi: önce hızı-sıfırlayan kısıtlamaları
momentum-koruyan hale getir, SONRA diz açı aralığını doğal anatomik
sınırlara geri çek, EN SONDA kütle hiyerarşisini kur.

### (1) + (2): Momentum-koruyan kısıtlama ve anatomik diz aralığının geri verilmesi

**`clamp_joint_angle_points()`** (`physics/fabrik.py`, sadece pasif/ragdoll
bacaklarda kullanılıyor): "ışınlama + hız sıfırlama" yerine, uygulanan
konum düzeltmesi (`correction = new_end - eski_end`) hem `points[i_end]`'e
hem de `prev_points[i_end]`'e AYNI miktarda ekleniyor. Bu, `points -
prev_points` farkının (Verlet hızı) düzeltmeden ÖNCEKİ değeriyle
MATEMATİKSEL OLARAK AYNI kalmasını sağlıyor -- kısıtlama artık sadece
YÖNÜ düzeltiyor, hızı yemiyor. Açı-katlama mantığının kendisi
(`_nearest_valid_bend_deg()`, 3. tur eki) korundu (şiddetli darbeler
altında hâlâ anti-teleport koruması gerekiyor) -- sadece SONUCUN nasıl
uygulandığı değişti.

**`FabrikChain2D.clamp_joint_angles()`** (`physics/gait.py`'nin kullandığı
AYRI kod yolu, sadece AKTİF/IK bacaklarda kullanılıyor -- `prev_points`/hız
kavramı YOK, her karede sıfırdan çözülüyor): `_nearest_valid_bend_deg()`
kullanımı GERİ ALINDI, orijinal büyüklük-koruyan-ayna formülüne
(`magnitude = clip(|açı|, min, max); clamped = ±magnitude`) dönüldü.
**Neden iki farklı fonksiyon iki farklı şekilde davranıyor:** bu fonksiyonun
noktaları her karede DEVAM EDEN bir hedeften yeniden çözülüyor, `prev_points`
yok, yani hız-donma riski hiç YOK -- 3. tur'daki "yansıtma" sıçrama hatası
(bkz. yukarı) sadece pasif/ragdoll tarafında geçerliydi. `_nearest_valid_
bend_deg()`'i BURAYA da uygulamak, sıçrama riskini gidermeden derin diz
bükülmelerini engelleyip peg-leg yürüyüşe yol açıyordu (kullanıcının
şikayeti). Diz açı aralığı sabitleri (`KNEE_LIMITS`) de `(8.0, 150.0)`
büyüklük aralığına genişletildi (kullanıcının "örneğin `[-10°,+140°]`"
önerisiyle aynı yönde -- akıcı "Rain World" yürüyüşü geri getirmek için).

**Doğrulama:** `git stash` A/B karşılaştırmasıyla, aktif SOL bacağın ham
diz açı aralığının orijinal round-3 (`7bef2d4`) taban çizgisiyle BİREBİR
aynı (`min=-82.4, max=0.7`) olduğu doğrulandı. ffmpeg kareleriyle görsel
doğrulama: `step9`'un AKTİF fazında (t=2.0s/3.0s) bacaklar artık derin,
doğal bir diz bükümüyle yürüyor (peg-leg YOK); GEÇİŞ anında (t=3.5s/4.0s)
bacaklar simetrik bir V-splits'e KİLİTLENMEDEN, asimetrik/akıcı bir pozla
pasif faza devam ediyor. `step7`/`step12` canary sayıları (0.794/0.985/
0.941; 8.65px/38.80px/1.0000) DEĞİŞMEDİ.

`clamp_direction()` (gövde/boyun/kol "güvenlik duvarı") BU turda da
KASITLI OLARAK değiştirilmedi -- aşağıdaki kütle bölümünde bunun neden
GEREKLİ bir sınır (ve aslında tam tersi yönde bir keşfe yol açtığı)
açıklanıyor.

### (3): Kütle hiyerarşisi -- ve bulunan GERÇEK bir kararsızlık

`VerletSystem.add_point(..., mass=...)` eklendi; `_satisfy_sticks()`'teki
çubuk düzeltmesi artık 50/50 sabit değil, standart PBD nokta-kütle
formülüyle (`w=1/kütle`; `frac_i=w_i/(w_i+w_j)`) TERS KÜTLEYLE
ağırlıklanıyor -- ağır nokta az hareket eder, hafif nokta farkı kapatmak
için çok hareket eder. `mass` verilmezse ya da iki nokta eşit kütledeyse
`frac=0.5` -- ESKİ davranışla birebir aynı (`step7`/`step12` canary'leriyle
doğrulandı, kütlesiz/eşit-kütleli HİÇBİR demo etkilenmedi). **Dürüst
sınır:** yerçekimi hâlâ kütleden bağımsız bir ivme (`F=ma`, `a=g` kütleden
bağımsız, gerçek fizikte de doğru) -- kütlenin GÖRÜNÜR etkisi SADECE çubuk
kısıtlaması ihlal edildiğinde ortaya çıkıyor; gerçek bir rijit-cisim
eylemsizlik-momenti simülasyonu değil.

**İlk denemede** (`MASS_HIP=4.0, MASS_SHOULDER=3.0, MASS_HEAD=1.2,
MASS_ELBOW=0.5, MASS_HAND=0.3, MASS_KNEE=0.8, MASS_FOOT=0.4` -- kalça:diz
oranı 5:1) tam entegrasyon hattında (gait + ragdoll + omurga yayı +
`clamp_direction()` + `collide_ground`) karakter EKRANDAN YUKARI FIRLAYIP
gitti (hip Y-konumu ~ -465px, `zemin=330`'a göre; NaN/patlama DEĞİL, ama
fiziksel olarak saçma bir enerji kazanımı). Bu, kod git'e commit edilmeden
ÖNCE, projenin kendi regresyon adımıyla yakalandı.

**İzolasyon süreci** (birer birer devre dışı bırakma/geri alma):
`RELAX_ITERS`'i 8'den 64'e çıkarmak HİÇBİR fark yaratmadı (yani sorun
kare-içi PBD yakınsaması değil, KARELER ARASI enerji birikimi). Diz
kelepçesini (`clamp_joint_angle_points`) tamamen kaldırmak sorunu
DÜZELTMEDİ, KÖTÜLEŞTİRDİ. `apply_angular_spring()`'i devre dışı bırakmak
da KÖTÜLEŞTİRDİ. Gövde/boyun/kol `clamp_direction()` çağrılarının HEPSİNİ
devre dışı bırakmak sorunu TAMAMEN durdurdu (tüm noktalar temiz şekilde
zemine, y=330'a yerleşti) -- ve tek başına SADECE kalça→omuz (`UP`
referanslı gövde eğim) çağrısı bile aynı firlamayı tek başına
üretebiliyordu. `clamp_direction()`'a `clamp_joint_angle_points()`'teki
AYNI momentum-koruma tekniğini uygulamak (iki farklı yöntemle denendi:
düz-öteleme ve rotasyonel/açısal-hız-koruma) sorunu ÇÖZMEDİ -- yani mesele
"hızı nasıl koruyoruz" değildi. Kalça/omuz kütlelerini tek tek 1.0'a geri
almak sorunu KISMEN azalttı ama gidermedi; asıl belirleyici kütle çiftinin
DİZ/AYAK olduğu (bacak zincirinin `hip→knee→foot` bağlantısı) izole
edildi. İkili aramayla (bisection): kalça:diz oranı ~2.67:1'de
(`MASS_KNEE=1.5`) KARARLI, ~3.33:1'de (`MASS_KNEE=1.2`) TEKRAR KARARSIZ --
yani gerçek bir sayısal kararlılık EŞİĞİ var, bu motorun (sabit 8 iterasyonlu
Gauss-Seidel PBD, kalçanın 4 çubuğa bağlı olması) doğal bir sınırlaması.

**Sonuç:** bacak kütleleri, yönelim (ağır gövde/hafif uç -- kullanıcının
istediği "kağıt bebek" etkisinin tersi) korunarak ama kararlı eşiğin
güvenli bir marjı altında ÇÖZÜLDÜ: `MASS_KNEE=1.6, MASS_FOOT=1.0` (kalça:
diz oranı 2.5:1). Diğer tüm kütleler (`HIP=4.0, SHOULDER=3.0, HEAD=1.2,
ELBOW=0.5, HAND=0.3`) değişmedi -- kol zinciri (`anchor→elbow→hand`)
zaten pinned bir anchor'a bağlı ve dahili oranı (0.5:0.3≈1.67:1) hiç
sorun çıkarmadı, tehlike SADECE kalça-diz bağlantısındaydı.

**Doğrulama:** `step9` ve `step13`'ün TAMAMI (step13 için 24/30/60 FPS
stres testi dahil) yeniden çalıştırılıp hem `any_nan=False` hem de HER
karede TÜM noktaların `|Y|`'sinin makul bir aralıkta kaldığı (step13'te
maksimum 354px, sadece başlangıç pozunda -- ragdoll firlaması YOK) ayrıca
DOĞRULANDI (mevcut `step13`'ün kendi NaN-kontrolü bu tarz "sonlu ama saçma"
bir patlamayı YAKALAMAZDI, bu yüzden ek bir manuel Y-sınırı kontrolü
yapıldı). ffmpeg kareleriyle görsel doğrulama: ragdoll çöküşünde gövde/
kafa artık bacaklardan gözle görülür şekilde daha "ağır" davranıyor.
`step7`/`step12` canary sayıları yine DEĞİŞMEDİ.

### Dürüst sınırlar (bu turda ELE ALINMAYAN/tam çözülmeyen sorunlar)

- **Öz-çarpışma (kullanıcının 4. maddesi):** nokta-tabanlı öz-çarpışma
  (`push_points_off_segment()`) bu turda hiç değiştirilmedi -- capsule
  collision hâlâ ayrı bir gelecek iş olarak duruyor.
- **`clamp_direction()`'ın kendisi hâlâ hız-sıfırlıyor:** yukarıdaki
  izolasyon sürecinde momentum-koruma denendi ve kararlılığı DÜZELTMEDİĞİ
  için GERİ ALINDI -- yani gövde/boyun/kol için "hızı sıfırlayan
  kısıtlamalar" eleştirisi TAM olarak giderilmedi, sadece (a) asıl somut
  şikayet konusu olan bacak/diz tarafında giderildi ve (b) kütle
  hiyerarşisi bu fonksiyonu DEĞİŞTİRMEDEN, sadece bacak kütle oranını
  kısıtlı tutarak güvenli hale getirildi. Gövde/boyun için gerçek bir
  momentum-koruyan çözüm hâlâ AÇIK bir problem (muhtemelen `clamp_
  direction()`'ı özünden mass-aware/iki-taraflı bir kısıtlamaya çevirmeyi
  gerektiriyor -- tek başına denenip işe yaramadı, bkz. yukarı).
  Kararlılık eşiği (2.67:1 civarı) da EMPİRİK bulundu, kapalı-form bir
  türetim değil -- daha büyük/karmaşık iskeletlerde yeniden ölçülmesi
  gerekir.
- **Tam pasif düşüşte bazen "yere tam yerleşmeyen, havada asılı kalan"
  bir tümbling pozu** (bkz. `step9`/`step13`'ün geç karelerinde bacakların
  yukarı-dışa açık kaldığı gözlemi) -- bu davranış kütle hiyerarşisinden
  ÖNCE de (kütlesiz/eşit-kütleli baseline'da da) AYNEN mevcuttu, yani BU
  turun bir regresyonu DEĞİL, önceden var olan ayrı bir sınırlama
  (muhtemelen basit 2 bacaklı ragdoll'un tam "yatarak dinlenme" durumuna
  hiç ulaşamaması, öz-çarpışma/eklem-limiti eksikliğiyle ilişkili
  olabilir) -- bu turun kapsamı dışında bırakıldı.


## 5. tur kullanıcı geri bildirimi ve düzeltmeler

Kullanıcı, 4. tur commit'inden (2.67:1 kütle oranının "kararlı" bulunduğu
rapor) sonra ilkesel bir itiraz getirdi: bulunan oranın (2.5:1, `MASS_KNEE=
1.6`) "bir çözüm değil, bir bantlama" olduğunu, çünkü altta yatan sorunun
Gauss-Seidel/sıralı Verlet kısıtlama çözücüsünün yapısal bir kusuru
olduğunu savundu -- gelecekte daha ağır bir ekleme (ör. kapsül çarpışması,
aksesuar) yapıldığında aynı kararsızlığın FARKLI bir eşikte geri
döneceğini öngördü. Kullanıcı üç somut teknik hipotez önerdi: (a)
`clamp_direction()`'ın kararsızlığının hız-sıfırlama YÖNTEMİYLE değil,
düzeltmenin anchor/free arasında kütle-ağırlıklı PAYLAŞILMAMASIYLA ilgili
olduğu; (b) "V-splits" pozunun sabit bir kısıtlama çalıştırma SIRASININ
açı ve mesafe kısıtlamalarını sonsuza kadar birbirine karşı "kilitlemesi"
("Constraint Iteration Order"); (c) tüm kısıtlama sistemini kütle-ağırlıklı
hale getirerek kökten çözüm. İstek: kapsül çarpışmasını bir tur daha
erteleyip önce bunu araştırmak.

### Metodolojik kusur itirafı: yetersiz doğrulama süresi

Bu araştırmaya başlamadan önce dürüstçe belirtilmesi gereken bir şey:
4. turun "2.67:1'de KARARLI" doğrulaması SADECE demoların varsayılan
13 saniyelik süresinde test edilmişti. 60+ saniyelik UZATILMIŞ testler
(bu tur için özel olarak eklendi -- `demo.N_FRAMES` çalışma-zamanında
`FPS*60`'a ezilerek, dosyaya DOKUNMADAN) 2.5:1 oranının aslında
t≈17-25s civarında başlayan, sabit ~87px/saniyelik DOĞRUSAL bir kalça-Y
kaçışına (karakter ekranın yukarısına doğru sonsuza sürüklenir) hâlâ
sahip olduğunu ortaya çıkardı -- yani 4. turun "kararlı" bulgusu bir
DÜZELTME değil, sadece bir GECİKMEYDİ. Bu, doğrulama sürecindeki gerçek
bir eksikti: kısa süreli demo videosu yavaş biriken bir sayısal kaymayı
gizleyebiliyor. **Yeni kural: bundan sonraki her kararlılık iddiası
en az 60 saniyelik uzatılmış bir koşuyla test edilmeli.**

### Elenen hipotezler (sistematik izolasyon testleri)

Kök nedeni bulmadan önce, kullanıcının ve kendi hipotezlerimin HER BİRİ
ayrı ayrı, izole şekilde (tek değişken, geri kalan her şey sabit) test
edildi -- hiçbiri repo dosyalarına yazılmadan, `/tmp` altında runtime
monkeypatch ile:

- **İki-taraflı kütle-ağırlıklı `clamp_direction()`** (kullanıcının somut
  önerisi -- standart PBD iki-cisim vektör-eşitlik kısıtlaması matematiği
  ile doğru şekilde uygulandı): kaçışı ~t17s'den ~t30s+'a GECİKTİRDİ ama
  ORTADAN KALDIRMADI -- kısmi doğrulama, tam çözüm değil.
- **Sönümleme/restitüsyon gücü** (momentum korumasının %100/%85/%50'si):
  kaçış zamanlamasında/büyüklüğünde ANLAMLI FARK YOK -- `clamp_direction()`
  'ın hız-işleme davranışının BASKIN enerji kaynağı olmadığını gösterdi.
- **Driver-kalça kayışının (leash) alt-gevşetilmesi (SOR)**: işleri
  KATEGORİK OLARAK KÖTÜLEŞTİRDİ (anında, daha temiz bir doğrusal kaçış) --
  kayışı SEBEP olmaktan çıkarıp aslında bir STABİLİZE EDİCİ mekanizma
  olduğunu ortaya çıkardı.
- **Global hız sınırı (12px/kare güvenlik ağı)**: kaçışı DURDURMADI --
  neredeyse birebir aynı ~-87px/s doğrusal kaçma devam etti. Bu en
  bilgilendirici NEGATİF sonuçtu: kaçışın kaotik/rastgele enerji patlaması
  DEĞİL, sabit, HEP AYNI YÖNLÜ ("vektörel yanlılık") bir sızıntı olduğunu
  kanıtladı.

### ΔY toplayıcı tanısı: kaynağın izolasyonu

Kullanıcının önerdiği kesin enstrümantasyon kuruldu: her kısıtlama
fonksiyonu (yay/açı/zemin/mesafe) çağrılmadan ÖNCEKİ ve SONRAKİ Y
pozisyonu arasındaki farkı ayrı bir toplayıcıda biriktirip, 60 saniyelik
bir koşuda 5 saniyelik pencereler halinde raporladı (yine `/tmp` altında
runtime monkeypatch, repo'ya yazılmadan). Sonuçlar:

- **`zemin` (collide_ground): kararlı-durum bölgesinde (t>17s) TAM OLARAK
  SIFIR katkı.** Sebep basit ama önemli: karakter o noktada zeminden
  TAMAMEN kopmuş, havada yükseliyor -- `collide_ground()` sadece zeminin
  ALTINA sızan noktaları düzeltir, havadaki bir noktayı asla geri çekmez.
  **Kullanıcının "Asimetrik Zemin Çarpışması" hipotezi kararlı-durum
  sızıntısı için ELENDİ** (ilk ~5 saniyede, hâlâ temas varken marjinal
  bir katkısı olmuş olabilir, ama bu SÜRDÜRÜLEN kaçışın sebebi değil).
- **`açı` ve `mesafe` ikisi de kalıcı, aynı-işaretli (negatif=yukarı) bir
  değeri, kare-kare AYNI şekilde tekrarlıyor** -- t=20s'den t=60s'ye kadar
  HER 5 saniyelik pencerede ondalık basamağına kadar BİREBİR AYNI toplam.
  Sistem gerçek anlamda periyodik bir limit-cycle'a girmiş: her kare
  aynı şeyi tekrarlıyor ve o tekrar sıfıra değil, küçük bir yukarı
  kalıntıya yakınsıyor -- **kullanıcının "Constraint Iteration Order"
  hipotezini güçlü şekilde destekleyen bir kanıt.**
- **`açı` kategorisi eklem bazında kırıldığında**, sızıntının **%75'inden
  fazlası TEK bir çağrıdan** geliyor: `clamp_direction(hip, shoulder, UP,
  max_lean)` -- gövde-eğim (torso-lean) kısıtlaması. Kararlı-durumda tek
  başına **-124px/s** (diğer 7 açı-kısıtlaması çağrısının TOPLAMI sadece
  ~-35px/s). Neden: gövde açısı t≈13s'de zaten `PASSIVE_TORSO_MAX_DEG`
  (75°) duvarına çarpıyor ve o andan itibaren bu clamp HER KAREDE
  tetikleniyor -- ve bu fonksiyon, 3. tur'da BİLİNÇLİ olarak dokunulmadan
  bırakılan o "hız sıfırlayan" sert clamp (`clamp_joint_angle_points()`
  'in round-4-ÖNCESİ "V-splits" kusuruyla AYNI mekanizma, farklı eklemde).
- **`mesafe` kategorisi** ise tek bir noktaya değil, `hip`/`l_knee`/
  `r_knee`/`l_foot`/`r_foot`/`l_elbow`'a neredeyse birebir AYNI (~-28px/s)
  dağılmış durumda -- muhtemelen `hip`'ten çubuk ağı üzerinden tüm
  iskelete yayılan tek bir ortak mekanizma (driver-kalça kayışı şüpheli
  bir aday; önceki bölümde onu gevşetmenin işleri KÖTÜLEŞTİRMESİ bununla
  tutarlı, ama bu tur bu payı DAHA FAZLA izole etmedi -- gelecek iş).

### Nedensel doğrulama ve uygulanan düzeltme

Bulunan tek-baskın-kaynak hipotezi NEDENSEL olarak doğrulamak için:
SADECE torso-lean `clamp_direction()` çağrısı momentum-koruyan hale
getirildi (`clamp_joint_angle_points()` ile BİREBİR AYNI ofset-tabanlı
yöntem -- `correction = yeni_konum - eski_konum`; bu hem `points`'e hem
`prev_points`'e AYNI MİKTARDA eklenir), geri kalan 5 `clamp_direction`
çağrısına (boyun, 2x kol konisi) HİÇ dokunulmadan:

| | ham (düzeltilmemiş) | sadece torso-lean düzeltildi |
|---|---|---|
| t=20s→60s kalça-Y hızı | ~-87 px/s (doğrusal kaçış) | ~-9.7 px/s (9x azalma) |
| davranış deseni | tekdüze, sınırsız kaçış | SINIRLI salınım (±150px bandı) |

**Bu, kullanıcının "tüm sistemi kütle-ağırlıklı yap" önerisinden FARKLI
ama daha kesin bir sonuç:** sorun kütle-ağırlıklandırma eksikliği değil
(zaten mesafe kısıtlaması kütle-ağırlıklı; kütle-ağırlıklı `clamp_
direction` de ayrıca denendi, kaçışı sadece 2x geciktirdi). Sorun,
SPESİFİK OLARAK bir tek aşırı-sık-tetiklenen sert clamp'ın hız
sıfırlamasıydı. TÜM `clamp_direction` çağrılarını AYNI ANDA
momentum-koruyan yapmak (bu araştırma sırasında ayrıca denendi) "çift
sarkaç" kaosu riskini geri getirip KARARSIZ kaldı -- bu yüzden değişiklik
SADECE en baskın sızıntı kaynağına, cerrahi şekilde uygulandı (bkz.
`physics/verlet.py`'nin `clamp_direction()` fonksiyonuna eklenen
`preserve_momentum: bool = False` parametresi -- varsayılan `False`,
yani BAŞKA HİÇBİR çağrı yeri etkilenmedi; sadece `demo/step9_ragdoll_
blend.py` ve `demo/step13_full_integration_test.py`'deki torso-lean
çağrısı `preserve_momentum=True` ile güncellendi).

**Regresyon doğrulaması:** `step7` kanarya değerleri (0.794/0.985/0.941)
ve `step12` kanarya değerleri (8.65px/38.80px/1.0000) BİREBİR AYNI kaldı.
`step13`'ün 24/30/60 FPS stres testi hiçbir konfigürasyonda NaN/patlama
üretmedi (önceden dokümante edilmiş "zaman-tutarsızlığı" sınırlaması --
FPS'e göre farklı `hip_x` -- bu turdan BAĞIMSIZ, DEĞİŞMEDİ).

**DÜRÜST YENİ BULGU (bu düzeltmenin görsel bir yan etkisi):** momentum
korunduğu için gövde artık -75°'lik duvara çarpıp orada DONMUYOR -- bunun
yerine "sekip" ters yöne salınıyor (bkz. `step9`'un varsayılan 13
saniyelik demosunda gövde açısı: t=10.2s'de -75° duvarına çarpıyor,
t=11.6s'de +6.8°'ye kadar SEKİYOR, t=12.9s'de -7.1°'de, yani neredeyse
DİK duruyor -- round-4'ün committed videosunda aynı an -75°'de donmuş
haldeydi). Görsel olarak üç örnek kare (`t=4.0s`, `t=10.2s`, `t=11.6s`)
incelendi -- karakterin uzuvları birbirinin içinden geçmiyor, poz
fiziksel olarak makul (bir "sersemlemiş, sallanan" ragdoll gibi), ama
60 saniyelik koşuda bu salınım GÖZLE GÖRÜLÜR ŞEKİLDE SÖNMÜYOR (±150px
bandında sabit genlikte devam ediyor) -- yani karakter artık ekrandan
uçup gitmiyor ama aynı zamanda "yere yığılıp durmuyor" da, sürekli
sallanıyor. Bu, `PASSIVE_TORSO_DAMPING`/`RAGDOLL_FRICTION` gibi mevcut
sönümleme parametrelerinin bu YENİ salınım moduna karşı yeterince
ayarlanmadığını gösteriyor -- düşük riskli bir ayar (sönümleme artışı)
ile muhtemelen giderilebilir, ama bu turun kapsamına BİLİNÇLİ OLARAK
DAHİL EDİLMEDİ (kullanıcı onayı "şimdilik (a) diyelim" ile mevcut hâliyle
kabul edildi).

**Kalan açık iş (gelecek tur için not edildi):** ~-9.7px/s'lik kalıntı
kaçış tam olarak sıfırlanmadı -- muhtemel kaynaklar: `r_elbow_hand` kol
konisi (~-34px/s payı, henüz nedensel doğrulanmadı), `mesafe` ağına
dağılmış ~-28px/s'lik pay (driver-kalça kayışı şüpheli), boyun (~-3.5px/s).
Ayrıca yukarıdaki salınım-sönmüyor bulgusu da ayrı bir sönümleme ayarı
gerektirebilir. Kullanıcının kararıyla bu turda daha FAZLA kovalanmadı --
kapsül öz-çarpışmasına geçiliyor.



## 6. tur: Kapsül Tabanlı Öz-Çarpışma (Segment-to-Segment Collision)

5. turun torso-lean düzeltmesi kabul edildikten ("şimdilik (a) diyelim")
hemen sonra kullanıcı üç büyük özelliği sırayla istedi: (1) kapsül tabanlı
öz-çarpışma, (2) aktif denge/refleks (CoM support-polygon kontrolü), (3)
içsel kas kuvvetiyle ayağa kalkma (active ragdoll). Bu bölüm SADECE
ilkini (2./3./4. turda defalarca ertelenen en kritik eksik) kapsıyor --
kullanıcının kendi tanımıyla: "iki çizginin (segment) birbirine en yakın
noktasını hesaplayıp, kolların, bacakların veya pelerinin gövdenin içinden
geçmesini fiziksel olarak imkansız hale getiren matematiksel kısıtlama."

### Mevcut yöntemin sınırı (neden yetersizdi)

`physics/self_collision.py`'deki `push_points_off_segment()` (2. tur eki)
sadece NOKTA-vs-SEGMENT itmesi yapıyordu: bir listedeki noktaları (ör.
dirsek, el) sabit bir segmentin (ör. kalça->omuz) en yakın noktasından
iter. **Kritik boşluk:** sadece `point_indices`'teki UÇ noktalar test
edilir -- bir ön-kolun (dirsek->el) TAM ORTASI gövde çizgisini kesse bile,
dirsek ve el kendileri segmentten yeterince uzaktaysa hiçbir çarpışma
algılanmıyordu (birim testiyle doğrulandı, aşağıya bakın).

### Yeni fonksiyon: `push_segment_off_segment()`

`physics/self_collision.py`'ye iki yeni fonksiyon eklendi:

- `_closest_points_segment_segment()`: iki 2D doğru parçası arasındaki
  GERÇEK en yakın nokta çiftini hesaplayan standart algoritma (Ericson,
  "Real-Time Collision Detection", §5.1.9 -- paralel/dejenere segmentleri
  de doğru ele alan, sayısal olarak kararlı versiyon).
- `push_segment_off_segment()`: bu en yakın nokta çiftini kullanıp, mesafe
  `min_dist`'in altındaysa iki segmenti (uçların `s`/`t` enterpolasyon
  oranlarıyla dağıtılan bir düzeltmeyle) birbirinden iter.
  - **Kütle-ağırlıklı** (`_satisfy_sticks()`'teki AYNI ters-kütle
    formülü, ama segment başına TEK birleşik kütle -- iki ucun ortalaması
    -- kullanılıyor, nokta başına tam ters-kütle DEĞİL; dürüst bir
    basitleştirme, dokstring'de belirtildi).
  - **Pinned-farkında**: bir segmentin iki ucu da pinned'se o segment
    tamamen sabit kabul edilir.
  - **Momentum-koruyan** (5. turun ΔY-toplayıcı dersinden çıkan BİLİNÇLİ
    tasarım kararı): düzeltme hem `points`'e hem `prev_points`'e AYNI
    miktarda ekleniyor -- `push_points_off_segment()`'in ışınlama+hız-
    sıfırlama davranışından farklı olarak, bu YENİ fonksiyon baştan hız
    sıfırlamayan bir kısıtlama olarak tasarlandı (5. turda tam da bu tür
    bir davranışın "vektörel yanlılık" kaynağı olabildiği görüldüğü için).

### Birim testi: nokta-bazlı yöntemin kaçırdığı, yeni yöntemin yakaladığı durum

Beş izole birim testi yazıldı (`/tmp/test_seg_seg.py`, repo'ya dahil
değil): paralel segmentler arası mesafe, dik kesişen segmentler (X
şeklinde, mesafe=0), pinned uçlarla asimetrik itme, eşit kütleli iki
serbest segmentin simetrik itilmesi, ve momentum korumasının doğrulanması
(`points`/`prev_points` AYNI miktar kayıyor mu). **En kritik test:** bir
"ön-kol" segmentinin (uçları gövdeden UZAK, ör. dirsek x=-10, el x=+10)
TAM ORTASI (x=0) bir "gövde" segmentini (dikey eksen) kesecek şekilde
kuruldu -- `push_points_off_segment()` bu durumu KESİNLİKLE
YAKALAYAMAZDI (uçlar segmentten uzak), `push_segment_off_segment()`
doğru şekilde çarpışmayı algılayıp düzeltti. Tüm 5 test geçti.

### Entegrasyon ve doğrulama

`demo/step9_ragdoll_blend.py` ve `demo/step13_full_integration_test.py`'
deki kol-vs-gövde/boyun çarpışma çağrıları (`push_points_off_segment` ile
[dirsek,el] noktalarını kontrol eden) `push_segment_off_segment`'e
yükseltildi (ön-kol SEGMENTİ artık gövde/boyun SEGMENTİNE karşı test
ediliyor). `step13`'teki pelerin-vs-gövde çarpışması da aynı şekilde
yükseltildi -- pelerin zincirindeki HER ARDIŞIK nokta çifti bir "pelerin
segmenti" olarak gövde/boyun segmentine karşı test ediliyor (önceden
sadece pelerin NOKTALARI tek tek test ediliyordu).

**Kapsam kararı:** gövde/boyun uçları (`hip`/`shoulder`/`head`) bu
çağrılar için BİLİNÇLİ olarak "pinned" muamelesi görüyor -- yani SADECE
kol/pelerin hareket ediyor, gövde bu çarpışmadan etkilenmiyor. Bu,
`push_points_off_segment()`'in ESKİ kapsamıyla BİREBİR AYNI (sadece
tespit kalitesi yükseltildi, düzeltmenin kime uygulandığı DEĞİŞMEDİ) --
gövdeyi de hareket ettirmek (iki taraflı çarpışma tepkisi) ayrı, daha
riskli bir sonraki adım olarak bilinçli şekilde ERTELENDİ (torso-lean
düzeltmesiyle etkileşimi ayrı doğrulama gerektirir).

**Yakınsama (relaksasyon) bulgusu:** tek geçişlik bir çağrı, aynı
taraftaki kol-vs-gövde VE kol-vs-boyun kısıtlamalarının birbirini
EZMESİNE yol açtı (biri düzeltirken diğerini bozuyordu) -- ölçüldü: tek
geçişte en yakın mesafe hedef 11px'in belirgin altında (~4-9px) kalıyordu.
`_satisfy_sticks()`'teki `RELAX_ITERS` mantığıyla AYNI ruhta, kol/gövde ve
kol/boyun çiftleri için 4 iterasyonluk küçük bir relaksasyon döngüsü
eklendi (`SELF_COLLISION_RELAX_ITERS=4`) -- kare-sonu doğru ölçümle
(self-collision TAMAMLANDIKTAN, diz-clamp'ten HEMEN ÖNCE) doğrulandı:
l_torso 10.86px, r_torso 10.79px, l_neck 11.00px, r_neck 21.11px (hedef
≥11px) -- yani pratik olarak yakınsıyor (kalan ~0.14-0.21px eksik,
`_satisfy_sticks()`'in kendi sınırlı-iterasyon yakınsamasıyla AYNI
mahiyette bir dürüst sınır, sonsuz iterasyon yok).

**Regresyon:** `step1`-`step12` tüm kanarya değerleri (step7:
0.794/0.985/0.941; step12: 8.65px/38.80px/1.0000) BİREBİR AYNI kaldı.
`step13`'ün 24/30/60 FPS testi hiçbir konfigürasyonda NaN/patlama
üretmedi, son `hip_x` değerleri (312.98/304.06/-14.02) bu turdan ÖNCEKİYLE
BİREBİR AYNI (beklenen -- kol/pelerin çarpışması `hip`'i hiç hareket
ettirmiyor, sadece kol/pelerin noktalarını etkiliyor). Görsel QA: en yakın
yaklaşım anı (t=3.03s, mesafe=10.79px) çıkarılıp incelendi -- kol doğal
şekilde gövdenin yanında duruyor, görünür bir "sıkışma"/tuhaflık yok.

**Dürüst sınırlar (bilinçli olarak bu tura DAHİL EDİLMEDİ):**
- Bacak-bacak veya bacak-gövde çarpışması hâlâ YOK (kullanıcının isteği
  kol/bacak/pelerin üçünü de kapsıyordu; bu tur sadece kol+pelerin'i ele
  aldı -- mevcut gait/ragdoll'da bacakların birbirine girdiği
  GÖZLEMLENMEMİŞ bir durum, ama ragdoll flailing sırasında teorik olarak
  mümkün; aynı `push_segment_off_segment()` fonksiyonu kullanılabilir,
  ayrı bir doğrulama turu gerektirir).
- Gövde/boyun uçları bu çarpışmalardan HİÇ etkilenmiyor (tek taraflı tepki)
  -- gerçek bir "kolun gövdeye çarpması gövdeyi de hafifçe iter" fiziği
  yok.
- Segment başına TEK birleşik kütle (iki ucun ortalaması), nokta başına
  tam ters-kütle değil.
- Tek karede sonlu (4) iterasyon -- matematiksel olarak KESİN sıfır-
  ihlal garantisi yok, sadece pratik/ölçülmüş yakınsama.



## 7. tur: Aktif Denge ve Refleks (Center of Mass Recovery)

Kapsül öz-çarpışmasının ("6. tur") kabulünden hemen sonra kullanıcının
sıraladığı üç büyük özellikten ikincisi. Kullanıcının kendi tarifi:
"Karakter iteratif olarak yürüyor ama dengesini kaybettiğinde (örneğin
dışarıdan bir kuvvet uygulandığında) bunu 'fark etmiyor'. Kütle merkezinin
(kalça/gövde) ayakların yere bastığı izdüşümün (support polygon) dışına
çıkıp çıkmadığını her karede ölçmeliyiz. Karakter düşeceğini anladığında,
dengesini sağlamak için kollarını ters yöne savurarak (counter-balance)
veya fazladan bir adım atarak refleks göstermeli." DURUM: **TAMAMLANDI,
commit edildi** -- ama aşağıdaki "dürüst bulgular" bölümünde açıklanan,
bilinçli olarak bu tura dahil edilmeyen bir sınır var.

**Önce sayısal doğrulama (mevcut sistemin gerçekten "fark etmediği" mi?):**
`physics/balance.py`'deki mevcut `counter_balance_offset()` sürekli çalışan,
küçük genlikli bir orantılı geri besleme -- ne bir "tehlike" eşiği var, ne
de gerçek bir destek ARALIĞI (eski `support_x()` tek bir NOKTA döndürüyor,
ayağın fiziksel genişliğini saymıyor). Bir tanı script'i `demo/
step12_balance.py`'nin sahnesine 55/150/300px'lik üç farklı büyüklükte
"tokezleme" uygulayıp ölçtü:
- **55px** (orijinal demo değeri): gerçek destek-aralığı-dışı mesafe sadece
  ~27px -- kol tepkisi (`BAL_MAX_ERR=80px`) DOYMUYOR, yani zaten yeterli.
- **150px**: gerçek dışı-mesafe ~103px, kol tepkisi **-44px'te DOYUYOR**.
- **300px**: gerçek dışı-mesafe ~240px (150px'in neredeyse 2.5 katı
  gerçek tehlike), ama kol tepkisi YİNE **-44px'te DOYUYOR** -- yani
  sistem 103px'lik bir tehlike ile 240px'lik bir tehlike arasında HİÇBİR
  AYRIM YAPMIYOR.
- Adım zamanlaması: her üç itki büyüklüğünde de İLK adım tam olarak AYNI
  karede tetikleniyor -- ama bu, sistemin dengeyi "fark etmesinden" değil,
  bacağın kendi kinematik `stride_release=18px` eşiğinin (kalçanın ayaktan
  ne kadar uzaklaştığına bakan, dengeden tamamen habersiz bir kural) HER
  itki büyüklüğünde zaten aşılmasından kaynaklanıyor -- **tesadüfi bir
  yan etki**, kasıtlı bir refleks değil.

Kullanıcının iddiası böylece sayısal olarak doğrulandı: sistem gerçekten
"fark etmiyor" -- ne destek poligonuna bakıyor, ne tehlike büyüklüğüne
göre ölçekleniyor, ne de ekstra bir adım atıyor.

**Yeni katman** (`physics/balance.py`, mevcut `counter_balance_offset()`
DEĞİŞTİRİLMEDEN, üzerine eklendi):
- `support_interval(stance_foot_x, foot_half_len, fallback_x)`: destek
  tabanını artık tek bir nokta değil, o an zeminde duran ayağın/ayakların
  fiziksel genişliğini (`FOOT_HALF_LEN=12px`, tek bir sabit yaklaşıklık)
  de sayan bir ARALIK olarak modelliyor.
- `outside_interval_error(com_x, interval)`: kütle merkezinin bu aralığın
  GERÇEKTEN dışına çıkma mesafesi (0 = güvenli, normal yürüyüş salınımı
  aralık içinde kaldığı sürece "tehlike" sayılmıyor).
- `FallRiskMonitor`: histerezisli (giriş eşiği ≠ çıkış eşiği) bir durum
  makinesi -- tek bir gürültülü karenin tehlikeyi tetikleyip hemen
  kapatmasını önlüyor, `FootPlantingLeg`'in kendi durum makinesiyle aynı
  tasarım ilkesi.
- `emergency_counter_balance_offset()`: `counter_balance_offset` ile aynı
  yön/şekil mantığı, ama çok daha büyük `gain_x`/`max_err` ile -- gerçek
  tehlikede kolları "ters yöne savurma" büyüklüğünde bir tepki.
- `physics/gait.py`'ye `FootPlantingLeg.trigger_emergency_step(target_x,
  speedup)`: bacak o an STANCE durumundaysa, normal `stride_release`
  beklemesini atlayıp hemen SWING'e geçirip yeni hedefe (çağıran kodun
  kütle merkezinin düştüğü yöne göre hesapladığı bir "yakalama noktası")
  yönlendiriyor, `speedup>1` ile swing süresini kısaltıyor (daha hızlı/
  aceleci bir adım). Bacak zaten SWING'teyse (havadaki bir adımın
  ortasındaysa) hiçbir şey yapmıyor ve `False` dönüyor -- fiziksel olarak
  yarım kalmış bir adımı kesintiye uğratmak bu turun kapsamına alınmadı.

**Entegrasyon** (`demo/step12_balance.py` + `demo/step13_full_integration_
test.py`): orijinal t=3.0s/55px "tokezleme" BİLİNÇLİ OLARAK dokunulmadan
bırakıldı (`FALL_RISK_ENTER_PX=45px` bu tokezlemenin gerçek dışı-mesafesinin
[~27px] ÜZERİNDE seçildi) -- eski kanarya sayıları (tokezleme öncesi/sonrası
`|error|`, kol-ofseti korelasyonu) BİREBİR AYNI kalıyor: `8.65px` / `38.80px`
/ `1.0000`. t=5.5s'de (step13'te STUMBLE_T=4.0 ile KNOCKDOWN_T=7.0 arasında,
blend hâlâ tam aktifken) çok daha büyük (220px) ikinci bir kalça itkisi
eklendi; bu itkide `FallRiskMonitor` tehlikeye giriyor, kollar
`emergency_counter_balance_offset` ile savruluyor, ve o an zemindeki bacak
`trigger_emergency_step` ile erken/hızlı bir yakalama adımı atıyor.

**Doğrulama:**
- Emergency tepkinin gerçek tehlikeyle ÖLÇEKLENDİĞİ sayısal olarak
  doğrulandı -- eski formül 103px/190px/240px'lik üç farklı gerçek
  tehlikede de aynı **-44px**'te doyarken (ayırt edemiyor), yeni
  `emergency_counter_balance_offset` aynı üç değerde **-92.7px / -170.8px
  / -198.0px** üretiyor -- gerçek büyüklükle orantılı, ~4.5 kat daha büyük
  bir tepki aralığı.
- 220px'lik itki sonrası: tehlikeye giriş frame 166 (t=5.53s), kurtulma
  frame 174 (t=5.80s) -- **8 karede** (0.27s) toparlanma, acil adım(lar)
  doğru bacak(lar)da tetiklendi (`[(172,'r'),(173,'l')]`).
- Görsel QA (3 kare: t=5.53s/tehlike girişi, t=5.73s/en derin düşüş +
  yakalama adımı bacağı uzatılmış, t=5.80s/toparlanmış normal duruş) --
  karakter gerçekten "yakalıyor" gibi görünüyor, uzuvlar birbirinin içinden
  geçmiyor.
- 60 saniyelik uzatılmış kararlılık testi (round-5'in dersi -- kısa
  doğrulama YETERSİZ olabilir kuralı bu tura da uygulandı): 8 saniyede bir
  tekrarlanan aynı (220px) itki 60s boyunca NaN/patlama üretmedi, dikey
  (Y) sapma hep <6px kaldı (yatayda biriken ~5100px tamamen beklenen --
  60s boyunca kesintisiz yürüme + 7 itkinin toplam mesafesi, bir patlama
  DEĞİL). `FallRiskMonitor` her tetiklendiğinde TEMİZ giriş/çıkış yaptı,
  çırpınma (aynı olaydan birden fazla tetiklenme) YOK.
- Tam regresyon: step1-step12 (step7: 0.794/0.985/0.941; step12: kendi
  3 kanarya sayısı BİREBİR AYNI) + step13'ün 24/30/60 FPS testinde
  NaN yok. step13'ün son `hip_x` değerleri DEĞİŞTİ (304→478 @ 30 FPS) --
  bu bir regresyon DEĞİL, bilinçli olarak eklenen yeni 220px'lik itki
  olayȅneşenin kalcayı ekstra ileri sürüklemesinin doğrudan/beklenen sonucu.

**Dürüst bulgular (bilinçli olarak bu tura DAHİL EDİLMEDİ veya çözülemedi):**
- **En önemli mimari sınır -- "ekstra adım" reflex'i step12/13'ün
  KİNEMATİK kalça mimarisinde ayrı bir kazanım olarak ÖLÇÜLEMEDİ:** bu
  sahnelerde kalça (`driver`) her zaman doğrudan pinned bir noktaya kısa
  bir çubukla bağlı (fiziksel eylemsizliği yok) ve bacağın kendi
  `stride_release=18px` eşiği, gerçek tehlike için gereken herhangi bir
  itki büyüklüğünden (>45px) ÇOK daha küçük. Bu yüzden support-polygon
  tabanlı tehlikeyi yaratacak KADAR büyük HERHANGİ bir kalça itkisi, aynı
  anda (aynı karede) var olan kinematik `stride_release` eşiğini de
  otomatik olarak tetikliyor -- eski VE yeni sistem, 220px'lik itkiden
  sonra AYNI karede (166→174, 8 kare) "toparlanıyor" çünkü ikisi de aynı
  tesadüfi mekanizmadan faydalanıyor. Bunu ayırt etmek için govdeye/başa
  SADECE bir hız-darbesi (kalçaya dokunmadan) da denendi, ama bu sahnede
  gövde/boyun HER KAREDE sert bir `TORSO_MAX_LEAN_DEG=12°` kenetlemesiyle
  kalçaya bağlı olduğundan, darbe büyüklüğü ne olursa olsun (80-320px/kare
  arası denendi) gerçek dışı-mesafe hep ~25-28px'de SABİT kaldı --
  tehlike eşiğinin (45px) altında, yani bu sahnede YETERLİ bir "gerçek
  tehlike" hiç yaratılamadı. Sonuç: `trigger_emergency_step()` mekanizması
  doğru çalışıyor (kendi izole birim testinde doğrulandı -- bkz. "acil
  adım" tetiklenmesi/hızlanması), commit edildi ve zarar vermiyor (normal
  yürüyüşte hiç tetiklenmiyor), ama BU İKİ DEMONUN kinematik-kalça
  mimarisinde "eski sisteme göre daha hızlı toparlanma" olarak AYRI/net
  şekilde GÖSTERİLEMEDİ -- gerçek kazanç muhtemelen 3. maddedeki (İçsel
  Kas Kuvveti / Active Ragdoll) kalçaya GERÇEK eylemsizlik kazandıran
  çalışmadan SONRA ortaya çıkacak (kalça artık pinned bir noktaya değil,
  gerçek fiziğe tepki verirse, "büyük ama stride_release'i tetiklemeyen"
  bir itki senaryosu mümkün olur).
- **Aynı büyüklükteki tekrarlanan itkiler HER ZAMAN tehlikeyi
  tetiklemiyor:** 60s testinde 7 tekrardan sadece 4'ü `FallRiskMonitor`'ü
  tetikledi (diğer 3'ünde gerçek dışı-mesafe ~38px'de kaldı, 45px eşiğinin
  altında) -- itkinin YÜRÜYÜŞ FAZININ (o an hangi ayağın stance/swing
  olduğu, kütle merkezinin faz içindeki konumu) hangi anına denk geldiğine
  bağlı. Bu bir çırpınma/kararsızlık DEĞİL (her tetiklenme temiz giriş/
  çıkış yapıyor) ama beklenmedik bir hassasiyet -- aynı darbe her zaman
  aynı tepkiyi ÜRETMEYEBİLİR, gait fazına göre değişir.
- `outside_interval_error` yalnızca ANLIK konuma bakıyor -- gerçek
  biyomekanikteki "extrapolated center of mass" (hız/momentum
  projeksiyonu ile ÖNCEDEN tahmin) gibi bir öngörü YOK, yani hızlı bir
  düşüşte geç kalabilir.
- `support_interval` ayak genişliğini tek bir sabitle (`FOOT_HALF_LEN`)
  yaklaşıklıyor, gerçek ayak geometrisi/basma noktası yok.
- Bu hâlâ gerçek bir ters-dinamik/rigid-body denge kontrolcüsü DEĞİL --
  eşik tabanlı bir refleks anahtarlama katmanı (`counter_balance_offset`
  gibi, sadece daha büyük/koşullu).
- Kalan iki büyük özellik (Aktif Denge tamamlandı; sırada: 3. madde --
  İçsel Kas Kuvveti ve Ayağa Kalkma) henüz başlanmadı.

Commit: (bu turun commit'i README ile birlikte yapılacak).


## 7. tur eki: örtüşen tetikleyicilerin ayrıştırılması (`hold_release`)

7. turun commit'inden hemen sonra kullanıcı, "ekstra adım" refleksinin bu
demolarda ayrı ölçülebilir bir fayda göstermediği bulgusuna somut bir
mimari eleştiriyle geri döndü: `gait.py`'nin normal adım-atma kararı
(`stride_release` -- sadece kalça-kayma mesafesine bakan kinematik bir
eşik) ile `balance.py`'nin acil-durum kararı (`trigger_emergency_step`)
birbirinden TAMAMEN HABERSİZ çalışıyor; ikisi de aynı bacağı, aynı
kalça-kayması sinyaline göre bağımsız olarak hareket ettirmeye
çalışabiliyor ("örtüşen tetikleyiciler"). Kullanıcının önerisi: kütle
merkezi destek poligonunun DIŞINDAYKEN normal yürüyüş döngüsü GEÇİCİ
OLARAK durdurulmalı (override) ve kontrol TAMAMEN `trigger_emergency_
step()`'e bırakılmalı.

**Diagnostik-A (fix'ten ÖNCE, iddiayı doğrula/netleştir):** `FootPlanting
Leg.update()` monkeypatch'lenip her stance→swing geçişi (NORMAL kinematik
mi, EMERGENCY mi -- `emergency_step_active` bayrağına bakarak) loglandı.
step12'nin TEK 220px'lik itki senaryosunda bu geçiş TAM OLARAK
kullanıcının tarif ettiği şekilde (aynı karede, aynı anda) gerçekleşmiyordu
-- sayısal olarak SIFIR "ihlal" (`in_danger=True` İKEN gerçekleşen NORMAL
geçiş) bulundu. Ama gerçek mekanizma daha ince bir zamanlama sorunuydu:
push'tan HEMEN önce (frame 160/161, `in_danger` henüz False iken), sıradan
yürüyüş ritmi zaten HER İKİ bacağı da normal `stride_release` ile
swing'e sokmuştu (push'la tamamen tesadüfi bir çakışma) -- push'un
kendisi (frame 165) `in_danger`'ı frame 166'da tetiklediğinde HER İKİ
bacak da zaten havadaydı, dolayısıyla `trigger_emergency_step()` ilk
birkaç karede hiçbir bacağı "ele geçiremedi" (ikisi de "stance" değildi).
Bacaklar inip STANCE'a döndüğü anda (frame ~170-171) acil sistem onları
HEMEN yakaladı (frame 172/173) -- çünkü "acil dene" kontrolü HER karede
`update()`'ten ÖNCE çalışıyor, yani bir bacak müsait olur olmaz acil
sistem yarışı zaten kazanıyor. **Bu, kullanıcının tarif ettiği mekanizmanın
BİREBİR AYNISI değil ama AYNI kökten (iki sistemin habersizliği) kaynaklanan
kardeş bir bulgu: örtüşme "aynı anda çift tetikleme" şeklinde değil, "önceden
başlamış, alakasız bir normal salınımın acil sistemin ilk tepki penceresini
işgal etmesi" şeklinde gerçekleşiyor.**

Bu yüzden TEK itkilik step12 senaryosu, iddiayı sınamak için YETERSİZ bir
test ortamı çıktı -- daha önce round-5'te öğrenilen derse ("kısa süreli
doğrulama yetersiz olabilir") paralel yeni bir ders: **tek bir senaryo,
belirli bir mimari sorunu göstermeye YETMEYEBİLİR, farklı gait-fazlarında
birden çok deneme gerekir.** 60 saniyelik tekrarlı-itki testinde (7×220px,
adım D'nin sahnesi) durum FARKLI çıktı: 4 başarılı tehlike tetiklemesinin
**3'ünde** (`frame 408/'r'`, `888/'r'`, `1368/'r'`), `in_danger=True` İKEN
bir bacak KENDİ kinematik `stride_release` eşiğini bağımsız aşıp NORMAL
(acil parametrelerinden habersiz) bir adım atıyordu -- kullanıcının
tarif ettiği örtüşmenin somut, sayısal kanıtı.

**Düzeltme:** `FootPlantingLeg.update()`'e yeni bir `hold_release: bool =
False` parametresi eklendi (varsayılan `False`, TÜM mevcut çağıran kod
-- step1-step11, step12/13'ün tehlike-dışı kareleri -- ETKİLENMEDİ).
`hold_release=True` iken, bacak STANCE durumundaysa normal kinematik
`stride_release` kontrolü TAMAMEN atlanır (ayak yerinde kalır) -- bacağın
swing'e geçmesinin TEK yolu `trigger_emergency_step()` çağrısı olur.
`demo/step12_balance.py` ve `demo/step13_full_integration_test.py` her
karede `left_leg.update(hip_pos, hold_release=in_danger)` /
`right_leg.update(hip_pos, hold_release=in_danger)` çağırıyor (step13'te
`in_danger`, `blend <= 0.99` iken varsayılan `False` -- denge mantığı
zaten sadece tam aktif modda anlamlı).

**Doğrulama (aynı 60s senaryosu, fix sonrası, `git show HEAD:...` ile
okunan ESKİ koda karşı `importlib` tabanlı A/B -- `git stash` bu ortamda
[bağlı klasörde silme izni yok] kendi iç içe `index.lock` döngüsüyle
deterministik olarak kilitlendiği için KULLANILMADI, salt-okunur `git
show` tercih edildi):**
- İhlal sayısı: **3 → 0**. Fix sonrası 60s testinde `in_danger=True` iken
  gerçekleşen HİÇBİR normal geçiş yok; NORMAL geçiş sayısı ikisinde de
  aynı (158) ama EMERGENCY geçiş sayısı **4 → 7** çıktı (önceden sadece
  1 bacak "resmi" acil adım alıyordu, üç itkide diğer bacak kontrolsüz
  kendi başına gidiyordu; artık her iki bacak da acil sistem üzerinden
  hareket ediyor).
- **Dürüst maliyet (bedelsiz değil):** aynı 3 itkinin kurtulma süresi
  3 kareden 4 kareye çıktı (giriş/çıkış çiftleri: `frame 406→409` (3 kare)
  → `406→410` (4 kare), aynı şekilde 886/1366 için de). Yani örtüşmeyi
  kapatmak, o üç örnekte kontrolsüz-ama-tesadüfen-hızlı bir tepkiyi,
  kontrollü-ama-1-kare-daha-yavaş bir tepkiyle değiştirdi -- ~33ms'lik
  (60 FPS'te değil, bu sahne 30 FPS, yani 1 kare = 33ms) gözle
  ayırt edilmesi neredeyse imkansız bir fark, ama SIFIR değil, bu yüzden
  "bedelsiz düzeltme" diye sunulmuyor.
- İlk itki (frame 167, step12'nin de test ettiği push) hiç etkilenmedi
  (8 kare, değişmedi) -- zaten ihlal içermiyordu.
- Regresyon: step7 (0.794/0.985/0.941), step12 (8.65/38.80px, korelasyon
  1.0000, 7. tur bloğu: PIK +189.73px, giriş 166→kurtulma 174, 8 kare) ve
  step13'ün 24/30/60 FPS `hip_x` değerleri (478.38/562.89/364.54)
  **BİREBİR AYNI** kaldı -- bu iki senaryonun kendisi zaten ihlal
  içermediği için (yukarıdaki diagnostik-A bulgusu) beklenen bir sonuç.
- 60 saniyelik uzatılmış kararlılık testi tekrarlandı: NaN/patlama yok,
  max hip Y sapması 6.24px (önceki 5.36px ile aynı mertebede, benign),
  4 temiz giriş/çıkış çifti (çırpınma yok).
- Görsel QA: `t=13.6s` (2. itkinin tam ihlal anı) karşılaştırıldı --
  eski koddaki kare bacağın hâlâ havada (kontrolsüz normal swing ortası)
  olduğunu, yeni koddaki AYNI kare bacağın zaten yere değmiş/yakalanmış
  olduğunu gösteriyor -- sayısal bulguyla tutarlı, görünür bir fark var.

**Dürüst sonuç:** kullanıcının mimari eleştirisi doğruydu ve düzeltme
gerçek, ölçülebilir bir sorunu kapattı (60s testinde 3/4 gerçek tehlike
olayında) -- ama etkisi committed 7. tur'un kendi TEK-itkilik demo
sahnelerinde (step12/step13) GÖRÜNMÜYOR, çünkü o iki spesifik sahne
tesadüfen ihlal içermiyordu. Bu, önceki "ekstra adım refleksi bu demolarda
ayrı ölçülebilir fayda göstermiyor" bulgusunu YANLIŞLAMIYOR ama
NÜANSLIYOR: refleksin kendisi hâlâ bu iki sahnede öne çıkan bir fark
yaratmıyor, ama altındaki mimari (artık gerçekten ayrıştırılmış iki karar
mekanizması) daha genel senaryolarda (60s testi gibi) somut biçimde daha
sağlam.

Commit: (bu ekin commit'i README ile birlikte yapılacak).


## 8. tur kullanıcı geri bildirimi: "Kinematik-Pin Kalça" eleştirisi ve Active Ragdoll (`step14`)

7. tur ekinin commit'inden sonra kullanıcı, motorun en temel mimari
zaafına işaret eden bir eleştiriyle geri döndü: kalça (`hip`), uzayda
**kinematik bir pin gibi** davranıyordu. `step9`/`step12`/`step13`'ün
hepsinde kalçanın gerçek konumu, bir `driver` adlı **pinned** Verlet
noktası tarafından `WALK_SPEED` ile sahnelenmiş bir hızda X ekseninde
sürükleniyor, `hip` ise bu `driver`'a `add_stick(driver, hip, length=3.0)`
ile neredeyse-rijit (3px) bir çubukla bağlanıyordu. Kullanıcının
eleştirisi: eğer kalça, gelen kuvvetlerden bağımsız olarak neredeyse
sabit bir yörüngeye zorlanıyorsa, "destek poligonu dışına çıkma"
temelli düşme-riski sistemi anlamsız bir **görsel yanılsama**dan
ibarettir — karakterin kütle merkezi gerçek bir itki veya darbe
karşısında asla doğal biçimde kayamaz.

### Ön-tanı: iddia sayısal olarak doğrulandı

Uygulamaya geçmeden önce proje disiplini gereği iddia izole olarak
sayısal biçimde test edildi (`/tmp/diag_kinematic_hip.py`, commit
edilmedi): `hip`'e doğrudan 220px'lik bir itki (`prev_points` üzerinden)
uygulandığında, kalçanın `driver`'dan maksimum sapması **tam olarak
3.0000px** ile sınırlı kaldı — `FALL_RISK_ENTER_PX=45px` eşiğinin
**onda birinden bile azı**. Aynı 220px, mevcut yöntemle (`driver_x`'e
eklenerek) uygulandığında kalça beklendiği gibi ~220px hareket etti.
Bu, kullanıcının "3px'lik çubuk her şeyi yutuyor" iddiasının birebir
sayısal kanıtıydı.

### Kapsam kararı ve mimari yön

Kullanıcıya üç seçenek sunuldu (her yere yaymak / sadece yeni Active
Ragdoll çalışmasına sınırlamak / önce izole prototipte denemek);
kullanıcı **önce izole prototip** seçeneğini seçti. İzole tek-bacaklı
ters-sarkaç prototipi (`/tmp/proto_active_hip.py`, gerçek
`physics.verlet.VerletSystem` sınıfı kullanılarak, repo'ya hiçbir
dokunuş yapılmadan) dört testle hipotezi doğruladı: (A) itkisiz sarkaç
düşüyor (net -7.98px/30 kare) — yani "itki olmadan yürünemez" matematiksel
olarak kanıtlandı; (B) itkiyle net +70.66px ilerleme; (C) 20/60/150px'lik
yanal darbelere **orantılı, gerçek** tepkiler (19.4/29.0/14.9px) — eski
3px'lik sahte tavana kıyasla; (D) 300-1200px'lik aşırı darbelerde bile
NaN yok, bacak-uzunluğu kısıtı tam korunuyor.

Kullanıcı bu sonuçları onayladıktan sonra ("düşmek artık sadece bir
`bool` değişkeni değil") açık ve ayrıntılı bir mimari direktif verdi:

1. Değişiklik **sadece** yeni Active Ragdoll çalışmasına (`step14`)
   sınırlanmalı — `step1`-`step11`'in kinematik kanaryaları korunmalı.
2. **Polimorfizm** kullanılmalı: `gait.py`'ye dokunulmamalı, `if
   active_hip:` gibi dallanma eklenmemeli — bunun yerine `gait.py`'nin
   `FootPlantingLeg` sınıfından kalıtım alan yeni bir `ActiveFootPlanting
   Leg` sınıfı, yeni bir `active_gait.py` modülünde tanımlanmalı.
3. Yeni dinamik bacak sınıfının `stride_release`'i artık sabit bir piksel
   eşiği olamaz — **Düşme Süresi (Time-to-Fall)** veya **Kütle Merkezi
   İzdüşümü (Capture Point)** üzerinden hesaplanmalı: kalça ne kadar
   hızlıysa ayak o kadar erken ve o kadar ileriye atılmalı.
4. Sonuç: yeni bir `step14_active_biped.py` ile iki bacaklı, değişken
   hızlı, dinamik kalça transferi (push-off + swing) kodlanmalı.

### Uygulama: `physics/active_gait.py` ve `demo/step14_active_biped.py`

`gait.py` **hiç değiştirilmedi** (grep ile doğrulandı — dosya bu turda
sıfır satır değişiklik gördü). Yeni `ActiveFootPlantingLeg(FootPlanting
Leg)` sınıfı, `update()`'i override ederek klasik Linear Inverted
Pendulum / Capture Point modelini (Pratt ve ark., 2006) uyguluyor:

```python
xcp = hip_pos[0] + capture_gain * hip_vx / omega0
if xcp - self.planted[0] > support_margin:
    # adımı ONCEDEN ve daha ILERIYE at
    ...
```

`step14_active_biped.py`'de `driver` noktası **yok** — `hip`, `mass=1.0`
ile tamamen serbest bir Verlet noktası; ayrıca o an zeminde duran bacağın
`planted` konumuna her karede taşınan pinned bir `anchor` noktası var ve
ikisi arasında bacakların toplam uzunluğu kadar (184px) rijit bir çubuk
bulunuyor — kalçayı "o an destekte olan ayağın üzerinde dönen bir ters
sarkaç" gibi davranmaya zorluyor (robotikte **compass gait** olarak
bilinen, dizin görsel FABRIK çözümünü etkilemeyen ama alttaki fiziği
basitleştiren standart bir yaklaşım — bkz. aşağıdaki "dürüst sınırlar").

Push-off, hedef hız ile mevcut hız arasındaki farka orantılı bir
P-kontrolcü ile modellendi (`thrust = THRUST_GAIN * (TARGET_VX -
hip_vx)`, `±THRUST_CAP` ile sınırlı).

### Ayar sürecinde bulunan ve düzeltilen üç bağımsız kararsızlık

Nihai demo'ya geçmeden önce, izole bir ayar düzeneğinde (`/tmp/proto14/`,
commit edilmedi) sırayla üç farklı kararsızlık bulundu ve düzeltildi:

1. **Koşulsuz itki → sınırsız hızlanma.** İlk denemede itki sabit/
   koşulsuzdu; sonuç, hiçbir zaman durağanlaşmayan, sürekli hızlanan bir
   karakterdi (300 karede final hız 6-12px/kare'ye çıkıyor, hâlâ
   artıyordu). **Düzeltme:** hedef-hıza orantılı (P-kontrolcü) itkiye
   geçildi — sistem kendiliğinden kararlı, yakınsayan bir yürüyüşe geçti.
2. **Çift-havada (double-swing) kararsızlığı.** Bacakların bağımsız,
   birbirinden habersiz adım kararları bazen HER İKİSİNİ aynı anda
   havaya kaldırıyordu — kalça anlık olarak sıfır fiziksel destekle
   serbest düşüşe geçiyor, sonra inişte şiddetli bir düzeltme darbesi
   (bir konfigürasyonda hız 3195px/kare'ye fırlıyor) oluşuyordu.
   **Düzeltme:** `ActiveFootPlantingLeg.update()`'e `other_leg_swinging`
   parametresi eklendi; çağıran kod her karede diğer bacağın durumunu
   kontrol edip iletiyor, böylece tek-destek (single-support) kuralı
   kesin olarak korunuyor.
3. **Teorik `omega0` bu motorda kararsız.** LIP modelinin teorik
   `omega0 = sqrt(g/L) ≈ 0.0188` değeri demoda kararsızdı (ortalama
   hız 3.664'e taşıyor, anlık tepe 25.47px/kare) — izole ayar
   düzeneğindeki taramalarda 0.045 ile çalışıyor olmasına rağmen.
   **Kök neden:** bu motor Verlet entegrasyonunda HER ZAMAN `dt=1.0`
   kullanıyor (gerçek saniye hiç kullanılmıyor — bu, projenin önceki
   "Mimari refactor" bölümünde zaten belgelenmiş bir tasarım kısıtı);
   bu yüzden sürekli-zaman LIP formülünün doğrudan ikamesi bu motorda
   geçerli değil. **Düzeltme değil, dürüst bir uzlaşma:** ampirik olarak
   doğrulanmış `OMEGA0 = 0.045` sabit değeri kullanıldı, kod içinde bu
   teorik/ampirik uyuşmazlık açıkça yorumlandı.

### Dördüncü bulgu: `clamp_direction()`'ın varsayılanı, inşa edilen özelliğin TAMAMINI maskeliyordu

Gövde/kafa eklenip demo tamamlandığında, sonuç ilk bakışta "mükemmel"
görünüyordu: 400px'e kadar hiçbir itki karakteri düşürmüyordu. Proje
disiplini ("inanılmayacak kadar iyi bir sonuca ham haliyle güvenme")
gereği bu şüpheyle karşılandı ve üç-yönlü bir izolasyon testi yapıldı
(gövdesiz / gövde+varsayılan-`clamp_direction` / gövde+`clamp_direction`
YOK). Sonuç: gövdesiz haliyle 60px+ itkiler gerçekten düşürüyordu;
gövde eklenip **varsayılan** (`preserve_momentum=False`) `clamp_direction`
çağrıları kullanıldığında karakter 400px'e kadar TAMAMEN düşmeye bağışık
hale geliyordu; `clamp_direction` çıkarılınca eşik ~150px'e geri
dönüyordu. **Kök neden:** `clamp_direction`'ın varsayılan davranışı,
gövde/boyun açı-kelepçesini uygularken `prev_points`'i doğrudan üzerine
yazıyor — bu, kalçanın KENDİ gerçek hızını da gövde üzerinden dolaylı
olarak sessizce sıfırlıyordu (5. turda AYNI sınıf sorunun torso-lean
için zaten keşfedilip düzeltildiği desenin, burada farklı bir bağlamda
yeniden ortaya çıkması). **Düzeltme:** her iki `clamp_direction` çağrısına
da `preserve_momentum=True` verildi. Sonuç: düşme eşiği, gövdenin
GERÇEK ek eylemsizliğini yansıtan daha fizik-tutarlı bir aralığa
(~100px) döndü (100/150/220/300px düşüyor, 30/60px hayatta kalıyor).

### Beşinci bulgu: düzeltme kendi ardından yeni, daha ince bir kararsızlık açığa çıkardı

`preserve_momentum=True` düzeltmesi UYGULANDIKTAN SONRA, önceden
(maskelenmişken) "güvenli" kabul edilen `STUMBLE_KICK_PX=30`
tokezleme darbesi artık **gecikmeli bir düşmeye** yol açtığı görüldü —
frame 141'de (tokezlemeden ~50 kare sonra) düşme. İzolasyonla
doğrulandı: SADECE tokezleme (başka hiçbir darbe yokken) tek başına
frame 141'de düşürüyordu. Küçük darbe taraması yapıldı:

| Tokezleme | Sonuç |
|---|---|
| 5px | hayatta kaldı |
| 10px | hayatta kaldı |
| 15px | hayatta kaldı |
| 20px | frame 163'te düştü |
| 25px | frame 171'de düştü |

Gerçek absorbe/düşme sınırı **15px-20px arasında**. `STUMBLE_KICK_PX`
bu yüzden **15.0px**'e çekildi — taramada doğrulanmış en büyük
"hayatta kalan" değer. Eski 30px değeri, düzeltilmiş fizikte artık
"küçük bir tokezlemenin gerçekten absorbe edilmesi"ni değil, neredeyse
bir düşme sınırını temsil ediyordu; bu adımın göstermek istediği şeyi
(gerçek absorbsiyon) yanlış temsil ediyordu.

Bu arada raporlama kodundaki bir mantık hatası da düzeltildi: eski
"absorbe edildi mi?" kontrolü, düşme karesi tokezlemeden **20 kareden
fazla** sonraysa doğrudan "EVET" diyordu — ama bu, düşmenin asıl
sebebinin (o zamanki 30px'lik tokezleme) yanlışlıkla "hayır, aslında
sonraki büyük itkiden kaynaklandı" gibi raporlanmasına yol açıyordu
(oysa büyük itki t=7.0s/frame 210'da, düşme ise frame 141'de --yani
düşme büyük itkiden ÖNCE gerçekleşiyordu, ama eski kontrol büyük
itkinin karesini hiç dikkate almıyordu). Düzeltilmiş mantık artık
düşme karesini büyük-itki karesiyle (`bp`) karşılaştırıyor: düşme
`bp`'den ÖNCEYSE, sebep tokezlemenin kendisidir.

### Reddedilen bir "düzeltme": bacak-koordinasyon döngüsünün simetrikleştirilmesi

`STUMBLE_KICK_PX=15` ile yeniden çalıştırıldığında, tokezlemeden hemen
sonra (frame 105-127 arası) sağ bacağın üst üste **üç kez** hızlıca
adım attığı, sol bacağın ise aynı pencerede hiç adım atmadığı ve
kalça hızının geçici olarak 26px/kare'ye kadar salındığı gözlendi. İlk
bakışta bu, ana döngünün SIRALI (`for leg in (left_leg, right_leg)`)
çalışmasından kaynaklanan bir "asimetrik değerlendirme" hatası gibi
göründü ve döngü, her iki bacağın da AYNI güncelleme-öncesi anlık
görüntüyü kullanacağı şekilde "simetrik" hale getirildi. **Bu düzeltme
YANLIŞTI ve geri alındı:** simetrik hale getirilmiş döngü, iki bacağın
da aynı karede "diğeri henüz havada değil" görüp AYNI ANDA swing'e
geçmesine izin vererek tam olarak daha önce çözülmüş çift-havada
sorununu geri getirdi — doğrulama testinde frame 72'de (büyük itkiden,
frame 210'dan, ÇOK önce) ani bir düşmeye yol açtı. Kod, sıralı
değerlendirmeye geri döndürüldü (bkz. `demo/step14_active_biped.py`
içindeki yorum). Ayrıntılı log analizi, gözlenen "sağ bacağın üç kez
sekmesi" olayının aslında bir koordinasyon hatası DEĞİL, tokezlemenin
sol bacağın iniş hedefini (capture-point hesabıyla) beklenenden çok
daha ileriye (372.2px) fırlatmasının doğal bir sonucu olduğunu
gösterdi: kalça bu ileri-fırlatılmış destek noktasının altından/ötesine
geçerken ters-sarkaç dinamiği geçici olarak şiddetleniyor, sağ bacak
bunu birkaç hızlı adımda telafi edip normal ritme (frame ~138'den
itibaren, tekrar 10-11 karelik düzenli alternans) geri dönüyor. **Bu,
projenin "gerçekmiş gibi görünen ama yanlış olan bir tanı"yı, düzeltmeyi
uygulayıp doğrulamadan commit etmemenin önemini gösteren somut bir
örneği** — düzeltme geri alınmadan önce fark edilmeseydi, çift-destek
güvencesi sessizce kırılmış olacaktı.

### Nihai sonuçlar (12 saniyelik demo, `outputs/step14_active_biped.mp4`)

- 21 adım, düzenli ~10-11 kare periyotla alternan sol/sağ.
- Tokezleme (t=3.0s, 15px): hip_vx aralığı [1.90, 13.86] — **absorbe
  edildi** (düşme yok, büyük itkiden önce).
- Büyük itki (t=7.0s, 150px): **gerçek bir düşmeye** yol açtı (frame
  220, t=7.33s) — görsel QA'da kare 225'te gövde/kafanın (kırmızıya
  dönen) yere doğru devrildiği, "DUSTU" durumuna geçildiği doğrulandı.
- Büyük itkiden önceki kararlı yürüyüşte ortalama hız: 2.310px/kare
  (hedef: 2.0px/kare).

### 60+ saniyelik kararlılık testi (proje kuralı)

Büyük itki devre dışı bırakılıp (t=9999s'e ertelenerek) sadece sürekli
yürüyüş + t=3.0s'deki 15px'lik tokezleme ile **65 saniyelik** (1950
kare) bir koşu yapıldı:

- **185 adım**, tamamı düzenli alternan (sol/sağ), NaN/patlama yok.
- Tokezleme yine absorbe edildi; ondan sonraki tüm 1850 karede tek bir
  ek düzensizlik yaşanmadı (frame 105-127 arasındaki geçici "hızlı
  telafi" olayı bir daha tekrarlanmadı — tek seferlik bir tokezleme-
  toparlama tepkisi olduğu teyit edildi).
- Ortalama hız (tüm koşu boyunca): 1.955px/kare (hedef 2.0).
- Son kalça-y konumu: 147.55 (zemin: 330, düşme eşiği: 320 — güvenli
  aralıkta).

### Görsel QA

12 saniyelik demo videosundan 60/100/130/215/225/250/359. kareler
çıkarılıp incelendi: normal yürüyüş kareleri (60, 100, 130) tutarlı bir
"yuruyor" durumu ve hedefe yakın `hip_vx` gösteriyor; büyük itkiden
hemen sonraki kare (215, hip_vx=+18.55px/kare) henüz düşmemiş ama
belirgin biçimde hızlanmış bir gövde gösteriyor; düşmenin kaydedildiği
kareden hemen sonrası (225) durumun kırmızı "DUSTU" etiketine döndüğünü
ve gövde/kafa dairesinin gerçekten yere doğru devrildiğini gösteriyor;
sonraki kareler (250, 359) karakterin devrilmiş halde zemin boyunca
sürüklendiğini (kamera kalçayı takip ettiği için sahnede kalıyor)
doğruluyor — yani "düşme" gerçek bir geometrik çöküş, salt bir `bool`
bayrağı değil.

### Dürüst v1 sınırları (bilerek çözülmedi, açıkça belgelendi)

- **Compass gait basitleştirmesi:** o an destekte olan bacak, fizik
  açısından her zaman `arm_length` (184px) kadar tam uzatılmış kabul
  ediliyor — FABRIK, dizin GÖRSEL bükülmesini bu sabit uzunluğa karşı
  hâlâ çözüyor, ama alttaki ters-sarkaç fiziği dizin gerçek bükülü
  geometrisini hesaba katmıyor. Bu, robotik literatüründe standart bir
  v1 basitleştirmesidir.
- **Tek-destek koordinasyonu dış müdahaleyle sağlanıyor:**
  `ActiveFootPlantingLeg` kendi başına iki bacaklı farkında değil —
  çift-havada durumunu önlemek tamamen çağıran kodun her karede
  `other_leg_swinging` bayrağını doğru iletmesine bağlı (yukarıdaki
  "reddedilen düzeltme" bölümü, bunun ne kadar kırılgan olabileceğini
  gösteriyor).
- **`swing_duration_frames` hâlâ sabit**, kalça hızına göre
  ölçeklenmiyor — kullanıcının orijinal direktifinin "adım ne kadar
  ileriye atılacağı" kısmı (capture-point ile) uygulandı, ama "adımın
  ne kadar SÜRECEĞİ" hâlâ sabit bırakıldı.
- **`OMEGA0=0.045` ampirik, teorik `sqrt(g/L)=0.0188` değil** — bu
  motorun `dt=1.0` entegrasyon kısıtı, sürekli-zaman LIP formülünün
  doğrudan taşınmasını engelliyor (yukarıda ayrıntılı).

Commit: (bu ekin commit'i README ile birlikte yapılacak).


## 9. tur kullanıcı geri bildirimi: Dinamik Swing Time ve Havada Yeniden Hedefleme -- İKİSİ DE REDDEDİLDİ, ortak kök neden bulundu

Kullanıcı bu turda üç madde birden getirdi: (1) swing süresinin kalça
hızına göre kısalması gerektiği ("Sabit Swing Süresi Fiziğe Aykırıdır"),
(2) swing halindeki bacağın havadayken bir darbe geldiğinde hedefini
değiştirememesi ("Havada Körleşme" / Mid-Air Retargeting), (3) destekteki
bacağın rijit bir çubuk gibi davranmasının kalçanın "sekiyormuş" hissi
yaratması (compass gait / SLIP-model önerisi), ve açık soru olarak: hangisi
önce -- Dynamic Swing Time mi, Mid-Air Retargeting mi?

**Metodoloji notu:** bu turun ikisi de "iyi görünen ama izole test edilince
saklı bir bedeli çıkan" düzeltme kategorisine girdi -- yani bu turda HİÇBİR
kod değişikliği commit edilmedi (repo 8. turun sonundaki `783b910`
commit'inde kalıyor). Bunun yerine iki ayrı prototip izole olarak test
edildi, ikisi de reddedildi, ve reddedilme nedenleri AYNI kök mimari
sorunu işaret etti -- bu da kullanıcının üçüncü maddesinin (compass gait)
aslında birinci öncelik olması gerektiğini gösterdi.

### Prototip 1: Dynamic Swing Time (`_active_swing_duration`'ı tetiklenme
anındaki `hip_vx`'e ters orantılı ölçekleme)

`speed_factor = max(1.0, |hip_vx| / TARGET_VX)`,
`_active_swing_duration = max(3, swing_duration_frames / speed_factor)`
ile izole edildi (`/tmp/proto_swing/diag_dynamic_swing.py`, repoya HİÇ
yazılmadı). Sonuç -- görünüşte iyi: 8. turda belgelenen "3 art arda hızlı
sağ-bacak adımı" anomalisi (kare 105/116/127/138) tek bir hızlı düzeltme
adımına (kare 98, süre=9.14 kare) indi, düzenli yürüyüş ritmine daha
erken (108/119/130) dönüldü.

**Ama saklı bedel:** aynı tokezleme penceresindeki (kare 90-105) tepe
`hip_vx` **13.86px/kare'den 33.55px/kare'ye (~2.4x)** çıktı. Kök neden --
iz sürülerek bulundu: `demo/step14`'te destekteki ayağın `anchor` noktası
HER karede o an destekte olan bacağın `planted` konumuna DOĞRUDAN
pin'leniyor (`body.set_pinned_position(anchor, [stance_leg.planted[0],
GROUND_Y])`), ve bu `anchor` ile kalça arasında **rijit (`compliance=0.0`)
bir Verlet çubuğu** var. `swing_target`, "bacak ~10 kare sürecek"
varsayımıyla ileri projekte ediliyor; süre kısaltılınca ayak, kalça
GERÇEKTE o kadar ilerlemeden o UZAK hedefe ulaşıyor, ve rijit çubuk bu
mesafe farkını handoff anında TEK KAREDE kapatmaya çalışıyor -- işte
gözlenen hız sıçraması budur. Yani "daha az düzeltme adımı" metriği
gerçek bir iyileşme değil, DAHA BÜYÜK bir handoff sıçramasıyla satın
alınmış sahte bir iyileşmeydi.

### Prototip 2: Mid-Air Retargeting (`swing_target`'ı swing sırasında her
karede capture-point ile yeniden hesaplama)

Önce HAM (filtresiz) versiyon denendi -- `swing_start`/`swing_t` sabit,
`swing_target` her karede `hip_x + hip_vx/omega0 + swing_lead_margin`
formülüyle YENİDEN yazıldı (`retarget_lock_t=0.85`'e kadar). Bu, repoya
GEÇİCİ olarak yazılıp `step14`'ün 12 saniyelik standart demo senaryosuyla
test edildi (ardından HEMEN geri alındı -- aşağıda neden).

**Sonuç -- felaket:** karakter artık büyük itkiden (t=7.0s) ÖNCE, sadece
15px'lik tokezlemeden (t=3.0s) hemen sonra, kare 102'de düşüyordu (daha
önce hiç düşmüyordu). Tepe `hip_vx` **99.6px/kare**'ye fırladı. Kök neden:
`omega0=0.045` küçük bir sayı olduğu için, `hip_vx/omega0` formülü ham
(gürültülü, tek kareli) hız sıçramalarını ~22 kat büyütüyor -- örneğin
tokezleme darbesinin kendisi (bir kareliğine `prev_points`'i doğrudan
kaydırması) yapay bir tek-kare hız sivrisine yol açıyor, ve bu sivri
DOĞRUDAN (filtrelenmeden) hedefe yansıtılınca hedef bir karede yüzlerce
piksel zıplıyor, Bezier eğrisi de o karede aynı zıplamayı ayağa yansıtıyor.

**Düzeltilmiş (damped) versiyon denendi:** hedefe TAM zıplamak yerine
karede en fazla `MAX_RETARGET_STEP_PX` kadar yaklaşma (`4px/kare` ile
başlandı). Bu, tokezlemeyi eskisi gibi (`hip_vx` aralığı [1.91, 14.49] --
8. turdakiyle neredeyse aynı) absorbe etti VE "3 hızlı adım" kademesini
TEK bir adıma indirdi (kare 105) -- ilk bakışta Prototip 1'in faydasını,
onun bedeli olmadan sağlıyor gibi görünüyordu.

**Ama duyarlılık taraması ikinci, daha sinsi bir saklı bedel ortaya
çıkardı.** `MAX_RETARGET_STEP_PX` değeri arttıkça (2 / 4 / 8 / 16px),
büyük itkiden (t=7.0s, 150px) sonra düşme davranışı değişti:

| cap (px/kare) | büyük itkiden sonra | tepe hip_vx (tokezleme) |
|---|---|---|
| 2 | düştü (kare 220) | 14.40 |
| 4 | düştü (kare 221) | 14.49 |
| 8 | düştü (kare 224) | 14.49 |
| 16 | **DÜŞMEDİ** | 14.49 |

`cap=16`'nın "büyük itkiden bile hayatta kaldı" sonucu ilk bakışta en
etkileyici iyileşme gibi göründü -- ama proje disiplini gereği ("ilk
bakışta mükemmel görünen sonuca ham haliyle güvenme", bkz. 8. turdaki
`clamp_direction` bulgusu) kare kare `hip_y` izlendi (`/tmp/proto_swing/
diag_bigpush_trace.py`). Bulgu: kare 220-221 arasında **`hip_y` TEK
KAREDE 308.14'ten 162.99'a (145px) zıplıyor**, aynı anda `hip_vx`
+2.61'den **-59.24**'e düşüyor. Bu, "karakter dengesini akıllıca kurtardı"
DEĞİL -- retargeting'in yeniden hedeflediği uzak nokta ile kalçanın
GERÇEKTE bulunduğu yer arasındaki farkı, yine o rijit `anchor` çubuğunun
TEK KAREDE zorla kapatması. `dustu=False` sonucu, düşme eşiğinin bu
fiziksel-olmayan sıçramayla "tesadüfen" atlatılmasından ibaret -- gerçek
bir dinamik kurtarma refleksi değil. Yani `cap=16`, bu turun EN tehlikeli
bulgusuydu: yanlışlıkla "başarı" olarak raporlanabilecek bir maskeleme
hatasıydı, ve iz sürme yapılmasaydı yanlış bir kazanç olarak
belgelenebilirdi.

### Ortak kök neden -- ve neden kullanıcının 3. maddesi (compass gait/SLIP)
aslında ÖNCELİKLİ olmalı

Her iki prototip de AYNI mekanizmaya çarptı: `anchor`-kalça arasındaki
**rijit (`compliance=0.0`), ANINDA yeniden pinlenen** Verlet çubuğu, "bacak
swing_duration kadar sürede swing_start'tan swing_target'a düzgünce
gider" varsayımını ÇİĞNEYEN her durumda (süre kısaltılırsa VEYA hedef
canlı güncellenirse), o çiğnemeyi TEK KAREDE, fiziksel olmayan bir
kalça-sıçramasıyla telafi ediyor. Hatta bu mekanizmanın KÜÇÜK bir versiyonu
zaten mevcut, DEĞİŞTİRİLMEMİŞ koddaki NORMAL (bozulmasız) yürüyüşte bile
gözlemlenebiliyor -- baseline izolasyonunda kare 11'de `hip_y` 153.72'den
147.32'ye (6.4px), kare 20-22 arasında 146.03'ten 149.03'e (3px) sıçrıyor;
işte kullanıcının 3. maddesinde sözünü ettiği "kalçanın kilitlenmiş gibi
yukarı sekiyormuş hissiyatı" TAM OLARAK budur, ve bunun BÜYÜK/felaket
versiyonu, dinamik swing süresi veya havada yeniden hedefleme
denendiğinde ortaya çıkıyor.

**Sonuç:** Dynamic Swing Time ve Mid-Air Retargeting, kullanıcının
sorduğu gibi "önce hangisi" sorusunun cevabı DEĞİL -- ikisi de aynı
duvara (rijit, anlık handoff) çarpıyor. Kullanıcının 3. madde olarak
listelediği compass-gait/SLIP eleştirisi ("dizden esneyerek amortisör
gibi") aslında bu ikisinin ÖN KOŞULU: `anchor`-kalça çubuğuna bir miktar
uyumluluk (compliance > 0, ya da handoff'un birkaç kareye yayılan yumuşak
bir geçişle yapılması) eklenmeden, ne swing süresi ne de swing hedefi
güvenle dinamikleştirilebilir -- ikisi de aynı sert duvara çarpıp ya
açıkça düşüyor (cap<16, süre kısaltma) ya da GÖRÜNMEYEN bir sıçramayla
maskeleniyor (cap=16).

### Doğrulama

- Her iki prototip de İZOLE test edildi (`/tmp/proto_swing/`), REPOYA
  KOMİT EDİLMEDİ. Ham Mid-Air Retargeting versiyonu kısaca repoya
  yazılıp `step14`'ün 12 saniyelik demo senaryosuyla koşturuldu, felaket
  sonuç (kare 102'de erken düşme) görülür görülmez `git diff --stat`
  ile doğrulanarak backup'tan geri alındı -- repo bu turun SONUNDA da
  `783b910` ile bit-bit aynı (`git status --short` boş).
- step1-13 kanaryaları bu turda hiç risk altında değildi: hem Dynamic
  Swing Time hem Mid-Air Retargeting SADECE `physics/active_gait.py`
  içinde (yani sadece `step14`'ün kullandığı sınıfta) denendi;
  `grep -rl "active_gait\|ActiveFootPlantingLeg"` bunun repoda SADECE
  `active_gait.py` ve `step14_active_biped.py` tarafından kullanıldığını
  doğruluyor.

### Dürüst v1 sınırları (bu tur, hiçbir kod değişikliği yapılmadan)

- Motor hâlâ 8. turun sonundaki halinde: swing süresi sabit, swing
  hedefi sadece tetiklenme anında bir kez hesaplanıyor.
- Kullanıcının orijinal sorusuna ("önce hangisi?") doğrudan cevap
  VERİLEMEDİ -- bunun yerine ikisinin de şu anki mimaride güvenle
  uygulanamayacağı, ve compass-gait/SLUMP esnekliğinin önce gelmesi
  gerektiği sayısal olarak gösterildi.
- `cap=16`'nın "büyük itkiyi absorbe etti" sonucu KASITLI OLARAK
  reddedildi ve bir kazanç olarak raporlanmadı -- yukarıdaki tabloya
  sadece karşılaştırma için dahil edildi.
- Bir sonraki adım için öneri (uygulanmadı, sadece belgelendi): `anchor`
  çubuğuna küçük bir compliance (>0) eklemek ya da handoff'u birkaç
  kareye yayılan yumuşak bir interpolasyona çevirmek -- bu, kullanıcının
  SLIP-modeli önerisiyle doğrudan örtüşüyor ve muhtemelen hem Dynamic
  Swing Time'ı hem Mid-Air Retargeting'i GÜVENLE mümkün kılacak.

Commit: (bu ekin commit'i README ile birlikte yapılacak; bu turda
`physics/`, `demo/` altında HİÇBİR dosya değişmedi -- sadece bu belge).

## 10. tur: Anchor-Kalça Çubuğuna Esneklik (Compliance) -- Kısmi ama Doğrulanmış İyileştirme

Bu tur, kullanıcının 9. turdaki bulguları değerlendiren ve doğrulayan bir
üçüncü taraf incelemesine cevaben yapıldı. İnceleme motoru üç eksende
puanladı: Mimari Kararlılık ve İzolasyon 95/100, Fiziksel Çözümleyici
85/100, Dinamik Refleksler ve Biyomekanik 70/100. İnceleme şunları
açıkça teyit etti:

- Mid-Air Retargeting'in 145px'lik "teleport" sonucunu bir başarı gibi
  raporlamak yerine reddedip geri almam, "kıdemli bir mühendisin yapacağı
  türden bir kalite kontrolü" olarak değerlendirildi.
- 9. turdaki kök-neden teşhisi ("rijit anchor-kalça çubuğu, iki
  özelliğin de önünü kesen ortak duvar") "%100 doğru" bulundu.
- Önerilen sıradaki adım -- `anchor` çubuğuna `compliance > 0` eklemek,
  yani çubuğu bir "şok emiciye" dönüştürmek -- doğrudan onaylandı.

Bu tur, tam olarak bu onaylanmış adımı uyguladı, sıkı biçimde ölçtü, ve
sonucu -- hem kazanımları hem sınırlarını -- dürüstçe raporluyor.

### Mekanizma: `physics/verlet.py` içinde zaten var olan compliance desteği

Yeni motor kodu YAZILMADI -- `add_stick()` ve `_satisfy_sticks()` içinde
zaten (önceki bir turda squash & stretch için eklenmiş) bir `compliance`
parametresi mevcuttu:

```python
def add_stick(self, i, j, length=None, compliance=0.0): ...

def _satisfy_sticks(self):
    ...
    diff = (dist - rest_length) / dist
    if compliance:
        diff *= max(1.0 - compliance, 0.0)
    ...
```

`compliance=0.0` iken çubuk her `step()` çağrısında (8 gevşetme
iterasyonu ile) tam mesafeyi ANINDA dayatıyor. `compliance > 0.0`
verildiğinde, her iterasyonda düzeltmenin sadece bir kısmı uygulanıyor;
kalan sapma sonraki karelere taşınıyor -- yani `anchor` her yeni
temas noktasına yeniden pinlendiğinde, kalça oraya TELEPORT etmek
yerine birkaç kareye yayılan yaylı bir geçişle "akıyor". Bu, kullanıcının
SLIP-modeli önerisindeki "diz üzerinden yaylı bağlantı" fikrinin motor
seviyesinde karşılığı. Değişiklik, tek bir çağrı noktasında:

`demo/step14_active_biped.py`, satır 129:
`sys_.add_stick(idx["anchor"], idx["hip"], length=ARM_LENGTH, compliance=0.0)`
→ `compliance=0.85`

### Doğrulama: değer seçimi (izole, repo'ya dokunmadan)

Doğru değeri bulmak için `compliance` bir dizi seviyede, `build_body()`
monkeypatch'lenerek (repoya hiçbir dosya değişmeden) tarandı.

**1) Taban tarama (12s standart senaryo, tek tokezleme + büyük itki):**

| compliance | adım sayısı | tokezleme hip_vx aralığı | büyük itki sonrası | ort. hip_vx |
|---|---|---|---|---|
| 0.00 | 21 | [1.90, 13.86] | düştü @220 | 2.310 |
| 0.30 | 21 | [1.90, 13.92] | düştü @221 | 2.355 |
| 0.50 | **27 (anomali)** | [1.90, 13.97] | düştü @233 | 2.355 |
| 0.70 | 21 | [1.91, 14.13] | düştü @222 | 2.204 |
| 0.85 | 21 | [1.74, 14.53] | düştü @224 | 2.080 |
| 0.92 | **26 (anomali)** | [1.70, 14.69] | düştü @246 | 2.068 |

Önemli bulgu: compliance ile adım sayısı/kararlılık İLİŞKİSİ monoton
DEĞİL -- 0.50 ve 0.92 anormal adım sayıları üretti (27, 26), 0.30/0.70/
0.85 ise temiz kaldı. Bu, projede tekrar tekrar görülen dersi doğruluyor:
kararlılık eşikleri formülle değil, doğrudan ölçümle bulunmalı.

**2) 9. turun reddedilen "cap=16 Mid-Air Retargeting" stres senaryosu
yeniden test edildi (sadece karşılaştırma amaçlı -- bu özellik hâlâ
repoya alınmadı):**

| compliance | max hip_y sıçraması | adım sayısı | gait alternation |
|---|---|---|---|
| 0.00 | 145.15px @f221 | 33 | görünüşte tamam (9. turda reddedilen sahte "başarı") |
| 0.70 | 114.86px @f221 | 34 | mükemmel alternation |
| 0.85 | 78.41px @f221 | 33 | mükemmel alternation |

**3) Daha yüksek compliance denendi (0.90-0.99), aynı stres
senaryosunda -- sıçrama küçülmeye devam ediyor ama YENİ bir bozulma
ortaya çıkıyor:**

| compliance | max hip_y sıçraması | adım sayısı | gait alternation |
|---|---|---|---|
| 0.90 | 57.72px @f222 | 32 | **bozuk** (aynı bacağın art arda sallanması) |
| 0.93 | 47.49px @f222 | 28 | **bozuk** |
| 0.95 | 34.58px @f222 | 26 | **bozuk** |
| 0.97 | 25.65px @f223 | 32 | **bozuk** |
| 0.99 | 22.91px @f211 | 33 | **bozuk** |

NaN hiçbir seviyede görülmedi. Ama sıçrama ASLA ihmal edilebilir bir
değere inmiyor (0.99'da bile hâlâ 22.91px), ve 0.90 üstü değerler
sıçramayı daha çok bastırmaya çalışırken sol/sağ adım alternation'ını
bozan YENİ bir kararsızlık modu getiriyor. **Sonuç: compliance TEK
BAŞINA Mid-Air Retargeting/Dynamic Swing Time'ı tam güvenli hale
getirmiyor -- sadece hasarı azaltıyor.**

**4) 65 saniyelik uzun-koşu kararlılık testi** (`BIG_PUSH_T=9999`,
sadece kararlı yürüyüş + t=3.0s'deki tek tokezleme), compliance=0.0 ile
0.85 karşılaştırıldı: HER İKİSİ de tam 185 adım, kusursuz sol/sağ
alternation, hiç NaN yok, neredeyse özdeş son `hip_y` (147.55 / 147.34)
ve ortalama `hip_vx` (1.955 / 1.929, hedef 2.0'a yakın). Bu,
compliance=0.85'in taban (retargeting'siz) senaryoda güvenli ve
gerileme yaratmadığını güçlü biçimde doğruluyor.

**5) Günlük "kalça sekmesi" karşılaştırması** (kare 8-35, normal
yürüyüş): compliance=0.0'da `hip_y` bir handoff anında TEK KAREDE
153.72 → 147.32 (6.4px) sıçrıyor. compliance=0.85'te AYNI geçiş
152.25 → 150.94 → 149.2 → 147.63 → 146.56 şeklinde BİRKAÇ kareye
yayılıyor -- niyet edilen "şok emici" etkisi tam olarak bu. Kare 9-199
aralığında maksimum kare-başı `|Δhip_y|` 23.077px'ten 7.053px'e düştü
(~3.3x azalma), ortalama ise neredeyse aynı kaldı, hatta çok hafif arttı
(0.800px → 0.846px) -- çünkü esnek bir çubuk hiçbir zaman tam
"dinlenme"de değil, her karede küçük bir düzeltme yapmaya devam ediyor.
Bu takas (çok daha düşük en-kötü-durum sıçraması, ihmal edilebilir
düzeyde daha yüksek ortalama titreşim) net bir kazanç olarak
değerlendirildi: göze çarpan "sekme hissi" tekil büyük sıçramalardan
kaynaklanıyordu, 0.8px'lik alt-piksel düzeyindeki sürekli titreşim
görsel olarak fark edilmiyor.

**Karar: `compliance=0.85` benimsendi** -- taban senaryoda doğrulanmış
güvenli/gerilemesiz davranış, günlük kalça-sekmesinde ölçülebilir
iyileşme, ve reddedilen Mid-Air Retargeting stres testinde (sadece
karşılaştırma amaçlı) gait alternation'ı bozmadan sıçramayı ciddi
oranda azaltması nedeniyle.

### Gerçek koda uygulama ve tam doğrulama

Bu turda -- önceki turların aksine -- değişiklik GERÇEKTEN repoya
yazıldı: `demo/step14_active_biped.py` satır 129, `compliance=0.0` →
`compliance=0.85`. Ardından TÜM doğrulama takımı, izole monkeypatch
değil, bu gerçek dosya üzerinde yeniden koşturuldu:

- **12s standart demo:** 21 adım, tokezleme hip_vx aralığı
  [1.74, 14.53] (absorbe edildi), büyük itki sonrası düştü @224,
  ortalama hip_vx 2.080px/kare -- izole taramadaki tahminle BİREBİR
  eşleşiyor.
- **65s kararlılık testi:** 185 adım, kusursuz alternation, NaN yok,
  son hip_y 147.34, ortalama hip_vx 1.929px/kare -- yine izole
  taramayla birebir eşleşiyor.
- **Görsel QA:** büyük itki sonrası düşme anının (kare 220-225)
  kareleri ffmpeg ile çıkarıldı ve incelendi -- karakter beklenen
  şekilde çöküyor (büyük itki testi zaten kasıtlı olarak
  absorbe-edilemeyecek kadar güçlü), iskelet çiziminde hiçbir
  bozulma/artefakt yok, renkler ve etiketler önceki turlarla tutarlı.
- step1-13 kanaryaları risk altında değildi: değişiklik SADECE
  `step14_active_biped.py` içindeki tek bir `add_stick` çağrısında.

### Dürüst v1 sınırları

- Bu, kullanıcının orijinal "önce hangisini çözelim?" sorusuna HÂLÂ
  doğrudan "ikisi de artık güvenli" cevabını VERMİYOR. Compliance=0.85,
  Dynamic Swing Time ve Mid-Air Retargeting'i güvenli hale getirmek için
  gereken ÖN KOŞULU iyileştirdi, ama TEK BAŞINA yeterli değil: aynı
  cap=16 stres senaryosunda sıçrama 145px'ten 78px'e indi, ama SIFIRA
  inmedi. Bu iki özellik henüz yeniden denenmedi/repoya alınmadı.
- Sıçramayı compliance'ı daha da yükselterek (≥0.90) azaltmaya çalışmak
  YENİ bir bozulma getiriyor (gait alternation kırılması) -- yani
  "daha fazla compliance = daha iyi" formülü YANLIŞ; 0.85 empirik
  olarak bulunmuş bir denge noktası, teorik bir optimum değil.
  Sıçramanın geriye kalan kısmı muhtemelen `anchor`'ın kendisinin
  yeniden-pinleme ANI'nın da yumuşatılmasını (sadece çubuğun
  sertliğini değil) gerektiriyor -- bu henüz denenmedi.
  Squash & stretch amaçlı diğer compliance kullanımları bu turda
  DEĞİŞTİRİLMEDİ; sadece anchor-kalça çubuğu güncellendi.
- Günlük yürüyüşte ortalama kare-başı titreşim çok hafif arttı
  (0.800px → 0.846px) -- ihmal edilebilir görüldü, ama not edilmeye
  değer bir takas.

Commit: bu README güncellemesiyle birlikte, `demo/step14_active_biped.py`
satır 129 değişikliğini içeren commit.

## 11. tur: Rest Length Lerping denemesi -- REDDEDİLDİ (kelebek etkisi: Capture Point zamanlamasını bozuyor)

10. turdan sonra kullanıcı, geriye kalan ~7px'lik günlük kalça sekmesini
gidermek için iki somut mimari öneri getirdi: (1) Çift Destek Fazı ile
Ağırlık Aktarımı (iki bacağa aynı anda anchor + zıt yönde compliance
rampası), (2) Rest Length Lerping (`anchor`-kalça çubuğunun `length`
parametresini handoff anındaki gerçek mesafeden `ARM_LENGTH`'e birkaç
karede yumuşakça çekmek). Analiz istendi: hangisi mevcut mimariye daha
kolay/az riskli entegre edilir? DURUM: **Rest Length Lerping izole olarak
prototiplendi, ölçüldü, ve kullanıcıyla birlikte REDDEDİLDİ -- bu turda
`physics/` veya `demo/` altında HİÇBİR kalıcı kod değişikliği yok.**

**Neden önce Rest Length Lerping seçildi:** kodun gerçek okunmasıyla
(`demo/step14_active_biped.py` satır 236: `body.set_pinned_position(anchor,
[stance_leg.planted[0], GROUND_Y])` + `physics/gait.py` satır 194:
`self.planted = self.swing_target.copy()`) sıçramanın TAM kaynağı
doğrulandı: `leg.planted`, bacak havadayken sabit kalıp SADECE dokunma
anında TEK KAREDE yeni değere atlıyor, `anchor` da bunu birebir izlediği
için `hip`-`anchor` mesafesi o karede aniden bozuluyor. Weight Blending
(çift destek), `gait.py`/`active_gait.py`'nin `other_leg_swinging`
tek-destek garantisine (8. turda "simetrik hale getirme" denemesiyle bir
kere ANLIK ÇİFT-HAVADA düşmeye yol açmıştı) dokunma riski taşıyordu; Rest
Length Lerping ise `sys_.sticks` listesindeki tek bir tuple'ı (zaten 10.
turda aynı yöntemle mutasyona uğratılmıştı) değiştirmekle sınırlı, hiçbir
yeni nokta/state-machine değişikliği gerektirmiyordu -- bu yüzden daha
düşük riskli görünüyordu.

**Prototip ve ölçüm (`_proto11/`, repo'ya hiç yazılmadan, `git status`
bu turun sonunda temiz):** `HANDOFF_LERP_FRAMES` 0'dan 15'e tarandı.
Birincil hedefte (günlük kalça sekmesi, kare 9-199 `|Δhip_y|`) gerçek bir
iyileşme ÖLÇÜLDÜ:

| lerp (kare) | ort. \|Δhip_y\| | maks. \|Δhip_y\| |
|---|---|---|
| 0 (10. tur, mevcut) | 0.848 | 7.053 |
| 6 | 0.722 | 5.961 |
| 7 | 0.473 | 4.265 |
| 8-9 | ~0.40 | ~3.0 |

Ama tam olay-listesi (`step_events`) incelendiğinde İKİNCİ, gizli bir
maliyet ortaya çıktı: tokezleme (t=3.0s) sonrası normal koşulda (lerp=0)
AYNI bacak sadece 1 kere fazladan adım atıyordu (`(105,'r'),(120,'r')` --
zaten 8./9. turda belgelenen, kabul edilmiş bir davranış). Lerp
eklenince bu **2-4 ardışık aynı-bacak adımına** çıkıyor (lerp=6'da 3,
lerp=7'de 4). Kök neden: rest-length lerping kalçanın handoff sırasındaki
YÜKSEKLİK yörüngesini değiştiriyor, bu da DİĞER bacağın capture-point
(`xcp = hip_x + hip_vx/omega0`) hesabının zamanlamasını kaydırıp adım
kararını erken/geç tetikliyor -- yani bir metrik (geometrik sıçrama)
düzeltilirken, ÖNGÖRÜLMEMİŞ şekilde başka bir sistemin (yürüyüş
zamanlama kararı) davranışına karışılmış oluyor. **Tavan da monoton
değil** (10+ karede adım sayısı 21'den 28-30'a anomaliye kayıyor) --
compliance'ta 10. turda görülen aynı "daha fazlası daha iyi değil"
deseni burada da tekrarlandı. 65 saniyelik uzun-koşu testi (lerp=0/6/7)
NaN üretmedi, ortalama hız/adım sayısı benzer kaldı -- yani bu bir
KARARSIZLIK değil, sadece tokezleme-sonrası gait düzgünlüğünde gerçek
bir bedel.

**Karar (kullanıcıyla birlikte, gerekçeli):** kullanıcı üç seçeneği
("takası kabul et", "capture point'i dondur", "reddet ve 10. turda kal")
değerlendirip ikinciyi de ("Capture Point'i lerp penceresinde dondurmak")
açıkça reddetti -- bu, bir hatayı örtmek için başka bir alt sisteme
yapay bir körlük eklemek anlamına gelirdi: karakter tam o 6-7 karelik
pencerede gerçek bir dış darbe alırsa, CP donuk olduğu için tepki
veremeyip düşerdi (fiziği "kandıran" bir yama, üzerine yeni özellik
inşa edildiğinde çökmeye mahkumdur -- tıpkı 9. turda reddedilen sahte
"başarı"lar gibi). **Aynı bacağın ardışık 3-4 kez adım atması, iki
ayaklı yürüyüş yanılsamasını görsel olarak bozacak kadar ciddi bir
bedel** -- kararlılığın bedeli anatominin bozulması olamaz. Sonuç:
Rest Length Lerping REDDEDİLDİ, `compliance=0.85` (10. tur) motorun bu
mimarisi için ampirik olarak doğrulanmış "altın oran" olarak kabul
edildi; geriye kalan ~7px'lik sekme artık bir HATA değil, eklemlerin
kütleyi karşıladığı andaki doğal/organik esneme payı (give) olarak
değerlendiriliyor.

**Doğrulama:** prototip TAMAMEN `_proto11/` altında izole çalıştı (repo
dosyalarına hiç yazılmadı), tüm ölçümler bu klasördeki tek-kullanımlık
tanılama scriptleriyle yapıldı, tur sonunda `rm -rf _proto11` ile
silindi -- `git status --short` bu turun başında da sonunda da temiz.

**Dürüst v1 sınırları / açık kalan konu:** Dynamic Swing Time ve
Mid-Air Retargeting hâlâ repoya alınmadı (9. tur). Kalan ~7px'lik sekme
artık KASITLI OLARAK "çözülmeyecek" ilan edildi (organik esneme payı) --
yani bunun sıfırlanması bundan sonra bu projenin bir hedefi DEĞİL.
`anchor` mimarisinin (tek nokta, ayrık handoff) kendisi hâlâ aynı, ama
üstüne inşa edilecek her yeni özellik artık compliance=0.85'in üstüne
DAHA FAZLA "geometrik sıçrama gizleme" yaması eklemek yerine, gerçek bir
çift-destek/ağırlık-aktarım mimarisi (daha büyük, ayrı bir round)
gerektirecek.

Commit: bu README güncellemesiyle birlikte (kod değişikliği yok).

## 12. tur: Dinamik Salınım Süresi v2 + Havada Yeniden Hedefleme v2 -- defter kapandı (İKİSİ DE KABUL EDİLDİ)

**Kullanıcının önerisi:** 9. turda reddedilen iki özelliği (Dinamik Salınım
Süresi, Havada Yeniden Hedefleme), 10. turun `compliance=0.85` yumuşatma
dersini uygulayarak, ANİ/tek-seferlik değil SÜREKLİ/yumuşatılmış biçimde
yeniden dene: salınım süresi her karede `hip_vx`'e göre yeniden hesaplanıp
sürekli sürüklensin; havadaki hedef kaymaları anlık ışınlanma değil, kare
başına sınırlı hızda kayan sönümlü (damped) bir vektörle yapılsın.

**Mekanizma (bkz. `physics/active_gait.py` -- tam kod ve gerekçe orada):**

- **Dinamik Salınım Süresi (DST) v2:** `_active_swing_duration`, her karede
  o anki `hip_vx`'ten hesaplanan bir hedef değere doğru birinci-derece bir
  gecikmeyle (`DST_SMOOTH_RATE=0.15`) sürüklenir -- asla bir kareden
  diğerine sıçramaz. `DST_MIN_DURATION=4`/`DST_MAX_DURATION=20` kare
  sınırları patolojik uçları keser.
- **Havada Yeniden Hedefleme (MAR) v2:** salınımın sadece ilk yarısında
  (`MAR_LOCK_IN_T=0.5`), yeni bir darbe capture-point'i (`xcp`) değiştirmişse
  `swing_target`, kare başına en fazla `MAR_MAX_STEP_PX=8` piksel kayan
  sönümlü bir vektörle yeni hedefe doğru itilir. Salınımın ikinci yarısında
  hedef DONAR -- inişin hemen öncesinde ayağın planlayabileceği kararlı bir
  hedef bırakmak için.

**Doğrulama, 3 ayrı senaryo (izole `_proto12/`, gerçek dosyalardan türetilen
kopyalar üzerinde):**

1. *Normal yürüyüş + büyük itki (150px, t=7.0s):* DST tek başına tepe
   hip_vx'i 14.58-17.51 aralığında tuttu (9. turun reddedilen ANİ
   versiyonunun 13.86→33.55 sıçramasına kıyasla sağlıklı). 65s uzun-süre
   kararlılık: NaN yok, 185 adım (özellik-siz haliyle BİREBİR aynı), max
   ardışık aynı-bacak adımı=1.

2. *Tokezleme (15px, t=3.0s) -- darbe HER ZAMAN destek/stance fazında:*
   MAR'ın etkisi ölçülemeyecek kadar küçük (tokez_peak 14.53→14.69).
   Sebebi basit: darbe zaten YERDEYKEN geliyor, henüz başlamamış bir
   salınımın hedefini "yeniden" hedefleyecek bir şey yok. **Bu senaryonun
   MAR'ı test etmek için yanlış senaryo olduğu anlaşıldı** -- adından da
   belli olduğu gibi "Havada Yeniden Hedefleme", darbe fiilen bir ayak
   havadayken gelmeli.

3. *Havada-iken darbe (yeni, doğru senaryo -- darbe bir bacak fiilen
   swing_t=0.5'te havadayken veriliyor):* burada MAR'ın gerçek, ölçülebilir
   faydası ortaya çıktı:

   | Ölçüm (15px darbe, havadayken) | v1 (MAR'sız) | MAR cap=8, lockin=0.5 |
   |---|---|---|
   | İniş noktası (px) | 153.87 (darbeden ETKİLENMİYOR) | 167.22 (gerçek yakalama noktasına kayıyor) |
   | Tepe \|hip_vx\| | 14.68 | 14.85 (~%1 artış) |
   | Tokezleme-sonrası max ardışık aynı-bacak adımı | 2-4 | **1-2 (İYİLEŞME)** |

   40px'lik daha sert bir darbede aynı desen büyüyerek tekrarlandı (iniş
   153.87→175px, tepe hip_vx 39.81→43.92, ~%10 artış). `cap=16` denendiğinde
   bedel (tepe hip_vx 47.39) orantısız büyüdü, iniş-doğruluğu kazancı ise
   çok az arttı (167→175px) -- 9./10./11. turların öğrettiği "daha agresif
   parametre = gizli bedel" deseniyle tutarlı, bu yüzden `cap=8` seçildi.

   65s uzun-süre kararlılık (MAR tek başına ve DST+MAR birlikte): NaN yok,
   adım sayısı v1 ile birebir aynı (185) -- rahatsızlık olmadan yürüyen
   kararlı yürüyüşte MAR'ın etkisi doğal olarak sıfıra yakın kalıyor (atıl
   değil, sadece gerekmiyor).

**Önceki (yanlış) test senaryosundaki adım-sayısı anomalisi açıklandı:**
Tokezleme testinde DST+MAR kombinasyonu toplam adım sayasında bir sıçrama
gösteriyordu (26 vs 21). Bu, gerçek bir etkileşim hatası DEĞİL: adım sayacı
360 karelik TÜM simülasyonu (150px'lik büyük itkiden SONRAKİ çöküş/ragdoll
fazı dahil) sayıyor, ve DST+MAR kombosu büyük itkiden sonra ~14 kare daha
uzun "debelenip" 6 fazla adım attıktan sonra aynı şekilde düşüyor (150px
zaten hayatta kalma eşiğinin üstünde, kaçınılmaz). İtkiden ÖNCEKİ pencereye
bakıldığında (asıl anlamlı pencere) tüm konfigürasyonlar 18-20 adımda
neredeyse özdeş.

**Karar: her iki özellik de kabul edildi.** DST v2 katıksız bir kazanç
(hiçbir ölcümde v1'den kötü değil). MAR v2'nin faydası senaryoya bağlı
(sadece gerçekten havadayken gelen darbelerde ölçülebilir) ama gerçek ve
doğrulanmış; bedeli küçük ve iyi karakterize edilmiş (tepe hız artışı). Bu,
11. turda reddedilen Rest Length Lerping'in TAM TERSİ: o, ana metriği
(ardışık-adım kararlılığı) KÖTÜLEŞTİRİYORDU -- burada ana metrik İYİLEŞİYOR.

**Gerçek dosyaya uygulama ve tam doğrulama:** `physics/active_gait.py`
(`ActiveFootPlantingLeg.update()`) DST v2 + MAR v2 ile güncellendi (bkz.
dosyanın kendi docstring'i -- tam mekanizma ve ölçümler orada da
tekrarlanıyor). `demo/step14_active_biped.py` değişmedi (arayüz/`update()`
imzası aynı kaldı). 12s demo, 65s kararlılık ve görsel QA gerçek dosya
üzerinde tekrarlandı ve `_proto12/` sonuçlarıyla birebir eşleşti (65s:
ort_hip_vx=1.904, son_hip_y=148.66 -- prototip ile aynı basamağa kadar
ayıni).

**Doğrulama:** prototip TAMAMEN `_proto12/` altında izole çalıştı, tur
sonunda `rm -rf _proto12` ile silindi -- `git status --short` bu turun
başında da sonunda da temiz.

## 12. tur eki: Sıradaki "Mükemmel" statü özellikleri (PLANLANDI, henüz uygulanmadı)

Kullanıcının Dinamik Salınım Süresi/Havada Yeniden Hedefleme defterini
kapattıktan SONRA eklenmesini istediği dört yeni özellik, roadmap'e
KAYDEDİLDİ ama bu turda UYGULANMADI:

1. **Yerden Kalkma (Active Ragdoll Stand-Up):** karakter tamamen devrilip
   tam ragdoll haline gelirse, eklemlere içsel tork (motor-spring)
   uygulayarak kollarından destek alıp ayağa kalkan prosedürel bir "get-up"
   dizisi.
2. **Topuk-Burun Teması (Heel-to-Toe Roll):** ayağı tek nokta temasından
   bir segmente/kapsüle çevirip topuk-çarpma → taban-yükleme → parmak-ucu
   itiş (toe-off) fazlarını modellemek.
3. **Prosedürel Kol Salınımı (Angular Momentum Cancellation):** kolların
   sadece acil denge durumunda değil, NORMAL yürüyüşte de bacakların
   açısal momentumunu sıfırlamak için ters-asimetrik salınması.
4. **Kinetik Sürtünme Sınırı (Slipping):** itki (thrust) kuvveti zeminin
   statik sürtünme sınırını aşarsa (örn. buzlu zemin), ayağın geriye doğru
   kaymaya başlaması.

Bu dört özellik motoru "Mükemmel" statüsüne taşıyacak; her biri kendi
izole prototip/doğrulama turunda ele alınacak.


## 13. tur: Kinetik Sürtünme Sınırı (Slipping) -- zemin artık sonsuz tutunmuyor

**Kullanıcının tespiti:** motorun yürüyüş fiziğinde büyük bir taviz (loophole)
vardı -- bacağın kalçayı ileri itmek (push-off) için uyguladığı kuvvet ne
kadar agresif olursa olsun, ayak zeminle arasında SONSUZ bir statik sürtünme
varmış gibi davranıyor, asla geriye kaymıyordu. İstenen: itki kuvveti zemin
sürtünme katsayısı (mu) ile o an bacağa binen normal kuvvetin çarpımını
(`F_thrust <= mu * F_N`) aşarsa, ayak `stance`'tan çıkıp geriye doğru
kinetik olarak kaymaya (slip) başlamalı -- buzlu/düşük sürtünmeli bir
zeminde "patinaj çekme" ve "ayağı kayıp düşme" dinamiği.

**Mekanizma (bkz. `demo/step14_active_biped.py` ve `physics/active_gait.py`
-- tam kod ve gerekçe orada):**

`physics/environment.py`'nin `Terrain` sınıfı (önceki turlardan hazır duran,
ama `step14`'e hiç kablolanmamış altyapı -- step8-13'ün eski
`collide_ground()` tabanlı demolarında kullanılıyordu) ilk kez `step14`'e
bağlandı. Her karede, o an zeminde duran bacağın konumundaki sürtünme
katsayısı (`terrain.friction_fn(planted_x)`) okunuyor:

  * `F_MAX = mu * THRUST_CAP` -- **dürüst basitleştirme:** gerçek
    `F_N = m*g` hesaplamak yerine (bu motorda itki zaten doğrudan
    kinematik bir hız-değişimi, gerçek bir F=ma kuvveti DEĞİL -- bu
    yüzden gerçek bir normal kuvvetle boyutsal olarak karşılaştırılamaz),
    zaten var olan `THRUST_CAP`'i (bu aynı birim sisteminde anlamlı "kas
    gücü tavanı") referans-normal-kuvvet olarak kullandık. Sonuç sezgisel
    bir ölçek: `mu=1.0` "zemin en az kas kadar güçlü tutuyor" (asla
    kaymaz), `mu<1.0` kaymaya başlatıyor (buz için `mu=0.15`). Bu,
    `OMEGA0`/`dt=1.0` gibi önceki turlarda da benimsenen "teorik değil,
    dürüstçe ampirik" yaklaşımla aynı ruhta.
  * İstenen itki (`desired_thrust`, mevcut `THRUST_CAP=1.5` ile
    sınırlanmış P-kontrolcü çıktısı) `F_MAX`'ı aşarsa: kalçaya SADECE
    `F_MAX` kadarı uygulanır (`applied_thrust`), kalan (`excess`) kuvvet
    `ActiveFootPlantingLeg.apply_slip()` (yeni, tek satırlık metod) ile
    ayağın `planted` konumuna, itkinin TERS yönünde bir kayma olarak
    aktarılır (`SLIP_GAIN=1.0` ile ölçeklenir) -- gerçek bir ayağın buzda
    kayması gibi.

**Doğrulama (izole `_proto13/`, gerçek dosyalardan türetilen kopyalar
üzerinde):**

  * **Regresyon:** `mu` her yerde `THRUST_CAP`'ten büyük tutulunca (buz
    yok) çıktı 12. tur ile BİREBİR aynı (adım listesi, düşme karesi,
    `hip_vx` -- hepsi karakter karakter eşleşti).
  * **65s kararlılık, rahatsızlık YOK:** periyodik buz yamaları (her
    400px'de bir) eklense bile sonuç baseline ile BİREBİR aynı (185 adım,
    NaN yok) -- çünkü sabit hızda yürürken istenen itki zaten sıfıra
    yakın, `F_MAX=0.15*1.5=0.225`'i hiç aşmıyor. **Mekanizma tam olarak
    olması gerektiği yerde atıl** (12. turdaki MAR v2 ile aynı desen:
    "atıl değil, sadece gerekmiyor").
  * **SLIP_GAIN taraması:** 1.0-1.5 arası temiz (max ardışık aynı-bacak
    adımı=1-2, hip_vx sınırlı sapma); 3.0'da kararsızlık başlıyor
    (hip_vx aralığı [-3.59, 10.89], max ardışık=3) -- 9./10./11./12.
    turların "daha agresif parametre = gizli bedel" deseniyle tutarlı,
    bu yüzden `SLIP_GAIN=1.0` seçildi.
  * **Tokezleme-büyüklüğü × buz taraması (en anlamlı bulgu):** aynı
    tokezleme darbesi, buzsuz zeminde ve buzlu zeminde karşılaştırıldı:

    | Darbe | Buzsuz (mu=1.2) | Buzlu (mu=0.15) |
    |---|---|---|
    | 5-15px | Hayatta kalır | Hayatta kalır (hip_vx sapması artıyor) |
    | 20px | Hayatta kalır | **DÜŞÜYOR** (frame 146) |
    | 25px | Hayatta kalır | **DÜŞÜYOR** (frame 128) |
    | 30px | Hayatta kalır | **DÜŞÜYOR** (frame 127) |

    Düzgün, monoton bir eşik etkisi -- NaN yok, ani/patolojik bir sıçrama
    yok. Buz, karakterin tokezleme toleransını yaklaşık yarıya indiriyor;
    bu tam olarak istenen fizik ("buzlu zeminde aynı tokezleme çok daha
    tehlikeli").

**Gerçek dosyaya uygulama:** `physics/active_gait.py`'ye `ActiveFootPlantingLeg.
apply_slip()` (tek satırlık, `planted[0]`'ı kaydırıyor) eklendi.
`demo/step14_active_biped.py`'ye `Terrain` import edildi, `GROUND_MU_DEFAULT
=1.2`/`ICE_ZONES`/`SLIP_GAIN=1.0` sabitleri ve itki-hesaplama bloğuna
sürtünme-sınırı + kayma mantığı eklendi (varsayılan demoda x=[250,450]
aralığında tek bir buz yaması var, karakterin tokezleme-sonrası bu yamaya
girdiği an görülüyor). 12s demo (buzsuz regresyon + buzlu), 65s kararlılık
(rahatsız edilmemiş + darbe+buz kombinasyonu) ve görsel QA gerçek dosyada
tekrarlandı, hepsi `_proto13/` sonuçlarıyla birebir eşleşti.

**Doğrulama:** prototip TAMAMEN `_proto13/` altında izole çalıştı, tur
sonunda `rm -rf _proto13` ile silindi -- `git status --short` bu turun
başında da sonunda da temiz.

## 13. tur eki: gercek bir surtunme motoruna donusum -- kullanicinin 3 mimari elestirisi kapatildi

**Kullanicinin tespiti:** 13. turda uygulanan "Kinetik Surtunme Siniri"
mekanizmasi mimari olarak temiz olsa da ("itki uretiminin step14'te, kayma
durumunun active_gait.py'de tutulmasi ... motorun modular yapisini
koruyan dogru bir sistem tasarimi"), fiziksel olarak gercek bir surtunme
modeli DEGIL, tek boyutlu bir "itki tiraslama" (thrust-clipping) hilesiydi.
Uc somut mimari acik tespit edildi:

1. **Dinamik agirlik aktarimi yok sayilmis:** surtunme sinirini sabit
   `THRUST_CAP`'e baglayarak bacaga binen GERCEK dikey yuk (heel-strike,
   agir inis) denklemden tamamen cikarilmisti -- ayak ister tuy gibi
   dokunsun ister tum govde agirligiyla cokup bassin, ayni itki degerinde
   kayiyordu.
2. **Statik/kinetik surtunme (Stribeck etkisi) eksik:** tek bir surtunme
   tavani vardi. Gercek fizikte statik katsayi (mu_s) daima kinetik
   katsayidan (mu_k) buyuktur -- kaymayi BASLATMAK zordur, ama basladiktan
   sonra tutunma aniden duser. Eski mekanizma, asan itkiyi ayni kare
   icinde "sizdirip" hemen tekrar `stance`'a kilitleniyordu -- gercek bir
   suruklenme (slip phase) yasanmiyordu.
3. **Cok yonlu (omnidirectional) kayma korlugu:** `apply_slip()` SADECE
   bacagin kendi urettigi ileri itkiye (1D) tepki veriyordu; disaridan
   gelen darbelere (yanal carpma, vs.) tamamen kordu.

**Anahtar cikarim (README'nin geri kalaninda tekrar eden "durust
basitlestirme" felsefesiyle ayni ruhta):** anchor-kalca cubugunun
(`build_body()`'deki ARM_LENGTH uzunlugunda, compliance=0.85 olan cubuk)
relaksasyon-SONRASI GERCEK boyu -- `body.points[hip] - body.points[anchor]`
-- `physics/verlet.py`'ye HICBIR degisiklik gerektirmeden zaten var olan,
fiziksel olarak gercek bir nicelik. Bu cubugun rest-length'ten (ARM_LENGTH)
sapmasi (`stretch_dev = |stretch_vec| - ARM_LENGTH`), itkiyi, yercekimi/
carpma kaynakli dikey yuklenmeyi VE disaridan gelen darbeleri (STUMBLE_KICK/
BIG_PUSH_KICK, ikisi de hip'in `prev_points`'ini degistirerek uygulaniyor)
TEK bir olcumde yakalar -- cunku hepsi ayni yoldan (hip'in konumunu/hizini
degistirerek) cubugun gercek gerilme/sikisma durumunu etkiler.

**Mekanizma (bkz. `demo/step14_active_biped.py` -- tam kod/gerekce orada,
uzun bir yorum blogu halinde):**

* **Agirlik aktarimi:** `stretch_dev`'in ISARETI fiziksel olarak anlamli --
  SIKISMA (negatif dev, hip anchor'a beklenenden yakin -- agir bir inis/
  heel-strike'ta olur) `f_n_effective`'i YUKSELTIYOR (daha fazla tutunma);
  GERILME (pozitif dev, kalkis/havalanma) DUSURUYOR:
  `load_factor = clamp(1 - LOAD_GAIN * stretch_dev, LOAD_FACTOR_MIN,
  LOAD_FACTOR_MAX)`, `f_n_effective = THRUST_CAP * load_factor`.
* **Cok yonlu stres:** `stress_vec = stretch_dev * (stretch_vec /
  |stretch_vec|)` -- stretch_dev'in cubugun GERCEK anlik dogrultusuna
  izdusumu (Verlet mesafe kisitlamasinin kendi duzeltme kuvvetiyle AYNI
  matematik). `total_stress = desired_thrust + STRESS_GAIN * stress_vec[0]`.
  Bu, `desired_thrust` zaten `THRUST_CAP`'e sikismis olsa bile (ki HER ZAMAN
  oyle -- bkz. asagidaki "yapisal kor nokta" bulgusu), buyuk bir dis
  darbenin normal zeminde (mu_static=1.2) bile kaymayi tetikleyebilmesini
  sagliyor.
* **Statik/kinetik (Stribeck):** `mu_kinetic = mu_static * KINETIC_RATIO`
  (KINETIC_RATIO=0.6, HER ZAMAN kucuk) iki AYRI `Terrain` orneginden
  (`terrain_static`, `terrain_kinetic`) okunuyor -- `Terrain` sinifinin
  KENDISI degistirilmedi (step8-13 ile geriye-uyumluluk icin, sadece iki
  kez ornekleniyor). Kayma tetiklenince (`is_slipping=True`, yeni bir alan
  -- `ActiveFootPlantingLeg.__init__`) ayak o kareden itibaren KALICI
  olarak kilitsiz kalir; `slip_velocity` kendi ivmelenen (kinetik esigi asan
  "excess" kadar) ve sonumlenen (`SLIP_DECAY=0.85`) dinamigiyle surer, ta ki
  hem HIZ (`SLIP_STOP_VEL` altina) hem de STRES (kinetik sinirin altina)
  ayni anda saglanana kadar. Her yeni ayak basisinda (`swing`->`stance`
  gecisi) `is_slipping`/`slip_velocity` sifirlaniyor -- her adim TAZE bir
  statik-surtunme sansiyla basliyor (gercek ayaklarin her adimda "yeniden
  tutunmasi" gibi).

**Onemli bir yapisal bulgu (izole testte kesfedildi, kullanicinin
elestirisini dogrudan dogruluyor):** `desired_thrust` HER ZAMAN
`THRUST_CAP=1.5`'e sikistirildigi ve normal zemin `mu_static=1.2>1.0`
oldugu icin, ESKI (13. tur) mekanizmasinda `abs(desired_thrust) > f_max`
kosulu normal zeminde MATEMATIKSEL OLARAK HICBIR ZAMAN gerceklesemezdi
(`f_max = 1.2*1.5 = 1.8 > 1.5`) -- yani buz disinda, hicbir disaridan gelen
darbe (ne kadar buyuk olursa olsun) eski modelde kaymayi tetikleyemezdi.
Bu, kullanicinin "3 numarali" elestirisinin (cok yonlu kayma korlugu)
somut, olcumle dogrulanan kanitidir.

**Dogrulama (izole `_proto13_eki/`, gercek dosyalardan turetilen paket-
yapida kopyalar uzerinde, hizli/video'suz bir "sweep_harness.py" ile):**

1. *Parametre taramasi (LOAD_GAIN x STRESS_GAIN, rahatsizlik YOK):*
   LOAD_GAIN<=0.02 VE STRESS_GAIN<=0.1 kombinasyonlarinda, hicbir kare
   sahte/gereksiz kayma tetiklenmiyor (`n_slip_events=0`) -- normal zeminin
   "hicbir zaman kaymaz" degismezi KORUNUYOR. STRESS_GAIN>=0.2'de bile
   gunluk yuruyus titresimi (dogal gait-cycle salinimi, dev araligi
   yaklasik [-3,+5]) bazen esigi asip 7-10 sahte kare uretebiliyor -- bu
   yuzden STRESS_GAIN=0.1, LOAD_GAIN=0.02 secildi (muhafazakar, kanitlanmis
   guvenli taraf).
2. *Agirlik aktarimi izole testi (AYNI 60px yatay darbe, farkli yukleme
   anlarinda):*

   | Yukleme durumu (dikey darbe) | Kayma tetiklenme karesi | Zirve slip_velocity |
   |---|---|---|
   | Agir inis (sikisma, +40px dikey) | 83 (GEC) | 3.59 (KUCUK) |
   | Notr (kontrol) | 81 | 5.96 |
   | Hafif/kalkis (gerilme, -40px dikey) | 61 (ERKEN) + 80 | 6.86 (BUYUK) |

   Monoton ve fiziksel olarak dogru sira: agir yuklu ayak daha GEC ve daha
   AZ kayiyor, hafif yuklu ayak daha ERKEN ve daha COK kayiyor -- agirlik
   aktarimi artik gercekten calisiyor.
3. *Cok yonlu (omnidirectional) acik kapatma testi (150px darbe, BUZ YOK,
   sadece normal zemin mu_static=1.2):* STRESS_GAIN=0.1 ile bile darbe
   53+ kare boyunca gercek bir kayma tepkisi uretiyor -- eski modelde bu
   YAPISAL OLARAK IMKANSIZDI (yukaridaki bulguya bkz.).
4. *Stribeck sureklilik testi:* standart 12s senaryoda (15px tokez + 150px
   itki) en uzun ardisik kayma serisi 70 kare (~2.3s) -- tek karelik bir
   "sizinti" degil, gercekten kalici, coklu-kareli bir suruklenme fazi.
5. *65s uzun-sure kararlilik, rahatsizlik YOK (regresyon):* 185 adim, kayma
   olayi=0, son_hip_y=148.66 -- ESKI (13. tur, eki-oncesi) modelle
   BIREBIR ayni (dijital olarak ozdes, cunku hicbir kayma tetiklenmedigi
   icin fizik tamamen ayni yolu izliyor).
6. *65s uzun-sure kararlilik, standart rahatsizliklarla:* NaN yok. Ilginc
   bir yan bulgu: yeni mekanizma dusme anini t=7.5s'den (eski model)
   t=11.57s'ye ERTELIYOR -- karakter, kontrollu/gercekci bir kayma-ve-
   toparlanma dinamigi sayesinde buyuk itkiyi ESKI modelden DAHA UZUN
   sureyle tolere ediyor (27 adim, eskisi 21 adim). Bu, mekanizmanin
   sadece daha karmasik degil, GERCEKTEN daha fiziksel/gercekci davrandigini
   gosteren emergent (istenmeden ortaya cikan) bir sonuc -- ayni 150px
   darbeyle sonunda yine dusuyor (150px hala hayatta kalma esiginin cok
   ustunde), sadece surecin kendisi artik daha inandirici.

**Durust sinir (bilerek kapsam disi birakildi, acikca belirtiliyor):** bu
motorda TUM dis darbeler (`STUMBLE_KICK_PX`, `BIG_PUSH_KICK_PX`) zaten
SADECE yatay (x ekseni) -- gercek bir dikey darbe modeli yok,
`Terrain.ground_y` sabit (egim/slope destegi henuz yok). O yuzden "cok
yonlu/2B" burada "yatay eksendeki TUM kaynaklarin (itki + dis darbe +
cubuk gerilimi) BIRLESIMI" anlamina geliyor, harfiyen "gercek dikey kayma"
degil -- gercek dikey yukleme zaten agirlik-aktarimi (`f_n_effective`)
yoluyla DOLAYLI olarak modelleniyor, ki bu fizikte de DOGRU yer: yercekimi
kaymaya degil, tutunma TAVANINA etki eder.

**Gercek dosyaya uygulama:** `physics/active_gait.py`'ye
`ActiveFootPlantingLeg.__init__`'e `is_slipping`/`slip_velocity` alanlari
eklendi (`apply_slip()` DEGISMEDI -- hala tek satirlik bir `planted[0]`
mutator'u). `demo/step14_active_biped.py`'de: `GROUND_MU_DEFAULT` ->
`GROUND_MU_STATIC` + `KINETIC_RATIO`/`SLIP_ACCEL_GAIN`/`SLIP_DECAY`/
`SLIP_STOP_VEL`/`LOAD_GAIN`/`LOAD_FACTOR_MIN`/`LOAD_FACTOR_MAX`/
`STRESS_GAIN` sabitleri eklendi; tek `terrain` -> `terrain_static` +
`terrain_kinetic`; itki-hesaplama bloğu tamamen yeniden yazildi (agirlik
aktarimi + cok yonlu stres + Stribeck kalici kayma durumu); her yeni ayak
basisinda kayma durumu sifirlaniyor. 12s demo, 65s kararlilik (rahatsiz +
rahatsiz-edilmemis) ve gorsel QA gercek dosyada tekrarlandi, hepsi
`_proto13_eki/` sonuclariyla birebir eslesti (rahatsiz-edilmemis 65s:
185 adim, son_hip_y=148.66 -- eski modelle dijital olarak ozdes).

**Doğrulama:** prototip TAMAMEN `_proto13_eki/` altinda izole calisti
(gercek dosyalardan turetilen, `physics`/`demo` paket yapisinda kopyalar +
bagimsiz bir `sweep_harness.py`), tur sonunda `rm -rf _proto13_eki` ile
silindi -- `git status --short` bu turun basinda da sonunda da temiz.

## 13. tur eki 2: "buz pateni safsatasi" ve "muz kabugu cokusu" -- anatomik geri bildirim eklendi

**Kullanicinin tespiti (13. tur eki'nin -- gercek surtunme motoru -- SONRASI,
yeni bir elestiri turu):** matematik (surtunme siniri, agirlik aktarimi,
Stribeck) dogru cozulmus olsa da, kayma mekanizmasinin ANATOMIK/gorsel
tepkisi hala eksikti -- 3 nokta:

1. **"Ice skate" safsatasi:** bir bacagin tek basina 70+ kare (~2.3s)
   boyunca kaymasi anatomik olarak imkansiz -- gercekte bacak-govde acisi
   hizla acilir, ya splits pozisyonuna girip yirtilir ya da sistem acil
   durum adimi atmak ZORUNDA kalir. Soru: kayan ayak kalcayi sadece
   suruklüyor mu, yoksa bir "duşme tehlikesi" mekanizmasi bu acisal
   acilmayi fark edip acil duruma geciriyor mu?
2. **"Muz kabugu" cokusu eksikligi:** ayak kaydiginda kalca (kutle merkezi)
   dikey eksende cokmeli. Eger anchor-kalca cubugu (compliance=0.85) ayak
   kayarken bile kalcayi sabit bir Y yuksekliginde tutmaya devam ediyorsa,
   karakter yeri itmiyor, "kukla ipiyle" havada tutuluyor demektir.
3. **Nokta-temas (point-contact) limiti:** ayak hala TEK bir koordinat
   noktasi uzerinden temas ediyor -- gercek bir kayan insanin topugu yeri
   kazir (friction spike) veya ayak bilegi bukulur; tek nokta tork
   uretemedigi icin kayma dinamigi hep "odunsu"/sabit gorunecektir.

**Tanı (kod-temelli, izole olcumle dogrulandi):**

* `demo/step14_active_biped.py` -- yani `step14` mimarisinin TAMAMI --
  `physics/balance.py`'nin `FallRiskMonitor`'unu HICBIR ZAMAN kullanmadi
  (o sadece eski `step12_balance.py`/`step13_full_integration_test.py`'de
  var, 8. turda `step14`'e capture-point tabanli TAMAMEN FARKLI bir denge
  modeline gecildiginde tasinmadi). Yani #1'in cevabi NET: HAYIR, kayan
  bacak HICBIR seyi tetiklemiyordu -- sadece kalcayi suruklüyordu.
  Izole olcum: buyuk-itki dusme senaryosunda bacak acisi (dikeyden)
  -74°'ye, hatta baska bir framede -77.6°'ye ulasti -- gercek bir insan
  hip abdüksiyon/ekstansiyon siniri ~30-45° civarindadir, yani bu
  ANATOMIK OLARAK SAcMA bir poz.
* **Muz kabugu**, HAFIF/hayatta-kalan bir kayma olayinda (15px tokezleme,
  frame 104-139) da olculdu: hip_y SADECE ~143-173px araliginda kaldi --
  neredeyse tamamen NORMAL yuruyus dongusunun kendi salinimi kadar,
  `is_slipping`'in hicbir olculebilir ek etkisi YOK. Kullanicinin
  tarifiyle birebir ortusuyor: "kukla ipi" dogrulandi.
* **Nokta-temas limiti** kullanicinin KENDISI tarafindan de doğru
  tanimlanmis: bu, ayagi TEK noktadan bir kapsule (Heel-to-Toe Roll)
  cevirmeyi gerektiren, roadmap'te zaten (2 numarali madde) bekleyen
  BUYUK bir mimari degisiklik -- bu turun kapsamina alinmadi (asagida
  "Durust sinir" bolumune bkz).

**Cozum (1 & 2 icin, izole `_proto13_eki2/`):**

* **Ice-skate-limit -> acil kurtarma adimi:** `MAX_SLIP_LEG_ANGLE_DEG=45°`
  -- kayan bacak (`is_slipping=True`) dikeyden 45°'yi asarsa, 7. turdan
  MIRAS alinan (ama `step14`'te hic cagirilmayan) `trigger_emergency_
  step()` HEMEN tetiklenir, ayni capture-point hedefine (xcp) yonlendirir
  -- normal esik beklenmeden. Tam bir `FallRiskMonitor`/support-polygon
  entegrasyonu DEGIL (bu, step14'un TAMAMEN farkli tek-anchor mimarisine
  gore yeniden tasarlanmasi gereken, cok daha buyuk ayri bir is) --
  SADECE bu ozel "kayan bacak splits'e giriyor" durumuna dar kapsamli,
  dogrudan bir guvenlik supabı.
* **Muz kabugu cokusu:** anchor-kalca cubugunun compliance'i ARTIK SABIT
  DEGIL -- `is_slipping` sirasinda, kayma hizina ORANTILI olarak
  gevsetiliyor (`ANCHOR_HIP_COMPLIANCE_BASE=0.85` -> `SLIP_COMPLIANCE_
  MAX=0.90`, `SLIP_COMPLIANCE_SAT_VEL=2.0`'da doyar). `Terrain` gibi,
  `VerletSystem.sticks` de zaten mutable bir liste -- `physics/verlet.py`
  HICBIR degisiklik gerektirmeden, `body.sticks[idx["anchor_hip_stick"]]`
  her karede yeni bir compliance degeriyle degistiriliyor.

**Onemli bir durust sinir/bulgu (sweep ile kesfedildi):** `SLIP_COMPLIANCE_
MAX` 0.92'yi asinca sistem "whip-crack" (kirbac sapi) kararsizligina
giriyor -- cubuk o kadar gevsiyor ki hiz gecici olarak kontrolsuzce
birikiyor, sonra compliance normale donunce bu enerji ani bir savurmaya
donusuyor (gozlemlenen max_hip_vx: 4054px/kare -- 14.7'lik normal tavanin
275 kati). Bu yuzden 0.90 GUVENLI TAVAN olarak secildi -- bunun anlami,
ulasilabilen "cokme derinligi" mutevazi (peak hip_y'de ~2-3px ekstra
sarkma, agir tokezleme sirasinda) -- gorsel olarak subtil ama olcumle
DOGRULANMIS ve KARARLI. Daha dramatik bir cokus icin cubuk-tabanli
yaklasimin OTESINE gecmek (ör. gercek bir "diz bukulmesi" -- bacak
segmentlerinin kendisini kisaltmasi) gerekir; bu, dogal olarak #3'un
(Heel-to-Toe Roll / Segment Foot) kapsamina giriyor.

**En carpici bulgu -- iki eki'nin birlikte anlami:** #1 (omnidirectional/
agirlik aktarimi, 13. tur eki) TEK BASINA, buyuk-itki dusme anini
t=7.5s'den (orijinal 13. tur) t=11.57s'ye ERTELEMISTI -- bu, o zaman
"gercekci kayma-toparlanma" olarak yorumlanmisti. Ama muz-kabugu-cokusu
EKLENINCE (bu turda), dusme anı t=7.77s'ye (frame 233) GERI DUSTU --
neredeyse ORIJINAL 13. turun degerine (t=7.5s, frame 225) esit. Yani
onceki "347 kareye kadar hayatta kalma" bulgusunun BUYUK KISMI, aslinda
tam da kullanicinin tespit ettigi hatanin (karakterin fiziksel olarak
imkansiz bir sekilde "buz pateni" yaparak cokmesi gereken anda cokmemesi)
BIR SONUCUYMUS -- gercek collapse eklenince bu yapay hayatta-kalma-suresi
KAYBOLDU, dusme zamanlamasi FIZIKSEL OLARAK DAHA DOGRU/DAHA MUHAFAZAKAR
bir degere geri donuyor. Bu, motorun onceki turda tespit edilemeyen bir
"gizli tavizi" bu turda ortaya cikardigini gosteriyor.

**Dogrulama:** 65s rahatsizsiz regresyon BIREBIR ozdes (185 adim,
son_hip_y=148.66, 0 kayma/0 acil-adim -- iki yeni mekanizma da HERHANGI
bir rahatsizlik olmadan tamamen atil). Standart 12s/65s senaryosunda
(15px+150px): 5 acil-kurtarma-adimi tetiklendi (aci: 64.9° ila -89.4°
arasi), dusme frame 233 (eskiden 347), NaN yok. Gorsel QA: acil adim
tetiklenmeden hemen once bacak neredeyse yatay (65° civari), sonrasinda
govde gercekci bir sekilde cokuyor -- artik uzun sureli "splits" pozu
YOK.

**Durust sinir (bilerek kapsam disi birakildi):** nokta-temas limiti (#3,
Heel-to-Toe Roll / Segment Foot) bu turda ELE ALINMADI -- bu, ayagi tek
bir koordinat noktasindan bir segmente/kapsule cevirmeyi, topuk-carpma /
taban-yukleme / parmak-ucu-itis fazlarini modellemeyi gerektiren, roadmap'te
zaten 2 numarali madde olarak bekleyen, cok daha buyuk bir mimari degisiklik.
Bu tur SADECE #1 (ice-skate-limit) ve #2'yi (muz kabugu) kapatti.

**Gercek dosyaya uygulama:** SADECE `demo/step14_active_biped.py`
degisti -- `physics/active_gait.py`/`physics/gait.py`/`physics/balance.py`
HICBIR sekilde degistirilmedi (7. turdan miras `trigger_emergency_step()`
oldugu gibi yeniden kullanildi, `VerletSystem.sticks` zaten mutable bir
liste oldugu icin `physics/verlet.py`'ye de dokunulmadi). `build_body()`'ye
`idx["anchor_hip_stick"]` (index) eklendi; yeni sabitler + acil-adim/
dinamik-compliance mantigi itki bloguna eklendi.

**Doğrulama:** prototip TAMAMEN `_proto13_eki2/` altinda izole calisti,
tur sonunda `rm -rf _proto13_eki2` ile silindi -- `git status --short`
bu turun basinda da sonunda da temiz.

## 13. tur eki 3: "45 derece sihirli sayi" ve "whip-crack yalani" -- var olan mekanizma dogru baglandi, hile kaldirildi

Kullanicinin 13. tur eki 2'ye getirdigi 2 yeni elestiri (3.'su -- nokta-temas/Segment Foot -- ayri, daha buyuk bir mimari is olarak `step15_segment_foot.py`'a birakildi, bkz. asagisi):

1. **"45 derece sihirli sayi bir geriye gidistir"**: kayan bacagi sabit bir aci esigiyle (`MAX_SLIP_LEG_ANGLE_DEG`) acil adima zorlamak, kutle merkezinin (COM) destek poligonuna GORE nerede oldugunu hic bilmiyordu -- COM kayan bacakla ayni yondeyse 45 derece guvenli olabilir, tersiyse 20 derece bile olumcul olabilir. Bu, projenin uzun zamandir kacindigi "constraint soup"a (sabit sihirli sayilarla yamali kisitlama yigini) geri donustu.
2. **"Whip-crack tesadüf degil, kotu fizigin ciglaligidir"**: `SLIP_COMPLIANCE_MAX`/`SLIP_COMPLIANCE_SAT_VEL` ile anchor-kalca cubugunun compliance'ini kayma hizina bagli dinamik degistirmek, bir yay sabitini yuk altinda degistirmekti -- gercek bir "ayagin artik dikey Normal Kuvvet uretememesi" fizigi degil, matematigi goruntu icin kandirmak.

### Tani

Kod incelemesi carpici bir şey ortaya cikardi: kullanicinin talep ettigi tam mekanizma -- kutle merkezi vs. GERCEK destek araligi (ayagin fiziksel genisligi dahil), histerezisli tehlike tespiti -- `physics/balance.py`'de **zaten mevcuttu** (`FallRiskMonitor`, `support_interval`, `outside_interval_error`) ve `demo/step12_balance.py`/`step13_full_integration_test.py`'de zaten kullaniliyor, dogrulanmisti. Bu, `demo/step14_active_biped.py`'ye (ve dolayisiyla 13. tur/13. tur eki/13. tur eki 2'ye) hic baglanmamisti -- yeni bir sey icat etmek yerine, mevcut ve zaten test edilmis makineyi doğru yere kablolamak gerekiyordu.

### Cozum

- **(1) icin**: acil adim tetikleyicisi artik `is_slipping`'e VEYA sabit bir aciya degil, `com_x`'in `support_interval([stance_leg.planted[0]], foot_half_len=FOOT_HALF_LEN, ...)`'in GERCEKTEN disina cikip cikmadigina (`outside_interval_error`, `FallRiskMonitor` histerezisiyle) bakiyor. Bilerek `is_slipping`'den BAGIMSIZ: bir darbe, ayak hic kaymadan bile COM'u destek disina cikarabilir.
- **(2) icin**: `ANCHOR_HIP_COMPLIANCE_BASE=0.85` artik HICBIR ZAMAN degismiyor -- dinamik compliance TAMAMEN KALDIRILDI. `build_body()`'deki `anchor_hip_stick` indeksi de (artik gereksiz oldugu icin) kaldirildi.

### Dogrulama (izole `_proto_eki3/demo/ablation.py`, gercek repo dosyalarindan TURETILEN kopyalar)

5 konfigurasyonluk bir ablation matrisi calistirildi (aci-kriteri x compliance-modu). Standart senaryo: 15px tokez (t=3s) + 150px buyuk itki (t=7s), 12s.

| Konfigurasyon | Dustu mu? | Dusme karesi | Buyuk-itki-sonrasi max\|bacak acisi\| |
|---|---|---|---|
| Eski (aci=45°, compliance=SABIT 0.85) | Evet | 230 | 85.9° |
| **Kommitli hali (aci=45°, compliance=DINAMIK)** | **Evet** | **233** | **89.4°** |
| Orijinal (13.tur eki, aci-kriteri YOK) | Evet | 347 | 77.6° |
| **YENI (COM-kriteri, compliance=SABIT)** | **HAYIR** | **--** | **53.8°** |
| COM-kriteri + compliance=DINAMIK (capraz-kontrol) | Evet | 225 | 84.6° |

**En onemli satir, son ikisinin karsilastirmasi**: aynen ayni (dogru) COM-kriterini kullanirken, dinamik compliance EKLEMEK sonucu DAHA KOTU yapiyor (hic dusmeyen bir senaryoyu 225. karede dusmeye ceviriyor). Bu, kullanicinin "compliance hilesi termodinamik bir yalan" elestirisinin DOGRUDAN sayisal dogrulamasi -- dinamik compliance'i kaldirmak sadece daha durust degil, OLCULEBILIR sekilde daha iyi.

**Siddet taramasi** (YENI konfigurasyon, COM-kriteri + sabit compliance) gercek bir kirilma noktasi buldu -- "hic dusmuyor" degil, cok daha yuksek VE karakterize edilmis bir esik:

| Tek darbe (px) | 150 | 200-700 | 1000 | 1500 | 2500 |
|---|---|---|---|---|---|
| Dustu mu? | Hayir | Hayir | **Evet** (frame 235) | Evet (215) | Evet (214) |

Ardisik cift darbe (300px @ t=7s + X @ t=8s): 150px ve 300px ikinci darbeler hayatta kaliyor, **500px ikinci darbe dusuyor** (frame 252).

**20s rahatsiz-edilmemis regresyon**: 0 acil-adim tetiklemesi, ort. `hip_vx`=1.898 (hedef 2.0) -- `FALL_RISK_ENTER_PX`/`EXIT_PX` (step12/13'ten AYNEN alinan 45.0/20.0) normal yuruyuste hicbir yanlis-pozitif uretmiyor, retune gerekmedi.

### Durust sinir

`_proto_eki3/` prototipi bu turda da (onceki tum prototipler gibi) silindi -- sadece gercek dosyalara (`demo/step14_active_biped.py`) uygulanan degisiklik kaliciydi. FALL_RISK_ENTER_PX/EXIT_PX degerleri step12/13'ten dogrudan devralindi (step14'un kendi geometrisi/COM vekili icin YENIDEN ayarlanmadi) -- 20s regresyon ve siddet taramasinda sorun cikarmadilar, ama ozel bir tarama ile ince ayar YAPILMADI. Nokta-temas/Segment Foot elestirisi (3.) bu turda da ELE ALINMADI -- kullanicinin kendi talebi uzerine ayri bir izole laboratuvar (`step15_segment_foot.py`) olarak baslatildi (bkz. asagisi).


## Adım 15 — Segment Foot (Heel-to-Toe Roll): checkpoint 1

Kullanıcının 13. tur eki 2'ye getirdiği 3. eleştiri ("nokta temasına
çarptın") üzerine açılan yeni, ayrı bir laboratuvar: `demo/step15_segment_
foot.py`. **Bu, tam bir yürüyüş entegrasyonu DEĞİL** — kalça kasıtlı
olarak kinematik/pinned ve yavaş (quasi-statik) bir süpürmeyle hareket
ettiriliyor, `step14`'ün stance/swing/kayma mekanikleri burada yok.

**Amaç**: `physics/verlet.py` veya `physics/collision.py`'ye TEK BİR
SATIR bile dokunmadan, tek bir "anchor" noktası yerine rijit bir
ankle→heel + ankle→toe + heel↔toe üçgeninden oluşan iki-noktalı bir
ayağın, gerçek bir topuk-parmak ucu yuvarlanmasını (heel-to-toe roll)
HİÇBİR "kaldır" kuralı yazmadan, sadece kısıt+çarpışma+yerçekiminden
kendiliğinden (emergent) üretip üretemeyeceğini ölçmek.

**Bulgu**: kalça ayağın tam üstünden (düz taban, açı≈0°) parmak ucunun
ilerisine doğru süpürülünce, `toe_y` zeminde kilitli kalırken `heel_y`
DÜZENLİ/MONOTONİK olarak yükseliyor (zeminden kalkıyor) ve ayak açısı
0°'den ~44°'ye kadar sürekli artıyor — gerçek bir topuk-kalkışı/parmak-
ucu-itişi yuvarlanması, hiçbir ek kural olmadan. Bu, kullanıcının "ayak
bileği torku ve yüzey alanı olmadan sürtünme fiziğini daha fazla ileri
götüremezsin" tespitinin doğru çözüm yönünü (nokta yerine rijit iki-nokta
kapsül) sayısal olarak doğruluyor.

**Dürüst sınır / sıradaki adımlar** (bu turda YAPILMADI): bu iki-nokta
ayağın `step14`'ün stance/swing state machine'ine entegrasyonu (`planted`
artık tek bir x değil, heel_x/toe_x çifti olmalı); Stribeck kayma
modelinin heel/toe'ya AYRI uygulanması (gerçek bir "topuk kazıma"
friction-spike modeli için); FABRIK/IK zincirinin ayak-ucu hedefi olarak
bu segmenti kullanacak şekilde güncellenmesi; ayak bileği açısının kendi
biyomekanik sınırı (bu checkpoint'te bilerek sınırsız bırakıldı — hip_x
anatomik olarak anlamlı aralığın çok ötesine (foot uzunluğunun 4+ katı)
taşındığında görülen tuhaf kısmi geri-sarma bir sınır-dışı artefakttır,
gerçek yürüyüşte bu kadar ileri gidilmeden çok önce ayak zaten swing'e
geçerdi). Segment Foot'un tam entegrasyonu ayrı, gelecekteki bir/birkaç
tur olarak planlanıyor.


## Adım 16 — Segment Foot dinamik doğrulama laboratuvarı (checkpoint 2): iki bulgu doğrulandı, bir sorun bilerek açık bırakıldı

Kullanıcı, Adım 15 checkpoint 1'in (kinematik/pinned kalça) başarısını
onayladıktan sonra 3 somut entegrasyon darboğazı sordu (IK uç-efektör
çatışması, state machine parçalanması, asimetrik sürtünme dağılımı) ve
bunlara mimari olarak cevap verilip bir ilk "checkpoint 2" prototipiyle
(serbest kalça + per-node yük sensörü) 3. maddenin fizibilitesi gösterildi.
Kullanıcı bunu doğrudan `active_gait.py` entegrasyonuna taşımak yerine
şunu istedi: **önce checkpoint 2'yi temizle, parametre taramalarını
tamamla, bağımsız bir laboratuvar modülü olarak commit'le** — çünkü ilk
denemedeki `FOOT_STICK_COMPLIANCE=0.3`/`LOAD_GAIN=0.05` değerleri
sezgisel seçilmişti; taranmadan entegre edilirse gelecekteki bir çöküşün
gait-geçiş mantığından mı yoksa bu ham parametrelerdeki gizli bir
rezonanstan mı kaynaklandığı asla izole edilemezdi.

Bu tur, `demo/step16_segment_foot_lab.py` içinde tam olarak bunu yaptı:
konsol-raporu bir ölçüm laboratuvarı (video üretmiyor), 2 tarama
TAMAMLANDI + 1 sorun bilerek AÇIK bırakıldı.

### BULGU 1 — Esneklik (compliance) taraması: 0.3 gerçekten güvenli

İki BAĞIMSIZ yöntemle çapraz doğrulandı:

**(a) İzole ring-down testi** (yerçekimi VE iç sürtünme KAPALI — en az
sönümlü, worst-case senaryo; `ankle` sabit, `heel`'e tek 15px darbe;
büyük kalça-sarkacı kirliliği olmadan SADECE `ankle-heel` çubuğunun
kendi salınım/sönümleme davranışı ölçülüyor):

| compliance | ilk_dev | max\|dev\| | 120.kare_dev | sönüm_oranı |
|---|---|---|---|---|
| 0.00 | 0.069 | 0.069 | -0.0000 | 0.0000 |
| 0.30 | 0.223 | 0.223 | 0.0010 | 0.0049 |
| 0.60 | 0.457 | 0.457 | 0.0067 | 0.0159 |
| 0.70 | 0.364 | 0.692 | 0.0126 | 0.0196 |
| 0.80 | -0.138 | 1.009 | 0.0255 | 0.0273 ← işaret tersine dönüyor |
| 0.90 | -1.577 | 2.265 | 0.0677 | 0.0323 |
| 0.95 | -2.944 | 4.535 | 0.1585 | 0.0378 |
| 0.99 | -4.514 | 12.412 | 0.5628 | 0.0922 |

0.7'ye kadar davranış sağlıklı (küçük, hızla sönen sapmalar). 0.8'den
itibaren ilk-kare sapması İŞARET DEĞİŞTİRİYOR (beklenen gerilme yerine
sıkışma görünüyor) ve genlik/sönüm oranı hızla büyüyor — kullanıcının
"sünger etkisi" tam olarak bu davranış.

**(b) Çapraz-kontrol** (aynı tarama, GERÇEK sistemde: yerçekimi+sürtünme+
itki AÇIK, tek seferlik 40px darbe): `heel_dev_max` 0.44 (c=0.1) → 0.83
(c=0.3) → 14.35 (c=0.9) — (a)'nın izole bulgusuyla AYNI eğilim, AYNI
sıra: 0.3 hâlâ küçük/sağlıklı tarafta, büyüme 0.5–0.7 civarında
hızlanıyor.

**Sonuç**: `FOOT_STICK_COMPLIANCE=0.3`, hip çubuğundaki 0.85'in ayak
çubukları için doğrudan karşılığı DEĞİL (çok daha kısa çubuk, farklı
kütle oranı) — ama kendi ölçeği içinde, kendi sistemli taramasıyla
doğrulanmış, güvenli bir seçim. ~0.7 civarı, gelecekte "daha esnek/hassas
ayak" denenirse yaklaşılmaması gereken sınır olarak not edildi.

### BULGU 2 — Yük-kazancı (LOAD_GAIN) taraması: 0.3'e kadar sakin, 0.5'te hafif çırpınma

Sabit yürüyüş itkisi altında (compliance=0.3 sabit, 200 kare), `LOAD_GAIN`
0.0'dan 0.3'e kadar heel/toe kayma-durumu hiç "toggle" (açıl/kapan)
yapmıyor (0 → 0 → ... → 0). 0.5'te ilk kez hafif çırpınma beliriyor
(heel: 2, toe: 2 toggle). `LOAD_GAIN=0.05` (13. tur eki'nden devralınan
mertebe) bu tarama içinde rahat, geniş bir güvenlik payıyla oturuyor.

### Dürüst sınır — açık/çözülmemiş sorun: per-node yatay stres formülü

Kullanıcının istediği "pogo testi"ni (ikinci/swing bacak OLMADAN, tek
ayağın en ağır darbe altında heel/toe sürtünmelerinin ayrışıp
ayrışmadığı) kurarken İKİ ayrı, gerçek sorun bulundu — **ikisi de
çözülmedi, bilerek yarım bırakıldı**:

1. Bu izole "tek ayak" rigi gövde/karşı-denge/adım-atma içermiyor —
   serbest kalça + rijit `ARM_LENGTH` çubuğu, matematiksel olarak ters
   bir sarkaç (inverted pendulum). Aktif bir düzeltme mekanizması
   (step14'ün torso-lean clamp'i ya da bir sonraki adımı atma refleksi)
   olmadan HERHANGİ bir yatay darbe er ya da geç kalçayı devirir — hatta
   sıfır darbeyle bile, sürekli yürüyüş itkisi altında (kontrol testiyle
   doğrulandı). Bu, ayak modelinin değil, rigin gövdesiz olmasının doğal
   sonucu. Yani "tam hayatta kalma" bu checkpoint'te ANLAMLI bir metrik
   DEĞİL; analiz sadece darbe-hemen-sonrası (ilk ~25 kare) pencereye
   odaklanabilir.
2. O kısa pencerede bile, per-node yatay "stres" formülü (step14'ün
   `stress_vec` tekniğinin `ankle-heel`/`ankle-toe`'ya doğrudan kopyası)
   güvenli bir çalışma noktası BULAMADI. `NODE_STRESS_GAIN` taraması:
   gain=1..8 → 20px darbede bile hiçbir kayma tetiklenmiyor; gain=10 →
   darbe YOKKEN bile (normal yürüyüş) toe 18 kare kayıyor; gain=15..30 →
   normal yürüyüşte her iki nod da sahte (spurious) kayıyor. Yani "büyük
   darbede tetiklenir, normal yürüyüşte tetiklenmez" diye bir güvenli
   aralık bulunamadı — step14'ün uzun anchor-kalça çubuğu için taranmış
   formülü, çok daha kısa ayak çubuklarına naif biçimde taşınamıyor.
   Olası neden (test EDİLMEDİ, sadece hipotez): yanlış nicelik ölçülüyor
   olabilir — stick-stretch yerine `collide_ground()`'un o karede
   uyguladığı gerçek dikey düzeltme miktarı (bir çarpma/impuls vekili)
   daha doğru bir sinyal olabilir.

**Bu yüzden**: pogo testi / asimetrik-kayma-ayrışma iddiası bu turda
DOĞRULANAMADI, commit'e dahil EDİLMEDİ. Sadece BULGU 1 ve BULGU 2
(sağlam ve tekrarlanabilir) commit'lendi. Per-node stres formülünün doğru
fiziksel niceliğini bulmak, tam `active_gait.py` entegrasyonundan ÖNCE
çözülmesi gereken ayrı bir sonraki adım.


## Adım 16 eki (checkpoint 3) — collide_ground() tabanlı gerçek Normal Kuvvet vekili: hipotez izole doğrulandı, daha derin bir sorun bulundu

Adım 16'nın açık bıraktığı soruna kullanıcı somut bir hipotezle geri
döndü: per-node yatay stres formülünün `dev` (çubuk-gerilmesi) yerine,
`collide_ground()`'un o karede uyguladığı **gerçek dikey düzeltme
miktarını** (ΔY — penetrasyon derinliği) Normal Kuvvet vekili olarak
kullanması gerektiğini savundu; kısa/sert `ankle-heel`/`ankle-toe`
çubuklarında `dev`'in mikroskobik Verlet titreşimleriyle kirlendiğini,
ΔY'nin ise iteratif fizik motorlarında Normal Kuvvet'in doğrudan
matematiksel karşılığı olduğunu belirtti.

**Bulgu 3 (hipotez İZOLE olarak doğrulandı, ama pogo testinin kendisi
yanlış senaryo çıktı):** orijinal pogo/push testi izole edildiğinde,
yatay bir darbenin bu rijit tek-bacak geometrisinde ayağı zemine daha
çok bastırmadığı, tam tersine zeminden kaldırdığı (ters-sarkaç
devrilmesi) ortaya çıktı — push arttıkça ΔY sıfıra düşüyor, hiçbir zaman
büyümüyor. Yani pogo testi bir devrilme testi, bir darbe/yük testi
değil. Bunun yerine gerçek bir **heel-strike (topuk-önce iniş)**
senaryosu kuruldu (checkpoint 1'in doğruladığı topuk-önce eğimle, küçük
bir yükseklikten ileri hızla düşürme). İzole ölçümde (yerçekimi/darbe
açık, yürüyüş itkisi KAPALI) ΔY ve ona eşlik eden bir "shear" sinyali
(ΔX — aynı Verlet-native kökten, adım-öncesi/sonrası konum farkından
türetilen yatay eşlenik) gerçekten çok daha temiz çıktı:

| Sinyal | darbe_max | yerleşik_max | oran |
|---|---|---|---|
| heel_pen (ΔY) | 1.0053 | 0.2307 | 4.4x |
| toe_pen (ΔY) | 0.7053 | 0.2466 | 2.9x |
| heel_shear (ΔX) | 0.3562 | 0.0130 | 27.4x |
| toe_shear (ΔX) | 0.3393 | 0.0130 | 26.1x |

Eski `dev` sinyalinin hiçbir gain'de ulaşamadığı bir ayrım — kullanıcının
hipotezi izole ölçümde tam isabetli.

**Ama daha derin bir sorun (ACIK SORUN 2):** bu iyileştirilmiş ΔY/shear
çifti, gerçek yürüyüş itkisiyle (`thrust_gain=1.0`) birleştirilip
`K_NORMAL` (ΔY'yi friksiyon-limitine çeviren yeni bir kazanç) tarandığında
AYNI temel ayrım sorunu farklı bir yüzeyde geri geldi:

| K_NORMAL | yürüyüş_heel | yürüyüş_toe | heel-strike_heel | heel-strike_toe |
|---|---|---|---|---|
| 0.2 | 12 | 31 | 0 | 26 |
| 1.0 | 3 | 22 | 0 | 24 |
| 2.0 | 9 | 2 | 0 | 1 |
| 4.0 | 11 | 1 | 0 | 0 |
| 20.0 | 16 | 0 | 0 | 0 |

Küçük `K_NORMAL`'da ikisi de tetikleniyor (ayrım yok); büyük `K_NORMAL`'da
heel-strike ARTIK TETİKLENMİYOR (hassasiyet kayboluyor) AMA normal
yürüyüşte topuğun ağırlığı öne doğru kayarken (checkpoint 2'nin zaten
bulduğu "sürekli itki ağırlığı öne taşır" davranışı) penetrasyon sıfıra
yaklaştıkça friksiyon limiti de sıfıra çöküyor ve kalan küçük sayısal
gürültü bu sıfıra-yakın eşiği trivial olarak aşıp sahte kayma
işaretliyor — istenenin tam tersi (K_NORMAL=4.0: yürüyüşte heel_slip=11,
DARBE YOKKEN; aynı K_NORMAL'da gerçek heel-strike'ta heel_slip=0).

**Sonuç (güncellenmiş):** kullanıcının ΔY/shear hipotezi izole ölçümde
DOĞRU çıktı (`dev`'den kat kat daha iyi SNR) — ama bu tek başına, gerçek
yürüyüş dinamiği ile gerçek darbe arasındaki ayrımı çözmüyor. Kök neden
muhtemelen kullanıcının kendi orijinal 2. maddesinde (mesaj: IK/state-
machine eleştirisi) zaten önerilen, henüz UYGULANMAMIŞ bir ön-koşul:
heel-strike/flat-foot/toe-off FAZ durum makinesi olmadan, "normal" ile
"anormal" yerel sinyali mutlak bir eşikle ayırmak yapısal olarak mümkün
görünmüyor — çünkü "normal"in kendisi (ağırlık aktarımının doğal roll'u
sırasında) zaten penetrasyonu sıfıra yaklaştırıp tam da darbe-sonrası
rejimle çakışan bir bölgeye giriyor. Faz bilgisi olmadan, mutlak eşik
tabanlı hiçbir formül (ister `dev`, ister ΔY/shear) bu ikisini güvenilir
şekilde ayıramıyor. Bu, tam `active_gait.py` entegrasyonundan ÖNCE —
hatta per-node stres formülünden bile ÖNCE — çözülmesi gereken, daha
temel bir ön-koşul olarak yeniden çerçeveleniyor.

Bu ek de (checkpoint 2'nin geri kalanı gibi) commit'e yeni bir slip-
tetikleme mekanizması olarak dahil edilmedi — `run_heelstrike()` izole
test fonksiyonu olarak `demo/step16_segment_foot_lab.py`'ye eklendi,
bulgular dosyanın kendi docstring'inde ve `main()`'in konsol çıktısında
belgeleniyor.


## Adım 16 eki (checkpoint 4) — Üç fazlı (Heel-Strike / Flat-Foot / Toe-Off) durum makinesi: faz maskelemesi, K_NORMAL eşik sorununu çözdü

**Düzeltme notu:** checkpoint 3'teki `run_heelstrike()` fonksiyonunda bir
heel/toe **açı-atama hatası** bulundu — "heel" adı verilen nokta aslında
GEÇ değen, "toe" adı verilen nokta ERKEN değen noktaydı (`base_ang ±
theta` terimleri ters atanmıştı). Bu turda düzeltildi. Aynı izole senaryo
tekrar koşulduğunda nicel sonuç aynı mertebede kaldı, sadece artık doğru
fiziksel noktaya doğru isim karşılık geliyor:

| Sinyal | darbe_max | oran (eski, ters etiketle) | oran (düzeltilmiş) |
|---|---|---|---|
| heel_pen (ΔY) | 1.1741 | 4.4x | **5.2x** |
| toe_pen (ΔY) | 0.8484 | 2.9x | **3.4x** |
| heel_shear (ΔX) | 0.9420 | 27.4x | **41.8x** |
| toe_shear (ΔX) | 0.5850 | 26.1x | **26.0x** |

ACIK SORUN 2 (K_NORMAL için güvenli aralık yok) tablosu da tekrar
koşuldu — sonuç aynı: hiçbir `K_NORMAL` "yürüyüş=0,0 VE darbe>0"
satırını birlikte vermiyor.

**Kullanıcının önerdiği çözüm ve doğrulanan sonuç:** eşikle (K_NORMAL)
boğuşmak yerine, `heel_pen`/`toe_pen` çiftinden doğrudan üç fazlı bir
durum makinesi (`classify_phase()`) türetilip, kayma hesaplaması faz'a
göre **maskelendi** (`run_phase_machine()`):

- **Heel-Strike** (`heel_pen > eşik` ve `toe_pen == 0`): sadece topuk
  hesaplanır, parmak ucu o kare tamamen göz ardı edilir.
- **Flat-Foot** (`heel_pen > 0` ve `toe_pen > 0`): ağırlık-aktarım (roll)
  fazı; iki düğüm de paylaşımlı bir kapasiteye
  (`MU_STATIC * k_normal * (heel_pen + toe_pen)`) karşı hesaplanır.
- **Toe-Off** (`heel_pen == 0` ve `toe_pen > eşik`): sadece parmak ucu
  hesaplanır, topuk tamamen susturulur (masking).
- Tanımsız durum (tek düğümde eşik-altı temas) → `airborne`'a düşürülür.

**Doğrulama (izole laboratuvar, yürüyüş itkisi KAPALI — checkpoint 3 ile
aynı izolasyon disiplini):** 9 farklı (`heel_lead_deg`, `drop_height`)
kombinasyonunun 8'inde faz dizisi `airborne → heel_strike → (kısa,
eşik-altı bir sıçrama) → flat_foot` şeklinde **ardışık** ilerliyor ve
test penceresinin **%79-90'ında** (197-225/250 kare) tek bir `flat_foot`
fazında **kesintisiz** kalıyor (NaN yok, faz çırpınması yok). En temiz
örnek (`heel_lead=15°, drop=15.0`):

```
airborne(22) -> heel_strike(4) -> airborne(1) -> flat_foot(223)
```

**En önemli bulgu — aranan asimetrik kayma ayrımı elde edildi:**
`heel_lead=15°, drop=5.0` koşusunda `heel_slip=True` olan tüm kareler
`[8, 9, 10, 11, 12, 13]` — hepsi `heel_strike` fazında, tam darbe tepe
noktasında (`heel_pen`: 0.81 → 1.58 → 0.76). Aynı koşuda `toe_slip`
hiçbir karede tetiklenmiyor (topuk kayarken parmak ucu maskeli).
Yerleşik `flat_foot` kuyruğunun tamamında (200+ kare) her iki düğümde de
kayma sıfır — checkpoint 3'ün K_NORMAL ile asla ayrıştıramadığı "gerçek
darbe" ile "sahte sıfır-yük kayması" ayrımı, hiçbir eşik ayarına ihtiyaç
duymadan, sadece faz bilgisiyle çözülüyor.

**Bulunan sınır durumu (dürüst rapor):** `heel_lead=35°, drop=5.0`
kombinasyonunda parmak ucu testin tamamında hiç yere değmiyor
(`toe_pen` sürekli 0.0) — çok dik başlangıç eğimi + yetersiz düşme
enerjisi, ayağın sadece topuk üzerinde sönümlenen bir sarkaç gibi
sallanmasına yol açıyor; genlik eşiğin altına inince tanımlı üç fazdan
hiçbirine uymayıp `airborne`'a düşüyor. Bu bir hata değil — yeterli
potansiyel enerjisi olmayan eğik bir sarkacın parmak ucunu hiç yere
değdirmemesi beklenen bir sonuç; bu geometri/enerji kombinasyonunun test
kapsamı dışında kaldığını gösteriyor.

**Sonuç:** üç fazlı durum makinesi ve faz-maskeli kayma mantığı izole
laboratuvarda doğrulandı — ardışık ve stabil çalışıyor, ve checkpoint
3'ün çözemediği "yürüyüş vs. darbe" ayrımını gerçekten çözüyor.
`classify_phase()` ve `run_phase_machine()`, `demo/step16_segment_foot_lab.py`'ye
eklendi. `active_gait.py`'ye tam entegrasyon henüz yapılmadı — bu commit
sadece izole doğrulamayı gerçek repoya taşıyor.


## Adım 16 eki (checkpoint 5) — Faz makinesinin gerçek dış müdahale (push) altında "savaş testi": active_gait.py entegrasyonundan önceki son izole doğrulama

Kullanıcı, checkpoint 4'ün faz-maskeli durum makinesini `active_gait.py`'nin
karmaşık çok-bacaklı ortamına taşımadan önce, gerçek bir dış müdahale
(push) altında sınamak istedi: kapsülü yürüyüş itkisiyle `flat_foot`
fazına oturtup, tam bu fazın ortasında ani bir yatay darbe uygulayıp,
sistemin darbeyi doğru algılayıp sadece gereken kayma sinyalini üretip
üretmediğini ve ayağın `airborne`'da takılıp kalmadan toparlanıp
toparlanamadığını ölçmek. Gerekçe: iki bacağı birbirine bağlayıp
yürütmeye başladığımızda bir şeyler ters giderse, hatanın faz makinesinden
mi yoksa bacaklar arası koordinasyondan mı geldiğini asla izole
edemeyeceğimiz için, önceden "çalıştığı kanıtlanmış" bir referans push
testi bırakmak.

**Yön asimetrisi (beklenmedik ama tutarlı bulgu):** `run_push_test()`,
hip'e projenin standart ani-darbe konvansiyonuyla bir yatay kayma
uyguluyor. İki yön çok farklı davranıyor:

| push_px | yön | f25 heel_pen | f25 toe_pen | f25 faz | heel_slip | toe_slip |
|---|---|---|---|---|---|---|
| -150 | geri | 0.0000 | 0.0000 | airborne | 0 | 0 |
| -50 | geri | 3.0675 | 3.1355 | flat_foot | 0 | 0 |
| **-20** | **geri** | **2.3040** | **1.8681** | **flat_foot** | **0** | **7** |
| -10 | geri | 1.4178 | 1.0554 | flat_foot | 0 | 0 |
| 1 | ileri | 0.0000 | 0.0453 | airborne | 0 | 0 |
| 2 | ileri | 0.0000 | 0.0000 | airborne | 0 | 1 |
| 150 | ileri | 0.0000 | 0.0000 | airborne | 0 | 0 |

**İleri yönde** (yürüyüş yönünde) +2px kadar küçük bir darbe bile ayağı
AYNI karede tamamen havaya kaldırıyor — checkpoint 3'ün "yatay darbe
ayağı bastırmıyor, kaldırıyor" bulgusunun +1/+2px'e kadar keskinleştirilmiş
hali. Bu bir hata değil: kütle merkezi destek noktasını geçtiği an, bu
rijit tek-bacak/ters-sarkaç geometrisinde bacak yerden kesilmek zorunda
(mutlak Newton kinematiği). **Geri yönde** (yürüyüşe karşı) orta
büyüklükte bir darbe (-20px) ayağı gerçekten yüklüyor (heel_pen/toe_pen
~15-16x artıyor) ve tam aranan dizi ortaya çıktı:

```
f=25-28  flat_foot  (gerçek yükleme, heel_pen/toe_pen zirvede)
f=29-52  airborne   (darbe ayağı GEÇİCİ olarak havalandırıyor)
f=53-54  toe_off    (parmak-ucu-önce TEMİZ iniş)
f=63-69  flat_foot  (yeniden basış) + toe_slip=True 7 kare KESİNTİSİZ
```

`heel_slip` hiç tetiklenmedi (doğru — darbe topukta değil parmak ucunda
yük yarattı). -50px ve -150px'te darbe çok büyük kalıp aynı anlık-kalkış
modunu tetikledi (+50/+150px ile aynı sonuç) — yani "yükleyip devirmeyen"
pencere dar ama gerçek ve tekrar üretilebilir.

**Maskeleme en agresif anda bile doğru çalıştı:** -50px darbesinde, ayak
havalandığı anda (`heel_pen=0`, `f_max=0`) ham `shear/f_max` oranı teknik
olarak eşiği aşıyordu (`shear=-2.87`, `f_max=0.0`) ama `airborne` dalı
bunu doğru şekilde bastırdı — test edilen hiçbir konfigürasyonda (9 farklı
`push_px` değeri, iki yön) sahte kayma üretilmedi. `foot_len_dev` (rijit
heel-toe mesafesi) tüm testlerde 1.7px altında kaldı, NaN hiç görülmedi,
faz çırpınması hiçbir konfigürasyonda oluşmadı.

**Sonuç:** faz makinesi bu savaş testinin hiçbir biçiminde bozulmadı —
yanlış sınıflandırma, çırpınma, sahte kayma veya sayısal patlama yok; ve
geri yönde orta büyüklükte bir darbede (-20px) tam aranan "yükle → geçici
ayrıl → toe_off'a temiz gir → flat_foot'a dönerken gerçek kayma üret"
dizisini üreterek nihai doğrulamasını geçti. Yön asimetrisi yeni bir hata
değil — gövde/karşı-bacak kütlesi olmadan bu rijit tek-bacak modelinin
sonsuza dek yüklü kalamamasının, checkpoint 3'ten beri bilinen aynı
mimari gerçeğin bir başka yüzü. `run_push_test()`, `demo/step16_segment_
foot_lab.py`'ye eklendi. `active_gait.py`'ye tam entegrasyon SIRADA.


## Adım 17 — Bilge derisi gerçek fizik iskeletinde + Faz A kinematik kontak-faz sensörü

Kaynak: Gül Nihal'in `codex/bilge-yuruyus-ve-sahne` dalı Git LFS ile
birleştirildi (medya ~21 MB, pointer olarak). Onun `bilge_walk_validation.py`
/ `bilge_walk_skinned.py` demoları kalçayı kinematik bir eğriyle
(`pelvis_at`) sürüyor; ana motor **değil**, render referansı olarak yerinde
duruyor. Bu adım 16 parçalı deriyi `step14`'ün gerçek fiziğine giydirdi ve
`active_gait.py`'ye mutlak açı tabanlı kontak-faz sensörünü (Faz A) ekledi.

**Çalıştırma:** `python3 demo/step17_bilge_physics_skin.py` (normal video),
`--debug` (iskelet + faz etiketleri + faz şeridi), `--roll-proposal`
(kabul EDİLMEMİŞ deney, aşağıya bkz.), `--preview-only`.

### 1. Refactor: `ActiveBipedSim` (davranış değişmedi)
`step14`'ün `main()` içindeki fizik döngüsü, kare-kare adımlanabilen
`ActiveBipedSim` sınıfına birebir taşındı; böylece çubuk-adam ve Bilge derisi
AYNI fizik kodunu kopyalamadan kullanıyor. Doğrulama: `step14` konsol raporu
satır satır aynı; 360 karenin tüm Verlet noktaları + iki bacağın FABRIK
noktaları **bit-bit aynı** (`np.array_equal`).

### 2. Faz A sensörü (`physics/active_gait.py`)
`classify_contact_phase(leg_angle_deg)`: ayak→kalça vektörünün dikeyle açısı
(`step14`'ün `leg_angle_deg` tanımı, yürüyüş yönü +x). `< -2°` → `heel_strike`,
`> +2°` → `toe_off`, arası `flat_foot`; swing'de `swing`. Faz etiketleri
`step16` laboratuvarınınkiyle aynı. **Saf gözlemci:** eklendikten sonra
`step14` parmak izi bit-bit aynı; `step15`/`step16` çıktıları satır satır aynı.

### 3. Bulunan ve düzeltilen hata: diz yönü (deri giydirilince göründü)
`step14`'te `KNEE_BEND_SIGN=-1.0` idi. Karakter +x yönünde yürüdüğü için bu,
FABRIK dizini **salınım karelerinin %100'ünde (384/384) geriye** (kuş dizi)
büküyordu. Çubuk-adamda fark edilmemişti. Sadece işareti çevirmek yetmedi:
`clamp_joint_angles()` yanlış daldaki çözümü uyluğu sabit tutarak aynaladığı
için ayak Bezier hedefinden **116 px'e kadar geriye** savruldu. Arkadaşının
render'ında görülen "adımların geride kalması" ile aynı mekanizma bu.
Düzeltme: `ActiveFootPlantingLeg._seed_knee_branch()` çözümden önce dizi
yürüyüş yönünde ~30° ileriye tohumluyor, `KNEE_BEND_SIGN=+1.0`. Sonuç: stance
ve swing'de **0 geri diz karesi**. Zincir ucu gait hedefiyle çakışıyor
(medyan 0.12 px). Tek istisna 45 swing karesi: bu karelerde hedef bacak
boyunu (184 px) aşıyor, sapma en fazla 47 px. Bu sapma fiziğin geometrisinden
geliyor, raporda ayrı gösteriliyor. Zincir noktaları kalça dinamiğine geri
beslenmediği için (anchor = planted) Verlet noktaları bit-bit aynı kaldı.
`step3`–`step13` kendi sabitlerini kullanıyor, **dokunulmadı**.

### 4. Deri eşlemesi (fiziğe yazılmayan, sadece çizim uzayında)
- Kalça, omuz ve baş Verlet noktalarından geliyor. Ayak konumu gait'in kendi
  hedefinden alınıyor (stance'ta `planted`, swing'de Bezier noktası).
- Gövde ve boyun **yönü** fizikten, **uzunluğu** rig'den geliyor (55 → 86 px).
  COM hesabı gerçek 55 px'lik noktaları kullanıyor.
- Kollar `step14`'te yok. Kol salınımı, iki ayağın fiziksel x-farkından
  türetilen bir FABRIK hedefi. Fiziksel bir kol DEĞİL; Prosedürel Kol
  Salınımı hâlâ yol haritasında.
- Ayakkabı eğimi Faz A'ya bağlı. `heel_strike`'ta topuk pivotunda parmak ucu
  kalkıyor, `toe_off`'ta parmak ucu pivotunda topuk kalkıyor, `flat_foot`'ta
  eğim 0. Dönmüş kabuğun en alt noktası zemine oturtuluyor. Swing'in son
  %40'ında eğim, iniş noktasının Faz A tahminine yumuşakça yaklaşıyor.
- Diz, kalçadan görsel bileğe sabit kemik boylarıyla analitik 2-kemik IK ile
  ve ileri bükülerek çözülüyor.

### 5. Ölçümler (12 s standart senaryo: 15 px tökezleme + 150 px itki)
| Metrik | Varsayılan gait (6/12) | `--roll-proposal` (60/−15) |
|---|---|---|
| Düştü mü / adım / acil adım | hayır / 27 / 36 | hayır / 25 / 7 |
| Görünür taban gömülmesi | 0 px | 0 px |
| Stance pivot kayması (fizik kayması hariç) | 1.15 px | 0.29 px |
| En küçük parça örtüşmesi | 90 px | 90 px |
| Monoton olmayan faz dizisi | 0 | 0 |
| Tam HS→FF→TO yuvarlanması (≥5 karelik stance) | 3 / 28 | 22 / 26 |
| Tek karelik stance ("dikiş makinesi" kurtarma adımı) | 33 | 6 |
| İnişte eğim sıçraması (ort. / maks.) | 8.2° / 27.5° | 3.1° / 29.7° |
| İncik gerilmesi p95 / maks. (aşırı uzanma) | 6.1 / 51 px | 1.6 / 61 px |
| İtki sonrası en düşük kalça yüksekliği | 110 px | **60 px** (diz çökmesi) |

### 6. Faz A'nın asıl bulgusu: varsayılan gait topuktan hiç çıkmıyor
60 saniyelik rahatsız edilmemiş koşuda (varsayılan `SUPPORT_MARGIN=6`,
`SWING_LEAD_MARGIN=12`) **171 stance'ın 0'ı** tam yuvarlanma yapıyor. Stance
karelerinin **%99'u `heel_strike`**. Sebep capture-point eşiği:
`xcp = hip + vx/ω0 ≈ hip + 44 px`, bu yüzden bacak kalça ayağın ~38 px
GERİSİNDEYKEN bırakılıyor ve kalça ayağın önüne hiç geçmiyor. Görselde
karakter geriye yaslanmış, topukları üzerinde yürüyor gibi duruyor.
İzole tarama (commit edilen varsayılanlar değişmedi):

| SUPPORT / LEAD | 12 s itki: en düşük kalça yüksekliği | 60 s itkisiz: acil adım | tam yuvarlanma | HS/FF/TO |
|---|---|---|---|---|
| 6 / 12 (mevcut) | 110 px | 0 | 0/171 | .99/.00/.01 |
| 30 / 12 | 72 px | 55 | 25/135 | .68/.05/.27 |
| 50 / 0 | 37 px | 0 | 110/112 | .68/.27/.05 |
| 55 / −10 | **13 px** (neredeyse düşme) | 0 | 122/123 | .50/.34/.16 |
| **60 / −15** | **60 px** (diz çökmesi) | **0** | **123/124** | **.32/.37/.32** |

Uyarı: `step14`'ün "düştü" eşiği (`hip_y > 320`, yani kalça yerden 10 px)
çok gevşek; tablodaki bütün satırlar bu eşiğe göre "ayakta". Ama 60/−15'te
150 px itkiden sonra kalça 60 px'e iniyor ve görselde karakter dizlerinin
üzerine çöküyor (önizlemedeki "İTKİ SONRASI" karesi). Yuvarlanmayı kazanmanın
bedeli itki dayanıklılığı. Bu yüzden bu satır tek başına "çözüm" değil, bir
sonraki turun başlangıç noktası.

60/−15 ile NaN yok, ort. hız 1.89 px/kare. Ancak adımlar hâlâ kısa
(`vx≈2 px/kare`) ve eğim açıları küçük (±5°), bu yüzden yuvarlanma
görselde ince kalıyor. Bu bir gait değişikliği: `step14` kanaryalarını
değiştirir, onay olmadan varsayılan yapılmadı. `--roll-proposal` bayrağıyla
yan yana izlenebiliyor.

Önizlemeler: [varsayılan](docs/previews/step17_bilge_physics_skin.png),
[Faz B kapalı](docs/previews/step17_bilge_physics_skin_nofazb.png); tam
ölçüm raporları `docs/validation/step17_*_report.json`.

> **GÜNCELLEME (Adım 18):** 60/−15 artık varsayılan, `--roll-proposal` bayrağı
> `--legacy-heel-gait` oldu, Faz B uygulandı — bkz. "Adım 18".

### 7. Faz B durumu: BAŞLATILMADI
Koşul "görsel doğal akıyorsa" idi. Varsayılan gait'te sensör doğru çalışıyor
ama ortaya çıkardığı yürüyüş doğal değil (topukta yürüme, itki sonrası 33
tek karelik stance). Push altındaki opportunistic override bu tabanın üstüne
kurulursa, override'ın etkisi gait'in kendi kusurundan ayrıştırılamaz. Bu,
Adım 16'daki "hatanın kaynağını izole edebilmek" ilkesinin aynısı. Önce
6.'daki gait kararı verilmeli.

## Adım 18 — 60/−15 yuvarlanma geometrisi varsayılan + Faz B: toe_off tetikli yakalama adımı

### 1. Geometri artık varsayılan
`step14`: `SUPPORT_MARGIN 6 → 60`, `SWING_LEAD_MARGIN 12 → −15`;
`ActiveFootPlantingLeg` sınıf varsayılanları da aynı. Eski değerler
`LEGACY_SUPPORT_MARGIN/LEGACY_SWING_LEAD_MARGIN` olarak duruyor
(`step17 --legacy-heel-gait`). Kanaryalar değişti (beklenen). 60 s itkisiz:
124 adım, 0 acil adım, 0 çift-havada karesi, stance açısı −7.8°…+14.4°,
123/124 tam HS→FF→TO.

### 2. Çöküşün teşhisi (Faz B'siz, 150 px ileri itki)
Kare kare iz (`t=7.0 s`): itki geldiğinde sol bacak havada
(`swing_t=0.30`), sağ bacak stance. Kalça 65 px/kare ile ileri fırlıyor.
Havadaki bacağın MAR hedefi kare başına 8 px ile sınırlı olduğu için hedef
kalçanın 119 px gerisinde kalıyor. Aynı anda `FallRiskMonitor` stance
bacağını acil adıma atıyor, ama diğer bacak zaten havada. Sonuç: 11 çift-havada
karesi. Kalçayı tutan tek şey geride kalmış eski anchor, kalça onun etrafında
sarkaç gibi düşüyor (183 → 60 px). İkinci darbe: toparlanırken `vx≈7–20` ile
yapılan normal capture-point bırakışları `xcp = hip + vx/ω0 ≈ hip + 22·vx`
yüzünden hedefi kalçanın 150–430 px önüne koyuyor, ayak 260 px ileriye basıyor.

### 3. Faz B mekanizması (`physics/active_gait.py`, `step14` bağlantısı)
- **Tetik (Faz A sensöründen):** stance bacağı `toe_off`'ta ve açı
  `> FAZB_TOE_OFF_OVERRUN_DEG = 20°` (itkisiz yürüyüşte hiç aşılmıyor), ya
  da `FallRiskMonitor` tehlike bildirirken bacak `toe_off`'ta.
- **Eylem, tek-destek kuralını bozmadan:** Havada bacak varsa
  `compress_swing()` onun kalan salınımını 3 kareye sıkıştırıyor. Bu
  `_active_swing_duration` üzerinden yapılıyor, `swing_t` değişmiyor, yani
  Bezier konumu sıçramıyor. Havada bacak yoksa `launch_catch_step()` toe_off
  bacağını 3 karelik salınımla hemen bırakıyor.
- **Hedef:** Yakalama salınımında hedef her kare, iniş anındaki tahmini kalça
  konumuna doğru kayıyor. Kayma sınırı `25 px + |vx|`.
- **Sıkıştırılmış bırakış:** Toe_off aşımı sürerken gelen normal
  capture-point bırakışı da sıkıştırılmış yakalama salınımına dönüşüyor.
- **Erişim sınırı:** `FAZB_MAX_STEP_AHEAD_PX = 70`. Capture-point hedefi
  kalçadan en fazla 70 px önde ya da geride olabilir. Normal yürüyüşte hedef
  ~+30 px olduğu için hiç devreye girmiyor.
- **Doğrulama:** 60 s itkisiz koşu Faz B açıkken ve kapalıyken bit-bit aynı,
  0 tetik. `FAZ_B_ENABLED` / `ActiveBipedSim(faz_b=False)` / `step17 --no-faz-b`
  ile kapatılabilir.

### 4. Ölçümler (15 px tökezleme + t=7 s itki, 16 s)
| İtki | Faz B KAPALI: en alçak kalça / çift-havada / acil adım | Faz B AÇIK: en alçak kalça / çift-havada / acil / yakalama |
|---|---|---|
| +50 | 146 / 13 / 12 | 149 / 2 / 2 / 2 |
| +100 | 90 / 13 / 8 | 119 / 0 / 0 / 1 |
| **+150** | **60 / 11 / 7** | **92 / 0 / 0 / 1** |
| +200 | 65 / 8 / 5 | 118 / 0 / 0 / 1 |
| +300 | 59 / 6 / 4 | 102 / 0 / 0 / 1 |
| +500 | 60 / 7 / 5 | 81 / 0 / 0 / 1 |
| −100 (geri) | **DÜŞTÜ** | 95 / 5 / 3 / 2 |
| −150 (geri) | **DÜŞTÜ** | 62 / 9 / 8 / 1 |

Tek senaryo tuzağına karşı 150 px itki, yürüyüş döngüsünün 12 farklı anında
verildi (`t = 7.0 + k/30`). Faz B kapalıyken en alçak kalça 12–62 px arası,
bir durumda düştü, 9–27 çift-havada karesi var. Faz B açıkken en alçak kalça
90–150 px, düşme yok, **0 çift-havada karesi**.

Deri (step17, varsayılan senaryo): 0 tek karelik stance (eskiden 6). İnişte
eğim sıçraması ortalama 1.0°, en fazla 8° (eskiden 3.1° / 29.7°). Taban
gömülmesi 0, en küçük parça örtüşmesi 90 px.

### 5. Dürüst sınırlar
- **Bir kare gecikme:** Sensör konum ölçüyor, hız ölçmüyor. İtkinin geldiği
  karede açı henüz +17°, tetik bir sonraki karede (+28°). Hızdan tahmin
  edilen açı bunu bir kare öne çekebilir. Bu turda denenmedi.
- **Yakalamadan sonra kalça 57 px/kare yükseliyor:** Faz B yokken bu 75
  px/kare idi. Ayak kalçanın altına indiğinde anchor–kalça çubuğu (sabit 184
  px, compliance 0.85) sıkışmış halden geri yayılıyor. Görselde derin
  hamleden bir-iki karede dikilme. 9./10. turlardaki anlık handoff'un aynısı;
  yumuşatılması ayrı bir iş.
- **Geri itkiler Faz B kapsamı dışında:** Tetik yalnızca `toe_off` tarafında.
  Geri itkide kalça ayağın gerisine düşüyor (`heel_strike` tarafı), eski
  `trigger_emergency_step` yolu hâlâ çalışıyor ve çift-havada kareleri orada
  kalıyor (−150 px'te 9). Düşme artık yok (erişim sınırı sayesinde), ama
  kalça 62 px'e iniyor. Aynı mekanizmanın `heel_strike` aynası sıradaki aday.
- **FABRIK erişemiyor:** Yakalama hamlesinde arka ayak 150 px geride
  kalabiliyor. Bacak zinciri bu hedefe erişemiyor (24 karede >5 px). Deri
  ayakkabıyı gait hedefine koyuyor, incik en fazla 14 px geriliyor.

## Adım 19 — Faz B'nin heel_strike aynası, yakalama sonrası şok emilimi, prediktif sensör

### 19a. Geri itki simetrisi (heel-side override)
`ActiveFootPlantingLeg.heel_strike_overrun()`: stance'ta, `heel_strike`
fazında ve açı `< FAZB_HEEL_STRIKE_OVERRUN_DEG = −20°`. İtkisiz 60 saniyede
stance açısı hiç −7.8°'nin altına inmiyor. `catch_overrun()` `"toe"`,
`"heel"` ya da `None` döndürüyor. Tehlike tetiği de artık cepheye duyarlı:
COM öndeyse ve bacak `toe_off`'taysa ya da COM geride ve bacak
`heel_strike`'taysa yakalama devreye giriyor. Faz B tetiklendiği karede eski
`trigger_emergency_step` yolu çalışmıyor. Yakalama hedefi (`_catch_target_x`)
zaten hız işaretine duyarlıydı, geri tarafta ek kod gerekmedi.

| İtki | Adım 18 (sadece toe_off): en alçak kalça / çift-havada / acil | Adım 19 |
|---|---|---|
| −50 | 130 / 5 / 4 | 139 / 0 / 0 |
| −100 | 95 / 5 / 3 | 117 / 0 / 0 |
| −150 | 62 / 9 / 8 | 93 / 0 / 0 |
| −200 / −300 / −500 | — | 78 / 68 / 52, 0 çift-havada, düşme yok |

150 px itki yürüyüş döngüsünün 12 farklı anında: ileri itkide en alçak kalça
90–155 px, geri itkide 73–129 px. Çift-havada 0, düşme 0, eski acil adım 0.
İleri itki sonuçları Adım 18 ile aynı. İtkisiz 60 saniyede Faz B açık ve
kapalı bit-bit aynı, 0 tetik.


### 19b. Kinetik şok emilimi (yakalama sonrası dikilme)
**Kaynak:** Yakalama ayağı kalçanın altına indiğinde anchor–kalça çubuğu
(184 px, compliance 0.85) çok sıkışmış durumda. Tam boyuna tek iki karede
yaylanıp kalçayı 57 px/kare fırlatıyordu.

**İlk hata:** Geri itkide yakalama ayağı indiği karede iki bacak birden
stance'ta olabiliyor ve anchor, "döngüdeki son stance bacağı" kuralı yüzünden
öbür (uzaktaki) ayakta kalıyor. Emilim anchor'ın yakalama bacağına
**gerçekten geçtiği** karede başlatılıyor. Anchor ona hiç geçmeden bacak
tekrar kalkarsa bekleyen emilim iptal ediliyor.

**Mekanizma (`step14`, SADECE yakalama inişinden sonra):**
`rest_k = min(ARM, max(rest_{k−1}, d_now) + SHOCK_RISE_CAP_PX)`. Bu bir
yükseliş hızı sınırlayıcı ve monoton: dinlenme boyu hiç kısalmıyor, yani
destek asla gevşemiyor. Kalça `SHOCK_FLOOR_PX = 90` altındaysa sönümleme
bırakılıyor, çubuk tam rijit boyuna dönüyor. `compliance` değişmiyor (13. tur
eki 3 dersi). Normal adımlarda devreye girmiyor (11. turdaki Rest Length
Lerping'in reddedilme sebebi buydu). İtkisiz 60 saniye emilim açık ve kapalı
bit-bit aynı.

**Tarama** (±150/300/500 px itki + ±150 px itkinin 12'şer farklı gait anı,
toplam 30 koşu):

| Yöntem | En hızlı yükseliş (maks / medyan) | ±150 en alçak kalça | −500 en alçak kalça | Düşme |
|---|---|---|---|---|
| Kapalı (Adım 19a) | 65 / 42 px/kare | 92 / 93 | 52 | 0 |
| Birinci derece low-pass (oran 0.1–0.5) | 13–35 / 9–29 | 83 / 84 | **düştü** | 1 (her oranda) |
| Hız sınırı, taban yok (cap 10 / 15 / 20) | 16 / 22 / 27 maks. | 89–93 | düştü / 26 / 33 | 1 / 0 / 0 |
| **Hız sınırı cap 15 + taban 90 (seçilen)** | 65 / **20** | **92 / 93** | **52** | **0** |

Low-pass bu yüzden reddedildi. Seçilen ayarda tipik yakalamada yükseliş
57 → 21 px/kare. Görselde çömelmeden 5–6 karede doğrulma. En aşırı
senaryoda (−500 px, kalça < 90 px) bilinçli olarak eski rijit davranışa
dönülüyor, bu yüzden tabloda en yüksek yükseliş hâlâ 65. Yumuşaklığın kalçayı
daha da çöktürmesine izin vermiyoruz.

**Yan düzeltme:** Diz tohumu artık bir önceki karenin zincir ucuna değil, bu
karenin hedefine (stance'ta `planted`, swing'de `swing_target`) bakıyor.
Sıkıştırılmış yakalama salınımında eski uç kalçanın üstünde kalabiliyor ve
bir karede dizi geriye atıyordu. Gövde noktaları tohumdan bağımsız, bit-bit
aynı.


### 19c. Prediktif sensör
Faz A sensörü bir önceki karenin sonundaki konumu okuyor. İtki
`prev_points` üzerinden verildiği için itkinin geldiği karede kalça henüz
hareket etmemiş oluyor, açı ancak bir sonraki karede sıçrıyor.
`ActiveFootPlantingLeg.predict_contact(hip, hip_prev)` bir sonraki karenin
kalça konumunu Verlet'in kendi kuralıyla tahmin ediyor (`x + (x − x_prev)`)
ve stance açısını ondan hesaplıyor. Faz B'nin aşım tetiği ölçülen ve tahmin
edilen açının **daha uç olanını** kullanıyor. Tahmin gecikmeyi kapatıyor,
ölçüm de tahmin hatasına karşı taban oluyor. Faz A'nın `contact_phase`'i
(görsel/ölçüm) değişmiyor. Kapatmak için `PREDICTIVE_SENSOR_ENABLED` /
`ActiveBipedSim(predictive_sensor=False)`.

| İtki | Tahminsiz: gecikme / en alçak kalça | Prediktif: gecikme / en alçak kalça |
|---|---|---|
| +150 | 2 kare / 92 | **0 / 154** |
| +300 | 1 / 102 | 0 / 126 |
| +500 | 1 / 81 | 0 / 104 |
| −150 | 1 / 93 | 0 / 115 |
| −300 | 1 / 68 | 0 / 86 |
| −500 | 1 / 52 | 0 / 71 |
| ±150, 24 gait anı | ort. 1.38 kare / 73–155 | **ort. 0.08 / 92–154** |

Tüm koşularda çift-havada 0, eski acil adım 0, düşme 0, hepsi toparlanıyor.
İtkisiz 60 saniyede tahmin açık ve kapalı bit-bit aynı, 0 tetik. Sadece
15 px tökezlemede de 0 tetik.

"Çarpıcı sonuca şüpheyle bak" kuralı gereği +150 px kare kare izlendi. Kalça
her karede sürekli ilerliyor (+65, +38, +23, +8… px), sıçrama ya da ışınlanma
yok. Havadaki bacak itki karesinde sıkıştırılıyor, 2 kare sonra kalçanın
48 px önüne iniyor (heel_strike −17°), kalça 154 px'te durup 2 karede normal
yüksekliğe dönüyor. Bu bir tökezleme adımı; önceki versiyondaki derin
hamle/çömelme artık yok.

### Adım 19 özet kanaryaları (`step14` varsayılan senaryo)
26 adım, 0 acil adım, 2 kayma karesi, 1 Faz B yakalaması (itki karesinde),
son `hip_y` 146.60. Testler 47/47.

## Adım 20 — Fiziksel (Verlet) kollar: kütle, momentum dengeleme torku, itki refleksi

Kozmetik kol sürücüsü (`step17`'de ayak x-farkından türetilen FABRIK hedefi)
kaldırıldı. Kollar artık `physics/arms.py`'deki `PhysicalArms`: omuz Verlet
noktasına asılı kütleli dirsek ve el noktaları, iki rijit çubuk. Aynı
`VerletSystem` kısıt çözücüsünde gövdeyle birlikte çözülüyorlar, yani omuz
üzerinden kalçaya geri etki ediyorlar. `ARMS_MODE` (`off` / `passive` /
`drive`) ve `ARM_REFLEX` ile kademeli kapatılabiliyor. `arms_mode="off"`
Adım 19c'nin kanaryalarını aynen veriyor (26/0/2/1/146.60, test edildi).

### 20.1 Kütle ve eklemler
- **Kütle:** Gövde (kalça + omuz + baş = 3 birim) anatomik olarak ~%58
  sayıldı (Winter segment tablolarına göre yuvarlatılmış oranlar). Bir kol
  ~%5 ediyor: dirsek 0.16, el 0.10. Fırıldak etkisi yok: gövde açısı itkiler
  dahil hiç 12°'lik kelepçeyi aşmıyor.
- **Uzunluk:** Fizikte 34/32 px (gövde ölçeği 55). Deride rig'in 54/50 px'i
  kullanılıyor. Yön fizikten geliyor, uzunluk rig'den (gövdedeki ilke).
- **Eklemler:** Omuz için aşağı yöne ±100° koni. Dirsek tek yönlü menteşe
  (`clamp_joint_angle_points`, 2°–140°), hiperekstansiyon yok. Dirsek
  yay-sönümü `apply_angular_spring` ile.
- **Bulunan 1 — "omuzdan kopma":** Gövde/boyun kelepçeleri `body.step()`'ten
  sonra omzu taşıyor ve kol çubukları açık kalıyordu. İtkide 36–87 px boy
  hatası ölçüldü. Kol noktaları kelepçeden sonra omza yeniden bağlanıyor;
  aynı kaydırma `prev_points`'e de uygulanıyor, yani kolun kendi hızı
  korunuyor. Hata normal yürüyüşte 0, itkide 2–5 px.
- **Bulunan 2 — sahte açısal hız:** Kolun açısal hızı önce `prev_points`'ten
  okunuyordu. Gövde kelepçesi (`preserve_momentum=True`, 5. tur) omzun
  `prev_points`'inde duvara doğru saklanan bir hız bırakıyor: itkisiz ~2
  px/kare, itki sonrası ortalama 14 px/kare. Kolsuz gövdede de var, önceden
  var olan bir artefakt. Kol sabit dururken −0.2 rad/kare okunuyor, PD'nin
  sönüm terimi hedefe dönüşü iptal ediyordu: kollar 40°'de asılı kaldı. Hız
  artık kaydedilen bir önceki kol açısından ölçülüyor.

### 20.2 Momentum dengeleme torku (`drive`)
Hedef kol açısı = −1.5 × aynı taraftaki bacağın kalça→ayak açısı. Bu PD bir
omuz torku: k=0.3, c=0.8, tavan 0.15 rad/kare². Tork dirseğe tanjantiyel hız
değişimi olarak uygulanıyor, eşit ve ters doğrusal momentum omza veriliyor
(Newton 3).

**Bacak açısal hızı ileri beslemesi denendi, reddedildi.** FABRIK zincir
ucunun açısı iniş karelerinde sıçrıyor; hız hedefi bu sıçramayı kola tork
olarak taşıyordu. Korelasyon −0.86'dan −0.71'e düştü, genlik ±25–36° oldu.
`max_leg_rate=0`, yani kapalı.

| 60 s itkisiz | Kol yok | Pasif | **Drive (seçilen)** |
|---|---|---|---|
| Düşme / adım / yuvarlanma | yok / 124 / 123 | yok / 123 / 122 | yok / 123 / 122 |
| Ort. hız, medyan kalça | 1.92, 183.7 | 1.92, 183.7 | 1.92, 183.7 |
| Kol–bacak açı korelasyonu (aynı taraf) | — | +0.03 / +0.25 | **−0.86 / −0.86** (1 kare gecikmeyle −0.88) |
| Kol genliği | — | ±3.6° | ±16° |

−1'e tam ulaşılmıyor. Omuz her itki karesinde kalçayla birlikte ivmeleniyor
ve sarkaç gibi asılı kol bu ivmeyle arkaya geri kalıyor. Bu gerçek bir
eylemsizlik tepkisi ve kol izinin asimetrik olmasının sebebi. Açısal hız
korelasyonu ham olarak −0.35, 3 karelik yumuşatmayla −0.60.

**Dürüst sınır — açısal momentum iptali ölçülemedi:** Bacaklara anatomik
sanal kütleler atandı (diz 0.5, ayak 0.33). Kalça etrafındaki sagital
açısal momentumun RMS'i kollarla ~%0 değişti. Bunun iki sebebi var:
(a) Bacaklar bu motorda kütlesiz (FABRIK kinematiği), kolların karşı
koyabileceği gerçek bir bacak momentumu motorda yok. (b) İnsan yürüyüşünde
kol salınımının iptal ettiği momentum ağırlıkla **dikey eksen** (yaw)
etrafında. Bacaklar gövdenin iki yanında zıt yönde hareket ettiği için
oluşuyor ve tek düzlemli bir 2B sagital modelde bu eksen yok. Kolların
gövdeye uyguladığı tork gerçek, ama "iptal" iddiası bu modelde
kanıtlanamıyor.

### 20.3 İtki refleksi
Tehlike ya da Faz B yakalaması anında kollar 12 kare boyunca ±70°'ye
gidiyor (tork tavanı 0.4). İki yön ölçüldü. Kolların COM'u geri çekmesi
(ters), açısal momentum kazancının COM'un yükselmesine karşı tartıldığı
taraf. Yel değirmeni (COM'un gittiği yöne savurma) de denendi.

| ±150/300/500 px + ±150 px × 24 gait anı | Kol yok | Pasif | Drive | Drive + ters refleks | **Drive + yel değirmeni (seçilen)** |
|---|---|---|---|---|---|
| 24 anda en alçak kalça: en kötü / ortalama | 91 / 127 | 88 / 126 | 94 / 129 | 93 / 128 | **99 / 133** |
| +150 / +500 | 154 / 104 | 132 / 81 | 135 / 82 | 133 / 83 | 139 / 82 |
| −300 / −500 | 86 / 71 | 111 / 86 | 112 / 87 | 111 / 88 | 113 / 86 |
| Düşme / çift-havada / toparlanamayan | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |

Yel değirmeni, COM'u geri çeken refleksten ölçülebilir şekilde iyi: açısal
momentum kazancı COM'un yükselmesinden daha ağır basıyor. Ama etki küçük
(ortalama +4–6 px). Anatomik %5 kütleyle kollar baskın bir denge aracı
değil. Senaryo bazında karışık: ileri itkilerde kolsuz gövde daha iyi
(+150: 154 → 139, +500: 104 → 82), geri itkilerde kollar belirgin şekilde
daha iyi (−300: 86 → 113). Toplamda ve en kötü durumda (91 → 99) kazanç var.

**Yan bulgu:** 500 px'te eski acil adım yolu, diğer bacak Faz B yakalaması
için havadayken stance bacağını fırlattı (1 kare çift-havada). Faz B açıkken
eski acil adım artık diğer bacak havadayken tetiklenmiyor (tek-destek
kuralı). Kolsuz gövdenin sonuçları bu değişiklikten etkilenmedi.

### 20.4 Gerileme
İtkisiz 60 saniyede bacak kalibrasyonunu yeniden ayarlamak gerekmedi: adım,
yuvarlanma, hız ve kalça yüksekliği kolsuz gövdeyle aynı mertebede. Yeni
kanaryalar (varsayılan senaryo): 25 adım, 0 acil, 4 kayma, 2 Faz B, son
`hip_y` 146.69, itki sonrası en alçak kalça 139.4 px. Testler 50/50,
step1–16 smoke geçti. `step14` çubuk-adam çizimi kolları da gösteriyor.

**Açık kalanlar:** Refleks bitişinde kolların ~12 kare yatay kalması
görselde biraz uzun. Dikey eksen momentumu ancak 2.5B/3B bir gövdeyle
ölçülebilir. Bacaklara gerçek kütle verilmesi ayrı ve büyük bir iş.

## Adım 21 — Hayalet hızın kök nedeni, dirsek incelemesi, refleks çıkış sönümü

### 21.1 "Flamingo dirseği" ölçüldü: menteşe ters değil
214–217. karelerde dirsek açısı **−2°**, yani kollar dümdüz ve menteşenin
düz-kol sınırına dayalı. Kol hızla öne savrulurken eylemsizlik ön kolu
geriye itiyor, menteşe hiperekstansiyonu engelliyor. Ön kolun yukarı
büküldüğü kareler 223–225 (−13° → −31°). Bu, kol yavaşlarken ön kolun
eylemsizlikle devam etmesi. Yön anatomik fleksiyon: kol öne uzanmışken
dirsek bükülünce el yüze doğru kalkar. Sınırları ters çevirmek asılı kolda
gerçek hiperekstansiyon üretirdi, o yüzden **yapılmadı**. Yukarı kıvrılmanın
asıl sebebi refleksin aniden kesilmesiydi; çıkış sönümü bunu azalttı (21.3).

### 21.2 Hayalet hız: kök neden düzeltildi, Adım 20'deki ayrı geçmiş kaldırıldı
`clamp_direction(..., preserve_momentum="inelastic")` ve
`clamp_joint_angle_points(..., inelastic=True)` eklendi. Momentum korunuyor,
ama eklem sınırı esnek olmayan bir duvar: sınırı aşan tanjantiyel göreli hız
siliniyor. Sadece `step14`'ün gövde/boyun kelepçeleri (`TORSO_CLAMP_MODE`)
ve kol eklemleri bunu kullanıyor; step9/13 dokunulmadı. Omuzdaki hayalet
hız 2.15 → 0.00 px/kare, itki sonrası 14.5 → 0.1 px/kare. Kol açısal hızı
yeniden gövdeyle aynı zaman çizelgesinden (`prev_points`) okunuyor.

**Ölçülen yan bulgular:**
- **Gövde her zaman sınırda:** Gövde normal yürüyüşte karelerin %100'ünde
  −12° sınırında. İtki sadece kalça noktasına uygulanıyor, gövde geride
  kalıyor. Hayalet hız bu duvara her kare çarpan hızdı. Gerçek çözüm,
  kalça–gövde arasında aktif bir dikleştirici tork olur; açık iş.
- **Eski mod sahte dayanıklılık veriyordu:** Eski (hayalet) mod 2500 px
  ileri itkiyi "atlatıyordu". Esnek olmayan modda 2500 px düşüyor,
  −1000/−1500 px ise artık ayakta (eskiden düşüyordu). Eski asimetri hayalet
  hızın bir maskelemesiydi (9. turdaki "çarpıcı sonuca şüphe" dersi).
- **Kolsuz gövdede itki dayanıklılığı arttı:** 24 gait anında en alçak kalça
  ortalama 127 → 151, en kötü 91 → 128.
- **Başlangıç sarsıntısı:** İlk karelerde gövde artık ters sınıra savruluyor.
  Kare 9'da bir Faz B yakalaması ve 8 karelik kayma var. Testler ilk 30 kareyi
  ayrı tutuyor.
- **Bayat kayma durumu:** İki ayak da yerdeyken, anchor'ın bağlı olmadığı
  ayağın kayma durumu donuk kalıyordu (`slip_velocity` −4 px/kare sabit).
  Anchor o ayağa geçince kayma 50+ kare sürebiliyordu. `STALE_SLIP_RESET`
  ile sıfırlanıyor. Etkisi küçük.

### 21.3 Refleks: çıkış sönümü, varsayılan KAPALI
Eskiden "12 kare sabit hedef, sonra ani kesinti" vardı. Artık hedef ağırlığı
3 kare tam, sonra kare başına ×0.75 üstel sönüyor; sönme süresince omuza +2.0
ek sönüm uygulanıyor. Kolun >45°'de kaldığı kare sayısı ±150 px itkide
30–36'dan 3–4'e indi.

24 gait anında (±150 px) en alçak kalça:

| | ort. ± std | en kötü |
|---|---|---|
| Kolsuz | 151.3 ± 8.9 | 128 |
| Drive, refleks yok | 152.0 ± 4.0 | **148** |
| Drive + yel değirmeni | 152.7 ± 7.1 | 138 |
| Drive + ters | 155.2 ± 4.6 | 148 |

Refleksin ortalamaya katkısı 1–3 px, yani standart hatanın 1–2 katı. Yönün
etkisi senaryoya göre değişiyor: ileri itkide kolların öne gitmesi, geri
itkide geriye gitmesi daha iyi. Sabit yönlü (asimetrik) iki politika da
denendi, ortalamalar 152 ve 155. Kör yel değirmeninin ileri itkide zarar
verdiği iddiası doğrulanmadı: +150'deki düşüş (Adım 20) refleksiz kollarda da
vardı, kolların varlığından geliyordu. Bu belirsizlik yüzünden refleks
varsayılan **kapalı** (`ARM_REFLEX=False`). Kolların asıl katkısı varyansı
yarıya indirmek (std 8.9 → 4.0) ve en kötü durumu 128 → 148'e çıkarmak.

### 21.4 Açık sorun: kollar + esnek olmayan kelepçe = çalkantılı toparlanma
İtkiden sonraki en uzun kayma serisi (24 anda medyan) şöyle: hayalet modda
kollu 8 kare, esnek olmayan kolsuz 8, esnek olmayan kollu (drive) **27**,
pasif 20. Kalça hızının 2 px/kare'den sapması da 3.4'ten 7.5'e çıkıyor.
Varsayılan senaryoda bu, itkiden ~20 kare sonra görünür bir bacak açılması ve
kayma olarak izleniyor. Pasif kollarda da olduğu için sebep kapalı döngü
torku değil: kol kütlesi artık duvardan ayrılabilen gövdeyle bağlaşıyor.

**Sol/sağ kol simetrisi:** 2B modelde iki kol aynı omuz noktasına asılı.
Pasif kollar birbirinin **aynısı** hareket ediyor (sol–sağ korelasyonu
+1.000, en büyük fark 0.08°). Kalça ivmesine tepki veren pasif bir sarkaç bu
modelde çapraz taraflı salınım **üretemez**. Çapraz salınım ya sürüş torkuyla
ya da bacaklara kütle verilmesiyle gelir.

Varsayılanlar: `TORSO_CLAMP_MODE="inelastic"`, `ARMS_MODE="drive"`,
`ARM_REFLEX=False`. Kanaryalar: 23 adım, 0 acil, 63 kayma karesi, 9 Faz B,
son `hip_y` 146.32, itki sonrası en alçak kalça 148.5. `arms_mode="off",
torso_clamp_mode=True` Adım 19c'yi aynen veriyor. Testler 50/50.

## Adım 22 — Anatomik bacak kütlesi, gerçek momentum alışverişi, gövde yaw iptali

Bacaklar bu motorda kinematikti (capture-point kararı + Bézier salınım +
FABRIK). Kütlesiz bacağın hızlanıp yavaşlaması gövdeye hiçbir tepki
vermiyordu, kolların "iptal edeceği" gerçek bir momentum yoktu. Bu adımda
bacaklara anatomik kütle verildi ve yürüyüş döngüsü buna göre yeniden
kalibre edildi.

### 22.1 Bacak kütlesi: ters dinamik, örtük/açık ayrımı (`physics/leg_mass.py`)
Kütleler (Winter segment tabloları, yuvarlanmış; gövde+baş 3.0 = %58):
uyluk 0.52, incik+ayak 0.31. Segment kütle merkezleri **pergel ekseni**
üzerinde: kalça → ayak hedefi doğrusunun %21.5'i ve %75'i.

**FABRIK dizi kullanılmadı.** Ölçüm: bacak neredeyse tam gergin (kalça–ayak
184 px = 92+92). Kalça yüksekliğindeki 0.5 px'lik değişimler dizi tek karede
±12 px yana sıçratıyor (kötü koşullu çözüm). Bu sıçramaların ikinci farkı
5–20 px/kare² sahte ivme üretiyordu. Pergel ekseninde aynı karelerde ivme
≤ 0.1.

İlk deneme (segment ivmesinin tamamını açık kuvvet olarak kalçaya vermek)
**kararsızdı**: kalçanın kendi ivmesi bir sonraki karenin bacak ivmesine
giriyor, gecikmeli bir geri besleme döngüsü oluşuyordu (|F| p99 17 → 33,
hız salınımı ±3 px/kare). Çözüm, segment konumunun kalçaya **doğrusal**
olmasından geliyor: `COM_i = (1−f_i)·kalça + f_i·ayak`.

- **Örtük pay:** Σ m_i(1−f_i) = 0.50 kalça noktasının kütlesine ekleniyor
  (salınımdaki her bacak için). Verlet kendisi çözüyor, yerçekimi de bu
  kütleye kendiliğinden etki ediyor.
- **Açık pay (ayak programı):** `F = Σ m_i f_i (a_ayak − g)`,
  `τ = Σ r_i × m_i (f_i a_ayak − g)`. Tepki: kalçaya −F itkisi, −τ ise
  kalça–omuz eksenine dik bir **kuvvet çifti** (net kuvvet 0).
- **Doğrulama:** Yerçekimsiz, zeminsiz serbest gövde + programlı salınım
  ayağıyla doğrusal momentum 80 karede **1e-13** hassasiyetle sabit (test).
  Bunun için tepki karenin **sonunda** uygulanıyor (gecikme 2 → 1 kare).

### 22.2 Kalibrasyon: neyi değiştirmek gerekti
| Değişiklik | Neden (ölçüm) |
|---|---|
| **İtki COM'dan** (`THRUST_MODE="com"`): aynı hız değişimi gövde+kol noktalarının hepsine | İtki sadece kalça noktasına verildiğinde gövde çubuğu dönüyordu. Kütleli bacakla bu mod yürüyemedi: hız 0.2–1.6, 170–750 kayma karesi |
| **Denetleyici tüm-gövde COM hızını okuyor** (`LEG_MASS_COM_CONTROL`) | Salınım bacağı ile gövde arasındaki iç alışveriş kalçayı yavaşlatıyor, sistemin COM'unu değil. Kalça hızını okuyan P-denetleyici bu iç alışverişle savaşıp zemin sürtünmesini aşıyordu |
| **Gövde duruşu: momentum koruyan kuvvet çifti** (`apply_angular_couple`, k=0.2, c=0.8) | Eski `apply_angular_spring` tek noktayı itiyordu (net dış kuvvet). c > 1'de de açık integrasyon her kare işaret değiştiriyordu (zikzak) |
| **İtkiler dürtü (momentum) olarak** | Kalça kütlesi örtük bacak payıyla artınca aynı "px/kare" itki %43 daha büyük momentum olurdu; kıyas adil kalsın diye bölündü |
| `FAZB_CATCH_FRAMES` 3'te **bırakıldı** | 4/5/6/8 kare denendi. Daha yavaş yakalama daha kötü: 6 karede 36 itkiden 5'inde düşme |

Sonuç: Gövde artık −12° duvarında değil. Duruş +3.1° ± 1.0° (eskiden
karelerin %100'ünde −12°). Bu salınım insan gövde eğimine (±2°) yakın.

### 22.3 Sagittal düzlemde kollar momentumu iptal EDEMEZ (ölçüldü)
Kütleli bacaklarla 900 karede, kalçaya göre sagittal açısal momentum:

| Kollar | bacak L std | kol L std | korelasyon | iptal |
|---|---|---|---|---|
| pasif | 131 | 3.9 | −0.44 | %1 |
| drive (Adım 20) | 130 | 11.1 | +0.05 | %−1 |

İki kol birbirine ters salındığı için sagittal momentumları birbirini
götürüyor. Gerçek insanda kol salınımının iptal ettiği şey, **dikey eksen
(yaw)** momentumu: kalça eklemi gövdenin yanında, öne atılan bacak gövdeyi
kendi etrafında burar. Bu eksen 2B motorda yoktu.

### 22.4 Gövde yaw serbestlik derecesi — 2.5B (`physics/trunk_yaw.py`)
Tek serbestlik dereceli bir gövde yaw durumu eklendi:
`I·dω/dt = −d(L_bacak + L_kol)/dt − kθ − cω`.
Segment yaw momentumu `m · yanal_ofset · (v_x − v_x_kalça)` şeklinde:
kalça yarı genişliği 18 px, omuz yarı genişliği 38 px (184 px = 0.9 m
ölçeğiyle ~9 / ~19 cm). Değerler: I = 1200 (gövde 3.0, yarıçap 20 px),
zemin serbest momenti ω_n = 0.2 rad/kare, ζ = 0.7. Yaw durumu sagittal
fiziğe geri etki **etmiyor** (tek yönlü).

**Yeni kol modu `cancel` (varsayılan):** Kollar bacakların ölçülen yaw
momentumunu sıfırlayacak açısal hızı hedefliyor:
`ω_sol = −ω_sağ = −L_bacak / (2K)`, burada `K = W·Σ m_i d_i cos a`. Tork,
`drive` ile aynı Newton-3 yoluyla uygulanıyor (sagittal tepki omza gidiyor).
İzleme kazancı 0.8, merkezleme 0.04, tavan 0.3.

| Kollar | gövde yaw RMS | maks | L_kol std | korelasyon | kalan |
|---|---|---|---|---|---|
| kolsuz | 5.00° | 10.6° | 0 | — | %100 |
| pasif | 4.94° | 9.4° | 0.0 | −0.11 | %100 |
| drive (açı PD, Adım 20) | 4.29° | 9.2° | 11.8 | −0.49 | %88 |
| **cancel** | **1.88°** | **4.3°** | 26.7 | **−0.90** | **%44** |

**Çapraz salınım kendiliğinden çıktı.** `cancel` modunda sol kol ~ sağ bacak
+0.80, sağ kol ~ sol bacak +0.85, sol kol ~ sağ kol −0.64. Kütleli
bacaklarla eski `drive` modunda iki kolun korelasyonu **+0.21**; yani açı
PD'si çapraz salınımı artık üretemiyordu. Kol açıları yaklaşık −16°…+20°.
Gövde eğimi `cancel` ile değişmiyor (±0.95°): kol sagittal pitch'i
etkilemiyor, beklendiği gibi.

### 22.5 İtki taraması (6 büyüklük × 6 faz = 36 koşu, en alçak kalça ort/en kötü)
| | +150 | +300 | +500 | −150 | −300 | −500 | düşme |
|---|---|---|---|---|---|---|---|
| Adım 21 (drive) | 159/149 | 139/125 | 108/83 | 147/132 | 126/108 | 106/78 | 0 |
| Adım 22 kolsuz | 148/131 | 120/98 | 86/**50** | 146/126 | 133/125 | 102/85 | 0 |
| Adım 22 pasif | 148/132 | 115/91 | 93/71 | 142/127 | 130/124 | 106/94 | 0 |
| Adım 22 drive | 149/133 | 115/93 | 95/74 | 144/127 | 128/123 | 106/92 | 0 |
| **Adım 22 cancel** | 149/131 | 116/90 | 96/72 | 145/133 | 121/90 | 101/75 | 0 |

**Dürüst gerileme:** İleri itkilerde kalça ortalamada 10–24 px daha derine
çöküyor. Sebep ölçüldü: Faz B yakalaması bacağı 3 karede 70 px taşıyor.
Kütleli bacakta bu, kalçada 36 birimlik tepki kuvveti demek (dikeyde ±6–12).
Bu, bacağın gerçek eylemsizliği. Kolsuz gövdede en kötü +500 durumu 50 px;
kollar bunu 71–74'e çıkarıyor. Hiçbir koşuda düşme, çift-havada kare veya
toparlanamama yok.

**Adım 21.4'ün açık sorunu (kollu çalkantılı toparlanma) tersine döndü.**
±150 px × 12 faz için en uzun kayma serisi (medyan/maks) ve itkiden 20–90
kare sonra en büyük |vx−2| (konum farkından):

| | kayma serisi | max \|vx−2\| |
|---|---|---|
| Adım 21 kolsuz | 8 / 99 | 1.9 |
| Adım 21 drive | 27 / 54 | 4.5 |
| Adım 22 kolsuz | 38 / 160 | 7.3 |
| Adım 22 drive | 17 / 50 | 1.3 |
| Adım 22 cancel | 20 / 48 | 1.5 |

Kütleli bacakta kollar toparlanmayı **sakinleştiriyor**. Kolsuz gövde daha
çalkantılı. Kaymaların çoğu buz bölgesinde: t=7 s itkisi kalçayı x≈400'de,
yani buzun içinde yakalıyor.

### 22.6 Kayma: artık sadece buzda, ama buzda daha çok
İtkisiz 30 saniyede buz dışında **0** kayma (eskiden 7–8), buzda 87–107
kayma karesi (eskiden 0). Kaymalar mikro düzeyde: hız 0.05–0.5 px/kare.
Fiziksel yorum: kütlesiz bacak buzda hiç sürtünme istemiyordu. Kütleli
bacakla COM hızı adım boyunca ±0.1 px/kare dalgalanıyor, denetleyici bunu
düzeltirken μ=0.15'lik buzu aşıyor. İnsanın normal yürüyüşte gereken
sürtünme katsayısı ~0.17–0.20, yani μ=0.15 buzda insan da kayar.

### 22.7 Gerileme ve kanaryalar
- 60 saniye varsayılan senaryo: NaN yok, düşme yok, 121 adım, vx 1.94, en
  alçak kalça 131, gövde yaw RMS 2.29°.
- 60 saniye düz yürüyüş: 123 adım, vx 1.90, en alçak kalça 182.
- Yeni kanaryalar: 25 adım, 0 acil, 122 kayma karesi (hepsi buzda), 2 Faz B,
  son `hip_y` 146.20.
- `LEGACY_21` bayrakları (`leg_mass=False, thrust_mode="hip", posture_k=0,
  posture_c=0, arms_mode="drive"`) Adım 21'i aynen veriyor (23/0/63/9/146.32).
- Testler 59/59 geçti (yeni `tests/test_step22_leg_mass.py`: momentum
  korunumu, kuvvet çifti, buz dışı kayma yok, FABRIK titreşimsiz yük, yaw
  iptali, ±150/300/500 itki). step1–16 smoke geçti.
- 19b/19c izolasyon testleri Adım 21 gövdesinde çalışıyor. **Bulgu:**
  Kütleli bacakta +150'de prediktif sensör çökmeyi küçültmüyor (130.8'e
  karşı 138.2). Sensör itkinin geldiği karede tetiklemeye devam ediyor, ama
  erken başlayan 3 karelik yakalama artık bedava değil.

### 22.8 Dürüst sınırlar
1. Bacak yörüngesi hâlâ kinematik. Tepki gövdeyi etkiliyor, bacağın
   hareketini etkilemiyor (ters dinamik, ileri dinamik değil).
2. Diz bükülmesinin momentum katkısı yok (pergel ekseni).
3. Kalça ivmesinin moment terimi `Σ r_i × m_i(1−f_i) a_kalça` atlandı (açık
   döngü kararsızlığı riski). Normal yürüyüşte <%5.
4. Yaw tek yönlü 2.5B bir defter: burulma sagittal fiziğe geri dönmüyor.
   I, k ve c seçilmiş parametreler.
5. `TUNED_GRAVITY` (0.065 px/kare²) gerçek ölçeğin ~1/34'ü (184 px = 0.9 m,
   30 fps için g ≈ 2.2 px/kare²). Bacak ivmeleri gerçek ölçekte makul
   (~1 g), ama gövdenin yerçekimi altında düşüşü hâlâ ağır çekim. Yerçekimini
   gerçek ölçeğe çekmek bütün motoru yeniden ayarlamak demek; ayrı iş.
6. `hip_vx_log`, `prev_points`'e işlenen tepki dürtüsünü de içeriyor, bu
   yüzden gürültülü (std 1.19). Denetleyici COM hızını okuyor (std 0.10).
   Gerçek kalça hızı (konum farkı) adım içinde 1.3–2.5 px/kare arasında
   (±%30). İnsanda bu oran ±%15–20.

Varsayılanlar: `LEG_MASS_ENABLED=True`, `LEG_MASS_COM_CONTROL=True`,
`THRUST_MODE="com"`, `POSTURE_K=0.2`, `POSTURE_C=0.8`, `ARMS_MODE="cancel"`.

## Adım 23 — Yakalama adımının süresi kalça torkundan

Adım 22'de kütleli bacaklar geldi, ama Faz B yakalaması hâlâ kütlesiz
günlerin sabit zamanlayıcısıyla çalışıyordu: her yakalama 3 karede (100 ms)
bitiyordu, gereken kuvvet ne olursa olsun.

### 23.1 Ölçüm: sabit zamanlayıcı insanüstü tork istiyordu
Tork birimi: toplam kütle 5.17 birim = 70 kg, 184 px = 0.9 m, 30 fps →
1 birim ≈ **0.291 Nm**. Ters dinamik kalça torku (`physics/leg_mass.py`):

| | kalça torku |
|---|---|
| Normal yürüyüş p99 | 122 birim ≈ **35 Nm** (insan: ~0.5–1 Nm/kg → 35–70 Nm) |
| Sabit yakalama, 1 m/s itki | **358 Nm** |
| Sabit yakalama, 2 m/s itki | **465 Nm** |
| Sabit yakalama, 150 px test itkisi | 686 Nm |
| Sabit yakalama, 500 px test itkisi | 921 Nm |
| Sağlıklı yetişkin kalça fleksör tepe torku | ~140 Nm (~2 Nm/kg) |

Normal yürüyüş gerçekçi. Yakalama, insan kalçasının 2.5–6.5 katı tork
istiyordu.

**Test itkilerinin ölçeği:** 150 px kalça itkisi, tüm-gövde COM'una
**~4.2 m/s** hız değişimi veriyor (500 px ≈ 14 m/s). Kalça tek karede 71 px
(500'de 145 px) sıçrıyor. Bunlar biyomekanik değil, sayısal stres testleri.
İnsan tek adımla ~1–1.5 m/s'lik bir itkiyi toparlayabiliyor, daha büyüğünde
çok adım gerekiyor. Bu yüzden m/s cinsinden gerçekçi bir itki takımı eklendi
(`KICK_PER_MS = 5.18 · 184/0.9/30 ≈ 35.3 px` = 1 m/s).

### 23.2 Tork sınırlı ayak servosu (`CATCH_TIMING="torque"`, varsayılan)
İlk deneme: "planlanan Bézier yolunun torku sınırı aşmayacak en kısa süre".
**Çalışmadı.** Plan anında tork 480 altındaydı, gerçekleşen tork 3000–4500
oldu. Sebep: yakalama hedefi her kare kalçanın peşinden kayıyor, ikinci
dereceden Bézier'de hedefin kayması ayağı tek karede `t²·Δ` kadar
ışınlıyordu (+150'de ayak 418 → 519, 100 px/kare). Plan anındaki sınır
gerçeği bağlamıyor.

Yerine yakalamada ayak, ivmesi kalça torkuyla sınırlı bir servo oldu:

- `|a| ≤ (τ_max − |τ_yerçekimi|) / (J · |kalça→ayak|)`, burada
  `J = Σ m_i f_i² = 0.198` (pergel ekseni, Adım 22 ile aynı model).
- Yatayda zaman-optimal (bang-bang) frenleme eğrisi kullanılıyor (ayrık
  zamanda: `v = √(a²/4 + 2a|e|) − a/2`). Dikeyde 14 px kalkış, hedefe
  yaklaşınca iniş. Hedefe ≤3 px ve yerdeyse basıyor. 24 karede varamazsa
  olduğu yere basıyor.
- Hedef hâlâ kalçanın peşinden kayıyor (Faz B mantığı aynı), ama ayak artık
  sıçrayamıyor; hedef kaçarsa süre uzuyor.
- `HIP_TORQUE_MAX = 480` birim ≈ 140 Nm.

Gerçekleşen tepe tork ±1/±2 m/s itkilerde 445–480 birim (139–140 Nm).
Sınır tutuluyor (test). Yakalama süresi artık sonuç: 1 m/s'de 4 kare
(133 ms), 2 m/s'de 9 kare (300 ms), geri itkilerde 5–10 kare. İnsan koruyucu
adımı ~150–400 ms.

### 23.3 Sonuçlar
**Gerçekçi itkiler** (6 faz, en alçak kalça ort/en kötü, px; ayakta 184):

| COM Δv | +0.5 | −0.5 | +1.0 | −1.0 | +1.5 | −1.5 | +2.0 | −2.0 | +2.5 | −2.5 |
|---|---|---|---|---|---|---|---|---|---|---|
| sabit 3 kare (358–465 Nm) | 179/178 | 172/167 | 173/168 | 155/139 | 169/164 | 151/149 | 169/166 | 147/129 | 164/155 | 144/137 |
| **tork sınırlı (140 Nm)** | 180/178 | 174/158 | 172/166 | 148/127 | 158/148 | 134/130 | 139/110 | 141/125 | 121/104 | 127/119 |

Hiçbir koşuda düşme yok. 1 m/s'ye kadar fark gürültü düzeyinde. 2–2.5
m/s'de insan torkuyla daha derin çöküş var (+2.5'te 164 → 121): ağır bacağı
öne taşımak zaman alıyor ve COM bu sürede düşmeye devam ediyor.

**Sayısal stres itkileri** (±150/300/500 px, 36 koşu): tork sınırlıyken
**14 düşme var** (+500 ve −500'ün hepsi, +300'ün bir kısmı). ±150 (4.2 m/s)
ayakta kalıyor: en alçak kalça 85/78 ve 79/71. Bu sayısal itkilerin sabit
zamanlayıcıyla "kurtarılması", 600–900 Nm'lik bir kalça sayesindeydi. Bu
testler artık `catch_timing="fixed"` ile Adım 22 davranışını ölçüyor.

**Varsayılan senaryo** (150 px itki): ayakta, 22 adım, 6 Faz B, yakalama
süreleri 6/12/5/6 kare, en alçak kalça 79 px (Adım 22: 131). Görselde derin
bir öne hamle var (render 212–216).

### 23.4 Yan bulgular
- **Diz dalı koruması:** Yavaş yakalama sırasında stance bacağı kalçanın
  ~130 px gerisinde, ~45°'de kalabiliyor. Bu durumda FABRIK dizi kalça→ayak
  çizgisinin ters tarafında bıraktı (kare 217, −10.7 px). İki kemikli
  zincirde diz bu çizgiye göre aynalanınca kemik boyları aynen korunuyor
  (`_guard_knee_branch`). Zincir fiziğe geri beslenmiyor.
- **Kollar büyük itkide başın üstüne çıkıyor:** `cancel` kolları 1 m/s'de en
  fazla 49° salınıyor. 2 m/s'de yakalama adımının büyük yaw momentumunu
  karşılamak için 7–8 kare omuz konisinin sınırında (100°), 150 px'te 20 kare
  kalıyor. Koruyucu kol kaldırma refleksine benziyor, ama sınıra dayanmak
  momentum iptalinin doyduğu anlamına geliyor.
- **Sıfır karelik "yakalama":** Hedef zaten ayağın 3 px yakınındaysa servo
  aynı karede basıyor (yerinde adım). Zararsız, ama `catch_frames_log`'da
  0 olarak görünüyor.
- Eski acil adım yolu (`trigger_emergency_step`, 4 kare) hâlâ sabit
  zamanlı. Faz B açıkken itki testlerinde tetiklenmedi.

### 23.5 Gerileme ve kanaryalar
- İtkisiz yürüyüşte yakalama tetiklenmiyor. Tork modu sabit modla **bit-bit
  aynı** (test).
- Yeni kanaryalar: 22 adım, 0 acil, 157 kayma karesi, 6 Faz B, son `hip_y`
  145.97. `catch_timing="fixed"` Adım 22'yi aynen veriyor
  (25/0/122/2/146.20), `LEGACY_21` Adım 21'i aynen veriyor.
- Testler 65/65 geçti (yeni `tests/test_step23_torque_catch.py`: frenleme
  eğrisi, tork sınırı, büyük itkide uzun yakalama, ±0.5…2.5 m/s toparlanma,
  itkisiz eşitlik). step1–16 smoke geçti.

### 23.6 Dürüst sınırlar
1. Tork sınırı yalnızca kalça fleksiyon/ekstansiyon torku. Bacak boyunca
   (radyal, diz) ivme ayrı sınırlanmıyor, toplam ivme vektörü kalça
   sınırına göre kısılıyor (tutucu).
2. 140 Nm sabit. Gerçek kas torku açısal hıza bağlı (kuvvet–hız eğrisi),
   hızlı hareketlerde daha düşük. Bu model iyimser.
3. Kalça ivmesinin moment terimi hâlâ dışarıda (Adım 22.8/3). İtki karesinde
   kalça 35–145 px sıçradığı için o karelerdeki tork tahmini kaba.
4. Prediktif sensör ve şok emilimi parametreleri sabit zamanlayıcıya göre
   ayarlanmıştı, yeniden ayarlanmadı.

Varsayılanlar: `CATCH_TIMING="torque"`, `HIP_TORQUE_MAX=480`
(`ActiveBipedSim(catch_timing=..., hip_torque_max=...)` ile değiştirilebilir).

## Adım 24 — Temas tabanlı şok servosu, kapanma hızı (TTC) kapısı

### 24.1 Önce teşhis (kod okundu, ölçüldü)
- Adım 19b şok emicisi 3 karelik bir zamanlayıcıya bağlı **değildi**. Yakalama
  inişinde, anchor o bacağa geçtiği karede olay tetikli başlıyordu. Servo
  yakalamalarında da devreye giriyordu.
- Adım 19c prediktif sensörü iniş zamanını tahmin **etmiyor**. Duruş
  bacağının bir sonraki karedeki açısını tahmin edip Faz B'yi tetikliyor.
- Asıl sorunlar ölçümde başka çıktı:
  1. **Yay fırlaması.** Eski emici dinlenme boyunu karede 15 px uzatıyordu
     (≈2.2 m/s). Kalça 90 px'in altına inince sönümlemeyi tamamen bırakıyordu
     (`SHOCK_FLOOR_PX`). 150 px itkide kalça 79 px'e iniyor, kare 217'de tek
     karede 49 px fırlıyordu. Gerçekçi itkilerde (1–2.5 m/s) en büyük yükseliş
     17–21 px/kare, 4.2 m/s'de 50–69 px/kare.
  2. **Gereksiz yakalamalar.** Yakalama ayağı bilerek kalçanın önüne
     basıyor. COM o ayağa 3–5 karede varacakken tehlike yolu (COM destek
     aralığının gerisinde + heel_strike) aynı bacağı yeniden fırlatıyordu
     (2 m/s itki, kare 220–221).
- **Kendi ölçüm hatam:** Önce "yakalamaların yarısı 0 karelik" demiştim. Bu
  sayı şişikti. Bacağın kendi içinden tetiklenen yakalamalar (`update()`
  içindeki toe_off fırlatması) olayın başlangıç karesini kaydetmiyordu ve 0
  olarak loglanıyordu. Düzeltildi: torque modunda süre servonun kendi kare
  sayacından okunuyor. Gerçek 0 karelik yakalama yok. Buna karşı eklediğim
  "önemsiz hedef" koruması ölçümde hiçbir şeyi değiştirmedi ve **kaldırıldı**.

### 24.2 Şok servosu (`SHOCK_MODE="servo"`, `SHOCK_TRIGGER="contact"`)
- **Tetik:** Artık sadece Faz B yakalamasında değil, **her inişte**.
  Temas anındaki sıkışma (`ARM_LENGTH − |kalça−ayak|`) 8 px'i aşarsa
  devreye giriyor. Normal yürüyüşte iniş sıkışması ≤ 1.3 px (60 s, 121
  iniş). İtkisiz yürüyüş Adım 23 ile **bit-bit aynı** (test).
- **Uzatma:** Anchor–kalça çubuğunun dinlenme boyu, hız ve ivmesi sınırlı
  bir servoyla `ARM_LENGTH`'e uzuyor. Sınırlar 3 px/kare (≈0.44 m/s) ve
  1 px/kare² (diz ekstansörlerinin sınırlı kuvveti). Boy hiçbir zaman
  mevcut boydan kısa değil, yani destek gevşemiyor. 90 px taban kuralı
  kaldırıldı.
- **Tarama** (6 faz, gerçekçi itkiler): 0.3–2 px/kare² × 3–8 px/kare
  denendi. Seçilen 1/3, en büyük yükselişi her itkide ≤10 px/kare tutuyor.

### 24.3 Kapanma hızı kapısı (`FAZB_CLOSING_TTC_FRAMES = 8`)
`predict_contact` artık COM hızıyla kalçanın duruş ayağına kapanma süresini
de hesaplıyor: `TTC = |ayak − kalça| / kapanma hızı`. Kapanma hızı yalnızca
kalça ayağa doğru gidiyorsa pozitif. TTC ≤ 8 kare ise COM o ayağa zaten
varacak demektir. Bu durumda ne açı aşımı (toe/heel) ne de step14'teki
tehlike yolu adım attırıyor, eski acil adım da tetiklenmiyor. Tarama: 4 kare
etkisiz, 8 ve 12 aynı.

### 24.4 Yol üstünde bulunan Adım 23 hatası
Duruş ayağı kayarken (`apply_slip`) yakalama servoya **sıfır hız ve
kaymış `planted`** konumundan başlıyordu. Bu konum/hız sıçraması sınırsız bir
ivme demek: −1 m/s itkide kalça torku **582 birim** (sınır 480). Artık servo
son çizilen ayak konumundan, son hızıyla başlıyor. Gerçekleşen tepe tork
±1/2/2.5 m/s'de 470–479 birim. Bu düzeltme Adım 23 kanaryasını da değiştirdi
(22/0/157/6/145.97 → 22/0/165/7/145.99).

### 24.5 Sonuçlar
6 faz × 8 itki = 48 koşu. Hücreler: en alçak kalça ortalama / en kötü /
tek karede en büyük yükseliş (px).

| | +1.0 | −1.0 | +2.0 | −2.0 | +2.5 | −2.5 | +4.2 (150 px) | −4.2 | yakalama |
|---|---|---|---|---|---|---|---|---|---|
| Adım 23 | 172/166/7 | 150/133/17 | 139/110/17 | 142/128/17 | 121/104/18 | 128/111/21 | 85/78/**52** | 82/71/**60** | 181 |
| yalnız şok servosu | 172/166/4 | 155/136/5 | 136/104/5 | 139/120/5 | 116/94/5 | 122/107/7 | 72/60/8 | 73/67/7 | 190 |
| yalnız TTC kapısı | 172/166/7 | 150/133/17 | 139/110/17 | 141/130/18 | 124/104/20 | 129/111/21 | 87/77/50 | 81/61/69 | 126 |
| **Adım 24 (ikisi)** | 172/166/**4** | 155/136/5 | 137/106/5 | 139/128/6 | 121/98/6 | 127/107/7 | 80/69/**9** | 75/57/7 | **137** |
| Adım 24, prediktif sensör yok | 172/166/4 | 155/136/5 | 136/115/5 | 136/120/5 | 118/99/6 | 120/103/6 | 72/53/8 | 73/64/10 | 130 |

- Hiçbir koşuda düşme ya da çift-havada kare yok.
- Kalçanın en büyük yükselişi 17–60 px/kareden **4–10 px/kareye** indi.
- Yakalama sayısı %24 azaldı.
- En alçak kalça ortalamada aynı mertebede kaldı. Bedel: 4.2 m/s'de daha
  derin (85 → 80 ortalama, 78 → 69 en kötü). Yaylanma artık kalçayı yukarı
  fırlatmıyor, yavaş kalkılıyor.
- Prediktif sensör hâlâ küçük bir katkı veriyor, açık kaldı.
- **Sayısal stres itkisi (±300 px ≈ 8.5 m/s):** Adım 24 ve `PRE24` (eski
  şok emici, kapı yok) aynı sonucu veriyor: 12 koşunun 7'si düşüyor. Servo
  başlangıç düzeltmesinden (24.4) önce eski şok emiciyle 2 düşme vardı. O
  "kurtarma", sınırsız ivmeli başlangıç sıçramasından geliyordu.
- **Görsel:** 150 px itkide öne hamleden sonra derin bir çömelme var
  (render 220–241). Karakter yaklaşık 1 saniyede doğruluyor. Kollar artık
  başın üstüne çıkmıyor.

### 24.6 Gerileme
- 60 s varsayılan senaryo: NaN yok, düşme yok, 121 adım, vx 1.96, en alçak
  kalça 71, 1 Faz B, yakalama süreleri 7/9/4 kare.
- 60 s itkisiz yürüyüş: Adım 23 ile bit-bit aynı.
- Kanarya: 25/0/104/1/145.66. `PRE24` bayrakları Adım 23'ü (servo
  düzeltmesi dahil) veriyor. `FIXED_CATCH` Adım 22'yi, `LEGACY_21` Adım 21'i
  aynen veriyor.
- Testler 71/71 geçti (yeni `tests/test_step24_contact_shock.py`). step1–16
  smoke geçti.

### 24.7 Dürüst sınırlar
1. Şok servosu bir kinematik dinlenme boyu servosu. Diz ekstansör kuvveti
   ters dinamikle hesaplanmıyor, 1 px/kare² ve 3 px/kare seçilmiş değerler.
2. TTC kapısı COM hızının sabit kalacağını varsayıyor (ivmeyi hesaba
   katmıyor).
3. İtki karesinde kalça 35–150 px sıçradığı için o karedeki tork tahmini
   kaba (tepe 536 birim, itki karesinde; Adım 23.6/3).
4. Hill kuvvet–hız eğrisi ve kol doygunluğu açık kaldı.

## Adım 25 — İnişe hazırlık (pre-activation)

### 25.1 Ayağın yere değme süresi (`time_to_contact`)
Servo kinematiği saf bir fonksiyona ayrıldı (`_servo_kin`; kanarya
bit-bit aynı kaldı). `time_to_contact` bu kinematiği servo durumunun bir
kopyası üzerinde ileri koşarak ayağın kaç karede basacağını tahmin ediyor.
Hedef ve ivme sınırı sabit varsayılıyor. Normal Bézier salınımında süre,
kalan salınım kareleri.

Doğruluk (150 px itki, tahmin ≤3 kare iken): 9 tahminin 5'i tam isabet,
2'si bir kare erken, 1'i bir kare geç. Bir tahmin 3 kare saptı: hedef kalçanın
peşinden kaydı.

### 25.2 Hazırlık refleksi (`PREACT_FRAMES = 3`)
Salınımdaki ayağın kalan süresi ≤3 kareye (≤100 ms) inince bacak
ekstansörleri temastan **önce** kasılmaya başlıyor. Hazırlık hızı her kare
`SHOCK_EXT_ACCEL` (1 px/kare²) artıyor, tavanı `SHOCK_EXT_VMAX` (3 px/kare).
Temas şok servosunu tetiklerse (Adım 24) servo sıfırdan değil, bu hızla
başlıyor. Normal yürüyüşte şok tetiklenmediği için itkisiz yürüyüş bit-bit
aynı (test).

### 25.3 Sonuç: etki küçük, çünkü çömelmenin çoğu temastan önce
Tarama: 0–4 kare hazırlık, 6 faz × 8 itki. Hücreler: en alçak kalça
ortalama / en kötü, toparlanma süresi (en alçaktan 170 px'e, kare), temas
sonrası ek çökme (px).

| itki | hazırlıksız (Adım 24) | 3 kare hazırlık |
|---|---|---|
| +2.0 m/s | 137/106, 10 kare, 2.2 | 138/107, 9 kare, 1.4 |
| −2.0 m/s | 139/128, 15 kare, 1.0 | 140/129, 13 kare, 0.5 |
| +2.5 m/s | 121/98, 14 kare, 3.2 | 122/99, 14 kare, 2.5 |
| −2.5 m/s | 127/107, 20 kare, 2.3 | 127/108, 19 kare, 1.6 |
| +4.2 m/s (150 px) | 80/69, 28 kare, 7.4 | 81/70, 25 kare, 6.4 |
| −4.2 m/s | 75/57, 38 kare, 5.9 | 76/58, 33 kare, 4.9 |

- Hiçbir koşuda düşme ya da çift-havada kare yok. En büyük yükseliş 9 px/kare.
- 4 kare hazırlık 3 kareyle aynı sonucu veriyor.
- Temas sonrası ek çökme ~%15–50, toparlanma süresi 1–5 kare azaldı.
- En alçak kalça neredeyse hiç değişmedi (+1 px).

**Sebep ölçüldü:** Çökmenin %93–95'i ayak yere değmeden **önce**
gerçekleşiyor:

| itki | 184'ten temasa | temastan en alçağa |
|---|---|---|
| 150 px | 105 px | 8 px |
| +2.5 m/s | 80 px | 6 px |
| −2.5 m/s | 73 px | 4 px |

Yakalama salınımı süresince (150 px'te 6 kare) eski duruş bacağı ters
sarkaç gibi devriliyor ve kalça kare başına ~17 px düşüyor. Hazırlık
refleksi temas sonrasını iyileştirebilir; temastan önceki düşüşü
değiştiremez.

**Toparlanma süresi bir parametre seçimi:** Kalça şok servosunun hız
tavanıyla (3 px/kare ≈ 0.44 m/s) kalkıyor; 71 → 170 px yaklaşık 33 kare.
Çömelmeden kalkış hızı ancak diz ekstansör kuvveti modellenirse fizikten
gelir (Hill modeli ile birlikte açık iş).

### 25.4 Gerileme
- Kanarya: 25/0/104/1/145.53 (Adım 24: 145.66).
- 60 s varsayılan senaryo ayakta, 121 adım, en alçak kalça 72. Temastaki
  hazırlık hızları 2/3/3/3 px/kare.
- `PRE24` (artık `preactivation=0` dahil) Adım 23'ü veriyor. Diğer eski
  bayrak setleri de değişmedi.
- Testler 75/75 geçti (yeni `tests/test_step25_preactivation.py`). step1–16
  smoke geçti.

### 25.5 Sıradaki gerçek açık
Derin çömelmenin kaynağı yakalama süresi boyunca duruş bacağının
devrilmesi. Bunu küçültmenin fiziksel yolları:
- Daha hızlı yakalama: Hill modeli yakalamayı tersine **yavaşlatır**.
- Duruş bacağında ayak bileği/kalça stratejisi: COP kaydırma, gövde
  eğme.
- Diz ekstansör kuvvetini ters dinamikle hesaplayıp şok servosunun hız
  tavanını fiziğe bağlamak.

## Adım 26 — Duruş bacağı: ayak rocker'ı ve kalça stratejisi

### 26.1 Kalça neden düşüyordu: yerçekimi değil, geometri ve çeken bacak
150 px itkide kalça yakalama sırasında kare başına ~17 px iniyor. Ayarlı
yerçekimi (0.065 px/kare²) altı karede bunun küçük bir kesrini üretebilir.
Düşüşün kaynağı geometri: kalça yatayda ~15 px/kare ilerliyor, duruş bacağı
ise ayak bileğine sabit, 184 px'lik bir çubuk. Kalça ayağın dx önüne
geçtikçe `h = √(L² − dx²)` yayına iniyor.

Ölçüm: kare 210–214'te bu çubuk **11–22 px gerilmede**. Yani bacak kalçayı
zemine doğru **çekiyor**. Gerçek bir bacak zemini iter, çekemez. Normal
yürüyüşte gerilme ≤ 0.6 px.

**Denenip reddedilen: tek yönlü (yalnız basınç) bacak.** İtkisiz
yürüyüşte bile gövde 574 karede bacaktan ayrılıp süzüldü, adım sayısı
61 → 42'ye düştü, kalça 150 px itkide 237 px'e yükseldi. Sebep, ayarlı
yerçekiminin gerçek ölçeğin ~1/34'ü olması: gövdeyi bacağın üzerinde
gerçekte yerçekimi tutar, bu motorda iki yönlü çubuk tutuyor. Gerçek
yerçekimiyle (2.2 px/kare²) 15 px/kare'de gereken merkezcil ivme v²/L =
1.2 < g olurdu ve bacak zaten basınçta kalırdı. Yani gerçekte de kalça
yaya iner; düşüşü azaltacak şey pivotu değiştirmek.

**Ayak bileği torkuyla frenleme (COP kaydırma) yetmez:** Yapılabilecek
en büyük fren, ağırlık × ayak uzunluğu / kalça yüksekliği. Bu, gerçek
ölçekte bile ~0.3 px/kare². 6 karede 15 px/kare'lik hızın ~2 px/kare'sini
alır. Ayak bileği stratejisi küçük itkiler içindir.

### 26.2 Ayak rocker'ı (büyük hareketteki ayak bileği stratejisi)
Gerçek ayakta topuk kalkar ve gövde parmak ucu üzerinden yuvarlanır
(forefoot rocker, plantar fleksiyon); geri tarafta topuk üzerinden yuvarlanır
(heel rocker). Model: kalça normal yürüyüşün hiç ulaşmadığı bir mesafeyi
geçince duruş pivotu kalçayla birlikte kayıyor. Normal yürüyüşte kalça–pivot
mesafesi −18.4…+29.4 px arasında.

| | eşik | en büyük kayma |
|---|---|---|
| Öne (parmak ucu) | +35 px | 40 px |
| Geriye (topuk) | −25 px | 13 px |

Ayak 0.26 m ≈ 53 px; bilekten parmak ucuna ~40 px, topuğa ~13 px.

- **Gereken plantar fleksör torku:** En büyük kayma (40 px) gerçek ölçekte
  70 kg × 9.81 × 0.196 m ≈ **134 Nm**. İnsan kapasitesi ~150–250 Nm,
  yani karşılanabilir.
- **Görsel:** Render'daki Faz A ayakkabı eğimi (topuk kalkışı) bununla
  tutarlı (kare 211).
- **Normal yürüyüş:** Rocker yalnızca başlangıç sarsıntısında (kare
  2–11) devreye giriyor. Bu yüzden itkisiz yürüyüş bit-bit aynı değil, ama
  ölçütler aynı: 122 ve 123 adım, vx 1.90, eğim 3.5° ± 1.1 ve 3.2° ± 0.9.

### 26.3 Kalça stratejisi (`HIP_STRATEGY_GAIN = 0.3` derece/px, tavan 9°)
COM destek aralığının dışına çıkınca gövde duruş hedefi kayıyor. Kuvvet
çifti gövdeyi döndürürken tepkisi kalçayı ters yöne itiyor. İşaret
ölçümle seçildi: COM öndeyken gövde **öne** eğiliyor ve kalça geri
itiliyor. Ters işaret her senaryoda daha kötü (−0.3'te +2.5 m/s en kötü:
74 px).

### 26.4 Sonuç
6 faz × 8 itki = 48 koşu. Hücreler: en alçak kalça ortalama / en kötü (px).

| | +1.0 | −1.0 | +2.0 | −2.0 | +2.5 | −2.5 | +4.2 | −4.2 |
|---|---|---|---|---|---|---|---|---|
| Adım 25 (bilek pivotu) | 172/166 | 155/137 | 138/107 | 140/129 | 122/99 | 127/108 | 81/70 | 76/58 |
| yalnız rocker | 179/177 | 165/147 | 163/155 | 145/135 | 140/116 | 126/107 | 89/84 | 86/60 |
| **rocker + kalça stratejisi** | **178/175** | **160/146** | **165/153** | **150/136** | **149/127** | **136/120** | **92/74** | **93/73** |

- Hiçbir koşuda düşme, toparlanamama ya da çift-havada kare yok.
- +1 m/s itkide 6 fazın 4'ünde yakalama adımı hiç gerekmiyor; rocker
  karşılıyor. −1 m/s'de 6 fazın 5'inde hâlâ 2–3 yakalama var (topuk
  rocker'ı yalnızca 13 px).
- Toparlanma süresi (en alçaktan 170 px'e) +2 m/s'de 9 → 2 kare,
  +4.2 m/s'de 25 → 22 kare.
- ±300 px sayısal stres itkisinde düşme 12'de 7 → 2.
- Varsayılan senaryonun en alçak kalçası 72 → 100.

### 26.5 Gerileme
- Kanarya: 25/0/89/1/145.95.
- 60 s varsayılan senaryo ayakta, 122 adım, vx 2.03.
- Testler 79/79 geçti (yeni `tests/test_step26_stance_strategy.py`).
  `PRE24` bayrakları artık `rocker=False, hip_strategy_gain=0.0` da
  içeriyor. Adım 23 servo-zamanlama testi Adım 25 gövdesinde ölçülüyor,
  çünkü 1 m/s'de artık yakalama yok. step1–16 smoke geçti.

### 26.6 Adım 27 (torka dayalı kalkış) için engel: yerçekimi ölçeği
Şok servosunun 3 px/kare tavanını diz ekstansör torkuna bağlamak için
bacağın üretebildiği dikey kuvvet gerekiyor. İki kemikli bacakta diz torku
τ ile kalça–ayak doğrultusundaki kuvvet `F = τ / (l · sin β)`, burada
`cos β = d / 2l`. Tek bacak, gerçek ağırlık (70 kg) için bacağın vücudu
taşıyabildiği en kısa kalça–ayak mesafesi:

| diz torku | taşıyabildiği en kısa mesafe |
|---|---|
| 200 Nm | 140 px |
| 250 Nm | 107 px |
| 300 Nm | 41 px |

Ölçülen en kısa kalça–pivot mesafeleri:

| itki | ort. | en kötü | o anda iki ayak yerde |
|---|---|---|---|
| ±2 m/s | 153–168 px | 137 px | — |
| ±2.5 m/s | 139–151 px | 123 px | — |
| ±4.2 m/s | 95 px | 73 px | geri itkilerde 5/6 |

Bunu sadeleştirmeden uygulamak üç karar istiyor:
1. Gerçek ağırlık mı, ayarlı yerçekimi mi? Ayarlı yerçekimiyle ağırlık
   34 kat küçük; kalkış ~12 karede fırlatma olur. Gerçek ağırlıkla
   200 Nm'lik tek bacak 140 px'in altında çöker.
2. Destek bacağı rijit bir çubuk; sonsuz yük taşıyor. "Kaldıramıyorsa
   çöker" için bacak kuvvet sınırlı bir elemana dönmeli.
3. İki ayak yerdeyken yük paylaşımı modelde yok, tek anchor var.

## Adım 27 — Kuvvet sınırlı bacak: gerçek ağırlığa karşı diz torku

Karar: motorun Verlet yerçekimi (`TUNED_GRAVITY`) değişmedi. Bacağın
taşıyabildiği yük ise **gerçek ağırlıkla** karşılaştırılıyor (70 kg,
g = 9.81 m/s² ≈ 2.23 px/kare²). Bacak ivmeleri zaten gerçek zaman
ölçeğinde (Adım 22.8/5).

### 27.1 Model (`SHOCK_MODE = "force"`)
Şok emilimi sırasında (temas sıkışması > 8 px) anchor–kalça çubuğunun
dinlenme boyu r artık bir hız/ivme tavanıyla değil, kuvvetle hareket ediyor.

- **Bacak kapasitesi:** İki kemikli bacakta diz torku τ, kalça–ayak
  doğrultusunda `F = τ / (l · sin β)` kuvveti verir (`cos β = d / 2l`). Dik
  bacakta F sonsuza gittiği için ağırlığın 6 katıyla sınırlandı.
  `TAU_KNEE_MAX_NM = 200` (tek bacak, ~2.9 Nm/kg).
- **Bacak ivmesi:** `a = (Σ F − M·g_gerçek) / M`. İki ayak da yerdeyse iki
  bacağın kuvveti toplanıyor.
- **Hareket:** Uzatma ivmesi en fazla `a`. Kas gevşerse gövde `g_gerçek`
  ile yavaşlıyor; frenleme eğrisi bu. a < 0 ise bacak ağırlığı taşıyamıyor
  ve zorla bükülüyor.
- **Temas anı:** r'nin hızı kalçanın bacak boyunca gerçek hızı. Yani iniş
  hızı artık rijit çubukla tek karede durmuyor, kuvvetle sönümleniyor.
- **Kas aktivasyonu:** Hazırlıksız temasta 0.2'den başlıyor, kare başına
  `(1 − act) · 0.57` artıyor (~40 ms). Adım 25'in hazırlık refleksi artık
  bu aktivasyonu temastan önce yükseltiyor.
- **Çökme:** Diz 150°'ye bükülünce (kalça–ayak 48 px) bacak çökmüş
  sayılıyor. Bu terminal bir durum: adım yok, gövde zeminin altına
  geçemiyor.
- 3 px/kare ve 1 px/kare² tavanları kaldırıldı (eski "servo" modu
  bayrakla duruyor).

Bacağın vücudu taşıyabildiği oran (F/W), kalça–ayak mesafesine göre:

| mesafe (px) | 184 | 170 | 160 | 140 | 120 | 100 | 60 |
|---|---|---|---|---|---|---|---|
| F/W | 6 (tavan) | 1.69 | 1.31 | **1.00** | 0.85 | 0.77 | 0.68 |

### 27.2 Sonuç
6 faz × 10 itki = 60 koşu. Hücreler: en alçak kalça ortalama / en kötü (px);
C = çöken koşu sayısı (6 fazda).

| | +1.0 | −1.0 | +1.5 | −1.5 | +2.0 | −2.0 | +2.5 | −2.5 | ±4.2 | çökme |
|---|---|---|---|---|---|---|---|---|---|---|
| Adım 26 (servo) | 178/175 | 160/146 | 177/174 | 154/143 | 165/153 | 150/136 | 149/127 | 136/120 | 92 / 93 | 0/60 |
| **200 Nm, hazırlıklı** | 178/175 | 159/146 | 177/174 | 141/82 | C2 | 117/76 | C6 | C2 | C6 / C6 | **22/60** |
| 250 Nm | 178/175 | 159/146 | 177/174 | 154/144 | C1 | 130/79 | C5 | C1 | C6 / C6 | 19/60 |
| 200 Nm, hazırlıksız | 178/175 | 158/144 | 177/174 | 141/108 | C5 | C3 | C6 | C5 | C6 / C6 | 31/60 |

- **±1.5 m/s'ye kadar** hiçbir koşu çökmüyor.
- **2–2.5 m/s'de** çökmeler başlıyor. Bu aralık, insanın tek adımla
  toparlanabildiği sınırın (~1.5–2 m/s) hemen üstü.
- **±4.2 m/s'nin (150 px) hepsi** çöküyor.
- **Hazırlık refleksi (Adım 25) artık belirleyici:** Kapatılınca çökme
  22'den 31'e çıkıyor. Adım 25'te sadece 1 px kazandırıyordu; kuvvet
  modelinde kas aktivasyonu fark yaratıyor.
- **Diz torkuna duyarlılık:** 200 → 250 Nm çökmeyi 22'den 19'a indiriyor.

### 27.3 Yan bulgu: aşırı hızlı kalkış (Hill eksikliği)
Dikleşen bacakta kapasite hızla büyüyor (170 px'te F/W 1.7, dik bacakta
tavan 6). Kalkış bazı koşularda **27 px/kare'ye** kadar ivmeleniyor. Örnek:
−2.5 m/s'de kalça 55 px'ten 193 px'e 8 karede çıkıyor ve dik boyu aşıyor,
yani kısa bir sıçrama. Kuvvet hızdan bağımsız olduğu için bu bekleniyor;
kuvvet–hız eğrisi (Adım 28) bunu sınırlamalı.

### 27.4 Gerileme
- İtkisiz yürüyüşte şok hiç tetiklenmiyor; Adım 26 ile bit-bit aynı (test).
  15 px tökezleme + 60 s yürüyüş ayakta.
- **Varsayılan sahne değişti:** 150 px itki (~4.2 m/s) artık kare 222'de
  çöküşle bitiyor. Kanarya 16/0/89/1, son `hip_y` 328.73 (kalça yerde).
  `shock_mode="servo"` Adım 26'yı aynen veriyor.
- Adım 23–26'nın itki testleri `shock_mode="servo"` ile kendi
  davranışlarını ölçüyor.
- Testler 85/85 geçti (yeni `tests/test_step27_force_limited_leg.py`).
  step1–16 smoke geçti.

### 27.5 Sınırlar
1. Kapasite yalnızca diz torkundan. Kalça ve ayak bileği ekstansörlerinin
   katkısı yok, yani model tutucu.
2. İki ayak yerdeyken diğer bacak fiziksel olarak bağlı değil. Yalnızca
   kuvvet kapasitesi toplanıyor.
3. Çöküş sonrası bir "yerde oturma" pozu yok. Yalnızca zemin kelepçesi
   var; render'da gövde yere yığılıyor.

## Adım 28 — Hill kas modeli (kuvvet–hız)

### 28.1 Model (`physics/hill.py`)
Kuvvet, eklemin açısal hızı s'ye göre ölçekleniyor (f(0) = 1):

| durum | formül | davranış |
|---|---|---|
| Konsantrik (kas kısalıyor, s ≥ 0) | `f = (1 − s/vmax) / (1 + s/(k·vmax))`, k = 0.25 | s = vmax'ta 0 |
| Eksantrik (kas zorla uzuyor) | van Soest–Bobbert formu | ~1.5'te platolanıyor |

`vmax = 0.4 rad/kare` (≈12 rad/s, insan diz/kalça üst sınırı).

Uygulandığı iki yer:
- **Diz ekstansörü (kuvvet sınırlı bacak, Adım 27):** Diz açısal hızı
  `ω = ṙ / (l · sin β)`. Bacak uzarken kuvvet düşüyor, zorla bükülürken
  artıyor.
- **Kalça torku (yakalama servosu, Adım 23):** Ayağın istenen ivme
  yönündeki hızı, kalça etrafında açısal hıza çevriliyor. Aynı yönde
  hareket konsantrik (tork düşer), frenleme eksantrik (tork 1.5 katına
  kadar çıkar). Gerçekleşen tepe tork ±1/±2 m/s'de 680–700 birim
  (eksantrik tavan 720). Hill kapalıyken tavan 480.

### 28.2 Adım 27'de bulunan hata: düz bacağın yük paylaşımı
Adım 27'de iki ayak yerdeyken diğer bacak, **düz olsa bile** kapasitesini
ekliyordu. Dik bacakta kapasite tavanı ağırlığın 6 katı; bu, tek karede
a = 18 px/kare² ve 15 px/kare'lik bir fırlatma üretti (−2.5 m/s, kare
244). Düz bacağın uzama payı olmadığı için kalçayı yukarı ivmelendiremez.
Artık diğer bacak yalnızca bükükse (sıkışma > 8 px) yük paylaşıyor.

**README 27.3'teki "27 px/kare'lik kalkış"ın çoğu Hill eksikliği değil, bu
hataydı.** Düzeltmeden sonra Hill kapalıyken en büyük yükseliş 12 px/kare,
çökme 22 → 29.

### 28.3 Sonuç
6 faz × 10 itki = 60 koşu. Hücreler: çökmeyen koşularda en alçak kalça
ortalama / en kötü (px); C = çöken koşu sayısı (6 fazda).

| | +1.0 | −1.0 | +1.5 | −1.5 | +2.0 | −2.0 | +2.5 | −2.5 | ±4.2 | çökme | en büyük yükseliş |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Adım 27 (düzeltilmiş), Hill yok | 178/175 | 157/136 | 177/174 | 142/82 C1 | 152/135 C2 | 133/111 C2 | C6 | C6 | C6/C6 | 29 | 12 px/kare |
| **Hill** | 178/176 | 158/144 | 175/168 | 150/127 | 155/145 C1 | 145/126 | 121/114 C4 | 86/67 C2 | C6/C6 | **19** | **6 px/kare** |
| Hill, hazırlıksız | 178/176 | 153/134 | 173/156 | 138/118 C1 | 127/122 C4 | 93/52 C1 | C6 | 69/69 C5 | C6/C6 | 29 | 7 px/kare |

- **Kalkış artık fizikten geliyor.** En büyük yükseliş 6 px/kare
  (≈0.9 m/s). Toparlanma medyanı 8 kare (en alçaktan 170 px'e).
- **Çökme 29'dan 19'a indi.** Eksantrik güç (iniş sırasında zorla
  bükülen kas ~1.5 kat kuvvet üretir) inişleri tutuyor.
- ±1.5 m/s'ye kadar çökme yok. 2–2.5 m/s'de 7/24, ±4.2 m/s'de 12/12.
- Hazırlık refleksi (Adım 25) hâlâ belirleyici: kapatılınca çökme 29.
- Yakalama süresi medyanı 7 → 5 kare. Eksantrik frenleme ayağı daha erken
  durduruyor.

### 28.4 Gerileme
- İtkisiz yürüyüş Adım 26 ile bit-bit aynı (Hill yalnızca şok ve yakalama
  sırasında çalışıyor).
- 15 px tökezleme + 60 s ayakta, 124 adım.
- Kanarya 16/0/92/1/327.21; varsayılan 150 px itki kare 222'de çöküyor.
- Önceki adımların testleri kendi davranışlarını ölçüyor:
  - Adım 23–26 testleri `hill=False` ile çalışıyor.
  - `PRE24` bayraklarına `hill=False` eklendi.
- Testler 90/90 geçti (yeni `tests/test_step28_hill.py`). step1–16 smoke
  geçti.

### 28.5 Açık: çöküş sonrası görüntü
Çöküş terminal bir durum, ama yere düşme pozu yok. Bacaklar donuyor,
kalça ilerliyor. Arkada kalan ayak kalçanın çok gerisinde kaldığı için deri
bacağı aşırı uzuyor (spagat görüntüsü). Kare 224'te bir bacak bir kare
zeminin altına geçiyor (render). Varsayılan sahnenin 150 px itkisi
(~4.2 m/s) artık bu görüntüyle bitiyor.

## Adım 29 — Çöküş sonrası yere yığılma (ragdoll geçişi)

Adım 27–28'de bacak ağırlığı taşıyamayınca çöküyordu, ama bir "yere
düşme" durumu yoktu. Bacaklar donuyor, kalça ilerliyordu. Deri spagat
gibi geriliyor, kare 224'te bir bacak zeminin altına geçiyordu.

### 29.1 Geçiş (`_enter_fallen`)
Çöküş anında (diz 150°, kalça–ayak 48 px):

- **Bacaklar fiziğe geçiyor:** Kinematik IK'dan çıkıp kütleli Verlet
  zincirlerine dönüyorlar (kalça–diz–ayak, 92 + 92 px). Diz noktası 0.45,
  ayak noktası 0.38 kütleli (bacak toplam 0.83). Başlangıç konumları son
  IK pozundan, hızları kalçadan alınıyor; yerdeki ayak duruyor.
- **Ayak yapıştırıcısı kapanıyor:** Anchor çubuğunun compliance'ı 1 oluyor.
  Kalçaya eklenen örtük bacak kütlesi kaldırılıyor; kütle artık ayrı
  noktalarda.
- **Yerçekimi gerçek ölçeğe geçiyor (2.23 px/kare²):** Adım 27'deki
  kararla aynı gerekçe; zaman ölçeği gerçek, düşüş de öyle.
- **Kaslar gevşiyor:** Duruş kuvvet çifti, kol sürüşü ve bacak kütlesi
  tepkisi kapanıyor. Gövde–kalça açısı serbest (179°), boyun ±60°.

### 29.2 Zemin, diz ve sürtünme (`_fallen_constraints`)
- **Zemin:** Esnek değil; çubuk çözücüsüyle aynı relaksasyon döngüsünde,
  8 iterasyon uygulanıyor. Temas yarıçapları (px): kalça 20, omuz 14,
  baş 16, diz 12, ayak 6, kol 5.
- **Sürtünme:** Kinetik sürtünme temas noktasının yatay hızını kare başına
  %60 sönümlüyor. Statik sürtünme, bu karede 0.6 px'den az kayan temas
  noktasını kare başındaki konumuna kilitliyor (konum tabanlı PBD). Böylece
  çubuk düzeltmeleri de onu kaydıramıyor.
- **Diz menteşesi (momentum korunuyor):** En fazla 150° bükülme, yani
  kalça–ayak ≥ 48 px; kütle ağırlıklı, yalnızca itme. Geri bükülme yok:
  diz kalça–ayak çizgisinin arkasına geçerse çizgiye geri konuyor.

### 29.3 Bulunup düzeltilen hatalar (ölçüldü)

| sorun | sebep | etki | çözüm |
|---|---|---|---|
| Zemin yalnızca sonda uygulanıyordu | çubuklar her kare noktaları zemine geri itiyordu | kare başına ~1 px "sürünme" | zemin relaksasyon döngüsüne alındı |
| `fabrik.clamp_joint_angle_points` | yalnızca uç noktayı oynatıyor (kütle ağırlıksız); 8 iterasyonda tekrarlanıyordu | 0.5–2 px/kare sürünme | yerine momentum koruyan diz menteşesi yazıldı |
| Düz bacakta diz aynalama | diz her kare ileri-geri atladı | diz hızı 40 px/kare | aynalama yerine çizgiye koyma |
| Duruş kuvvet çifti açık kaldı | ayarlı yerçekimine göre ayarlı tork, gerçek yerçekimine karşı + sürtünme | 0.4–0.5 px/kare sürünme | yığılmada gevşek gövde |
| Dar boyun sınırı (18°) | yerdeki baş ile boyun kelepçesi çekişiyordu | baş 45 px sıçradı | sınır ±60° |

### 29.4 Render (`demo/step17_bilge_physics_skin.py`)
- **Bacaklar:** Diz fizik dizinden, bilek incik yönünde görsel incik
  boyunda. Ayakkabı bileğe takılı ve incike dik; zemine gömülmüyor.
- **Uzun deri parçaları:** Deri parçaları fizik gövdesinden uzun (gövde
  86/55 px, baş 43/30, kol 54+50 / 34+32). Yerde yatarken aynı yön uzun
  parçayı zeminin altına sokuyordu. Parça, zemine değecek şekilde kökü
  etrafında yataya doğru döndürülüyor. Bu yalnızca görsel; fiziğe
  yazılmıyor.
- **Ölçüm:** Çöküş sonrası tüm render eklemleri (diz, bilek, kalça, göğüs,
  baş, dirsek, el) zemin çizgisinin üstünde; en düşükleri 6 px üstte.

### 29.5 Sonuç
- **Varsayılan sahne (150 px itki, ~4.2 m/s):** öne hamle (212–219),
  bacak kare 222'de çöküyor, dizler bükülüyor (224–229), gövde öne yere
  düşüyor (233), ~240'tan sonra yerde hareketsiz yatıyor. Spagat gerilmesi
  ve zemine girme yok.
- **Durgunluk:** 36 itki koşusunun (±2 / ±2.5 / ±4.2 m/s × 6 faz)
  19'u çöküyor; hepsi duruyor. Son 60 karede en hızlı nokta medyan 0.003,
  en fazla 0.010 px/kare. Kalça kayması < 1 px. NaN yok.
- **Çarpma anı:** Çöküş +0…+3 karede diz sınırı 5 px'e kadar aşılabiliyor,
  sonra toparlanıyor.

### 29.6 Gerileme
- Çöküş öncesi her şey Adım 28 ile aynı; durum yalnızca çöküşte açılıyor.
- Kanarya 16/0/92/1, son `hip_y` 277.33 (kalça katlanmış bacaklar
  üzerinde, yerden ~53 px).
- Testler 94/94 geçti (yeni `tests/test_step29_fallen.py`: zemin altına
  geçmeme, durgunluk, diz menteşesi ve kemik boyları, render'da göğüs/baş
  zemin üstünde). step1–16 smoke geçti.

### 29.7 Sınırlar
1. Yığılma pasif bir ragdoll. Kollarla kendini koruma (el uzatma) ya da
   kalkma davranışı yok; terminal durum.
2. Gövde tek çubuk (kalça–omuz–baş). Omurga bükülmesi ve kalça–gövde
   kas tonusu yok.
3. Render, uzun deri parçalarını zemin üstünde tutmak için kökleri
   etrafında döndürüyor. Fizik gövdesiyle deri arasındaki ölçek farkı (Adım
   17'den beri) burada görünür hale geliyor.

## Adım 30 — Koruyucu kol refleksi (bracing)

Adım 29'da karakter yere pasif bir ragdoll olarak düşüyordu: gövde kalas
gibi devriliyor, baş zemine 1.2–18.5 px/kare (0.2–2.7 m/s) ile çarpıyordu.
Bu adım düşüşte kolları zemine uzatan, gövdeyi kollar üzerinde tutan ve
sonra kontrollü indiren bir refleks ekliyor. Omurga hâlâ tek çubuk (bir
sonraki adım).

### 30.1 Tetik ve yön (`_update_brace`, `_brace_side`)
- **Tetik:** Kuvvet bacağı ağırlığı taşıyamıyor (Adım 27 kuvvet dengesi
  a < 0 ve bacak bükülüyor, 2 kare üst üste) ya da çöküş. Yürüyüşte ve
  toparlanan itkilerde (±1, ±2 m/s) hiç tetiklenmiyor (test).
- **Düşüş tarafı:** Omzun kalçaya göre konumu + 3 kare göreli hızı. İlk
  denemede mutlak omuz hızı kullanıldı. Geri itkide kalça geriye kaçarken
  gövde öne katlanıyor ve refleks eli kalçanın arkasına, omuz çizgisinin
  dışına uzatıyordu (ölçüldü).
- **Yeniden planlama:** Hiçbir el yük taşımıyorken gövde ters tarafa
  devriliyorsa (ellere sekip geriye oturma) kollar yeni tarafa uzanıyor.

### 30.2 Uzanma: dünyaya sabit hedef
El hedefi omza göre değil zemine göre: kalça + yön × 50 px (gövde 55 px).
Burası, gövde kalça etrafında devrilince omzun ineceği noktanın altı.
El zemine 25 px yaklaşınca hedef donuyor. Omza göre hedef (0.75 × kol boyu
önde) denendi: el omzu kovalıyor, zemine 11 px/kare yatay hızla çarpıyor,
kayıp kolu yataya yatırıyordu. Omuz yine 59 → 14 px'e düşüyordu
(ölçüldü). Yere değen avuç yapışıyor: yatay hız temas karesinde sıfır.

### 30.3 Kol kolonu (`_fallen_constraints`, `arm_column_capacity`)
- El yerdeyken omuz–el arasında yalnızca itebilen bir destek var; rest
  boyu temas anındaki boy. El–kol doğrusu sürtünme konisinin içindeyse
  (|dx| ≤ 0.8|dy|) zemin tepkiyi karşılıyor ve düzeltmenin tamamı omza
  gidiyor. Dışındaysa el kayıyor (kütle ağırlıklı).
- **Dirsek kapasitesi:** Triseps 60 Nm. Kuvvet τ/(l sin β), cos β = r/2l;
  eksantrikte Hill ≈1.5×. Kapasiteyi aşan sıkışma hızı kolu büküyor
  (rest boyu kısalıyor). Düz kol kemik kolonu gibi davranıyor: r = 65 px'te
  kapasite 36 px/kare, r = 40 px'te 7.9.
- **Ölçüm:** Taranan sahnelerde omuz kollara 8–10 px/kare (1.2–1.5 m/s)
  ile iniyor ve kollar neredeyse düz. Kapasite aşılmıyor, dirsek bükülme
  yolu hiç çalışmadı. Çarpmayı düz kol durduruyor.

### 30.4 Kontrollü indirme ve boyun tonusu
- **İndirme:** Omuz 15 kare boyunca 0.5 px/kare'den yavaşsa kolonlar
  0.6 px/kare kısalıyor (dirsek 140°'ye kadar eksantrik bükülme). Bitince
  refleks %10/kare sönüyor.
- **Boyun:** Refleks boyunca 18° (dik duruş sınırı); indirmede 0.7°/kare
  60°'ye gevşiyor. Sınır, kalça–baş arasında yalnızca itebilen en kısa
  mesafe olarak relaksasyon döngüsünde çözülüyor. Dışarıdaki
  `clamp_direction` ile 18° denendi: Adım 29'daki bulgunun aynısı, yerde
  baş 4 px/kare titriyor, eller zeminden sekiyordu (ölçüldü). Yerde
  60°'lik pasif sınır da artık döngüde.
- **Kol sürüşü:** Yığılmış gövdede refleks yürüyüş salınımına değil sıfır
  torka karışıyor. Yük alan kol sürülmüyor; dirsek tonus çifti de yok
  (kolon onu temsil ediyor). Önceki hali, refleks sönerken kolları
  yürüyüş pozuna sürüp dirsekleri 7 px/kare savuruyordu.

### 30.5 Bulunup düzeltilen hatalar (Adım 29'dan kalan, ölçüldü)

| sorun | etki | çözüm |
|---|---|---|
| ≥ ~5.6 m/s itkilerde kuvvet bacağı çöküşü tetiklemiyor (iki ayak havada) | kalça zeminin **184 px altına** iniyordu (Adım 29'da da; ±200/±260 px itkiler Adım 29 taramasında yoktu) | kalça, bacağın en katlanmış halinden (48 px) alçaktaysa yığılmaya giriliyor |
| Geç tetiklenen çöküşte FABRIK dizi zeminin 58–71 px altında ya da bacak > 150° katlı (kalça–ayak 30 px) | ilk karede kalça 70–150 px yukarı fırlıyordu | `_fallen_leg_init`: geçerli duruş aynen korunuyor; değilse yeni ayak noktası zemin boyunca uzaklaştırılıp diz IK ile doğru tarafta kuruluyor |
| Erişilemeyen salınım hedefi (kalça–ayak 215 > 184 px) | diz–ayak çubuğu 31 px uzun başlıyordu | ayak erişilebilir uca çekiliyor (varsayılan sahne de; kanarya değişti) |
| Sabit 0.6 px statik sürtünme eşiği | kol itişi oturan kalçayı her kare 2 px geri kaydırıyordu | eşik yüke bağlı: \|dx_t\| ≤ 0.8 × bu karenin normal düzeltmesi (PBD Coulomb) |
| Döngü çubuklarla bitiyordu | kilitli temas noktaları ~0.02 px/kare kayıyordu, gövde sürekli sürünüyordu (480 karede 1.4 px; yönü iterasyon sayısına göre değişiyordu) | statik kilit son kez çubuklardan sonra; relaksasyon 8 → 24 iterasyon. 600 karelik koşularda son 100 karede kayma 0.000 px |

### 30.6 Sonuç
24 çöküş sahnesi (±130/150/200/260 px itki × itki zamanı 6.6/7.0/7.4 s),
pasif (`brace=False`) ve refleksli. Ölçü: baş zemine 4 px yakınken en
büyük aşağı hız.

| | medyan | ortalama | en çok |
|---|---|---|---|
| pasif | 9.06 px/kare (1.33 m/s) | 9.06 | 18.48 (2.71 m/s) |
| refleks | 1.09 px/kare (0.16 m/s) | 1.21 | 4.10 (0.60 m/s) |

- Refleks hiçbir sahnede pasiften kötü değil. 23/24'te baş ≤ 1.47 px/kare
  (indirme hızı mertebesi). İstisna: −260 px @7.0 s (4.10; pasif 7.85).
  Kalça çok hızlı geriye kayarken eller omzun altında kalamıyor.
- Pasif 18 sahnede baş > 5 px/kare ile çarpıyor; refleksle hiçbirinde.
- Omuz: refleksle hiçbir sahnede zemine çarpmıyor (pasif 4 sahnede
  2–10 px/kare).
- Tüm sahneler duruyor: 480. karede son 60 karenin en hızlı noktası
  ≤ 0.009 px/kare.
- **Varsayılan sahne (150 px):** kare 222 çöküş, eller 238'de yerde,
  gövde ellerin üzerinde, omuz yerden 70 px yukarıda tutuluyor (240–257).
  Sonra kontrollü iniyor (omuz 70 → 40 px, ~60 kare). Kalça dizlerin
  üzerinde (53 px), alnı yerde duruyor.

### 30.7 Gerileme
- Kanarya 16/0/92/1. Son `hip_y` 277.01 (refleksle; kare 360). Yeni
  `CANARY_29` (`brace=False`) 277.38 (Adım 29: 277.33; fark §30.5'teki
  giriş/sürtünme düzeltmelerinden).
- `test_step29`: çöküş koşusu 420 → 480 kare (kontrollü indirme yığılmayı
  ~60 kare uzatıyor).
- Testler 103/103 geçti (yeni `tests/test_step30_bracing.py`: baş çarpması
  < 2 px/kare ve ortalamada ≥ 3× azalma, kolların omzu tutması, ellerin
  düşüş tarafına ve koni içine konması, kontrollü indirme + durgunluk,
  yürüyüşte/toparlanan itkide tetiklenmeme, aşırı itkide zemine girmeme,
  kol kapasitesi).

### 30.8 Sınırlar
1. Dirsek bükülmesiyle çarpma emilimi modelde var ama bu sahnelerde hiç
   devreye girmedi; çarpmayı düz kol kolonu durduruyor. Gerçekte bu,
   bilek kırığı (FOOSH) mekanizmasıdır.
2. Omuz–kol açısı için tork yok: kolon yalnızca omuz–el mesafesini
   tutuyor. Kol yataya yaklaşırsa destek biter. Eller ancak omzun
   altına konduğunda çalışıyor (−260 @7.0 istisnası).
3. Son poz çoğunlukla "alnı yerde diz çökmüş": 2B'de baş yana
   dönemiyor, refleks bitince kas tonusu yok. Kalkma yok.
4. Gövde hâlâ tek çubuk. Omurga segmentasyonu (yalnız yığılmada) sıradaki
   adım.
5. Render: yerdeki başın deri açısı 60° boyun sınırıyla ters görünebiliyor.

## Adım 30b — Alternatif koruyucu kollar: impuls tabanlı (Gül Nihal)

Çöküş sırasında kollar yere uzanır; sınırlı yay/sönüm kuvveti dirsek
bükülürken destek verir. Yürüyüş ve çöküş öncesi hareket aynıdır.
Birleşimden sonra bu sürüm `bracing="impulse"` ile seçilir (varsayılan refleks Adım 30'dur);
`ActiveBipedSim(gravity_mode="legacy", bracing=False, fall_solver="gn", articulated_spine=False)` onun Adım 29 karşılaştırmasını korur.

Referans 150 px itkide baş temas hızı 20.434 → 13.036 px/kare (%36.2),
göğüs temas hızı 11.328 → 5.081 px/kare (%55.1) azalır. 36 koşuluk taramada
19 çöküşün 15'inde baş, 16'sında göğüs hızı azalır; tüm çöken koşular
hareketsiz duruma yaklaşır. Bazı itki fazlarında temas hızı artar; bu sonuç
her düşüşte koruma garantisi değildir.

```bash
python3 demo/step30_bracing.py --sweep
python3 -m unittest discover -s tests
```

Karşılaştırma videosu: `outputs/step30_bracing_comparison.mp4`.
[Uygulama, ölçümler ve sınırlar](docs/ADIM30_KORUYUCU_KOL_REFLEKSI.md).
Sonraki sıra: yalnızca düşüşte omurga → tek yerçekimi kararı → ayağa kalkma;
her aşama video ve ölçümlerle ayrı incelenecek.

## Adım 31 — Yalnızca düşüşte esnek omurga

Çöküş anında kalça–omuz çubuğu iki parçaya ayrılır; kütleli bel eklemi,
yay/sönüm ve açı sınırları devreye girer. Çöküş öncesi topoloji ve hareket
korunur. Karakterin gömleği de iki gövde parçasına bağlanır.

```bash
python3 demo/step31_spine.py --sweep
```

Video: `outputs/step31_spine_comparison.mp4`.
`ActiveBipedSim(gravity_mode="legacy", articulated_spine=False)` Adım 30'u korur.
36 koşuda 19 çöküşün tamamı yerleşir; en yüksek son hız 0.00812 px/kare,
kalça kayması 0.1608 px altındadır. Darbe her koşulda azalmaz. Geçiş toplam
kütleyi ve iki momentumu korur; kütle dağılımı değiştiği için kinetik enerji
birebir korunmaz. [Ölçümler, testler ve sınırlar](docs/ADIM31_DUSUSTE_ESNEK_OMURGA.md).

## Adım 32 — Tek yerçekimi

Varsayılan `ActiveBipedSim()` yürüyüşten düşüşe kadar aynı 9.81 m/s²
(30 Hz ölçekte 2.228444 px/kare²) kullanır. Bacak yükü, yakalama torku,
kas-ağırlık hesabı ve temas tahmini aynı değere bağlıdır. Adım payları ve
tutunma hesabı yeniden ayarlandı; 60 saniyelik kuru/buzlu yürüyüş kararlıdır.

**Itki dayanıklılığı geriledi:** önceki güçlü itki taramasında 19/36 olan
çöküş sayısı 36/36 oldu. Hepsi zemin ihlali olmadan yerleşir. Küçük itkiler
ve yön bağımlılığı raporda ayrı gösterilir. Bu değişiklik tutarlı yerçekimi
sağlar; her düşüşte daha iyi koruma iddiası taşımaz.

```bash
python3 demo/step32_gravity.py --sweep
```

Video: `outputs/step32_gravity_comparison.mp4`.
`gravity_mode="legacy"`, Adım 31'in iki yerçekimli davranışını yeniden üretir;
Adım 30/29 için omurga/kol bayrakları da kapatılır. Yeni mod 30 Hz simülasyon
ister. [Ayarlar, ölçümler ve sınırlar](docs/ADIM32_TEK_YERCEKIMI.md).

## Adım 33 — Denge toparlama ve baş duruşu

Tek yerçekimi korunarak darbe hızı aynı karede okunur; geri adım kararı
ve acil adımın iniş hedefi düzeltildi. Orta şiddetteki 36 darbe denemesinde
çöküş **11'den 1'e** indi. Güçlü 36 denemenin tamamında hâlâ düşme var.
Düşüşteki diz sınırı zeminle birlikte çözülerek yeni geri düşüş pozlarında
ortaya çıkan sürünme giderildi.

Başın kaynak resimdeki eğimi göz işaret noktalarıyla düzeltilir; görüntü
boyna bağlanır. Boyun yürürken dik duruşu korur, düşüşte serbest kalır.
Darbesiz yürüyüşte görünür baş eğimi en fazla 4.29°; kuru/buzlu zeminde
60 saniyelik yürüyüşte çöküş yoktur. Yüzün orijinal pikselleri korunur.

```bash
python3 demo/step33_balance_head.py --sweep
```

Video: `outputs/step33_balance_head_comparison.mp4`.
Yakın plan: `outputs/step33_head_comparison.png`.
Adım 32 fiziği `balance_recovery=False, upright_head=False` ile yeniden
üretilebilir. [Ölçümler ve sınırlar](docs/ADIM33_DENGE_VE_BAS.md).

Başın boyunla birleşmesi ayrıca düzeltildi: yüz merkezi yerine kaynak
resimdeki boyun kökü, gömleğin yaka noktasına bağlanır. Yürüyüş/düşüşte
360 kare boyunca bağlantı farkı sayısal yuvarlama düzeyindedir.
Yakın plan video: `outputs/neck_attachment_comparison.mp4`;
yeniden üretmek için `python3 demo/neck_attachment_preview.py`.

## Adım 34 — El–diz desteğine geçiş

Ayağa kalkmanın ilk alt aşaması eklendi: yerde durulma, uygun yatış pozundan
kalça/göğsü kaldırma ve iki el–iki diz üzerinde kararlı bekleme. Tam doğrulma
ve yürüyüşe dönüş bu aşamada yapılmaz. Geçiş kuvvet/tork sınırlı motorlarla
çalışır; eller veya kalça sabitlenmez, yerçekimi değişmez.

```bash
python3 demo/step34_ground_support.py --sweep
```

Özellik `ActiveBipedSim(ground_recovery=True)` ile açılır; önceki pasif
karşılaştırmalar için varsayılan kapalıdır. Bacakların öne uzandığı yatışlar
`needs_roll` olarak ayrılır: henüz eklenmemiş dönme/yeniden yerleşme geçişi
gerektirir ve bu pozlarda motorlar çalıştırılmaz.

Video: `outputs/step34_ground_support_comparison.mp4`.
[Geçiş şartları, ölçümler ve sınırlar](docs/ADIM34_EL_DIZ_DESTEGI.md).

## Adım 35 — Bacak ve diz temasını yerleştirme

Öne uzanmış bacaklar sırayla arkaya alınır; havada kalan dizin hedef yönü
düzeltilerek gerçek zemin teması sağlanır. Aynı 36 darbe/faz koşulunun
36'sında kararlı el–diz desteği oluştu; 1500 karelik denemelerin son 60
karesinde korundu. Tümü ilk 967 kare içinde desteğe ulaştı. Tam doğrulmadan
önceki bu aşama `ground_recovery=True, recovery_reposition=True` ile açılır.

```bash
python3 demo/step35_support_placement.py --sweep
```

Video: `outputs/step35_support_placement_comparison.mp4`.
[Geçiş şartları, ölçümler ve sınırlar](docs/ADIM35_DESTEK_YERLESTIRME.md).

## Adım 36 — Bir ayağa yük aktarma

Kararlı el–diz desteğinden sol ayak gövdenin altına alınır; sınırlı bacak
itişiyle yük aktarılır ve eller/diğer diz destek vermeye devam eder.
Aynı 36 darbe/faz koşulunun 36'sında bu poz oluştu ve son 60 karede
korundu. Tam doğrulma bir sonraki aşamadır.

```bash
python3 demo/step36_foot_transfer.py --sweep
```

`ground_recovery=True, recovery_reposition=True, recovery_transfer=True`
ile açılır. Video: `outputs/step36_foot_transfer_comparison.mp4`.
[Başarı şartları, ölçümler ve sınırlar](docs/ADIM36_AYAGA_YUK_AKTARMA.md).

## Adım 37 — Gövdeyi doğrultma ve elleri ayırma

El destekli yarım diz çökmeden kalça ve göğüs yükselir; eller/dirsekler
yerden ayrılır. Son duruşta denge yalnızca öndeki ayak ve gerideki dizin
oluşturduğu destek aralığıyla doğrulanır. Tam ayağa kalkma sonraki aşamadır.

```bash
python3 demo/step37_kneel_rise.py --sweep
```

Önceki üç recovery seçeneğine ek olarak `recovery_rise=True` ile açılır.
Video: `outputs/step37_kneel_rise_comparison.mp4`.
[Geçiş şartları, ölçümler ve sınırlar](docs/ADIM37_GOVDEYI_DOGRULTMA.md).

Son taramada 36/36 koşul eller serbest son duruşu korudu; 149 test geçti.
Geçişte kısa süreli destek aralığı aşımı bulunur; son duruşun dengesi ile
hareket boyunca statik denge raporda ayrı değerlendirilir.

## Adım 38 — İki ayak üzerinde doğrulma

Dik yarım diz çökmeden gerideki ayak öne yaklaşır; diz desteği bırakılır ve
iki ayak üzerinde tam doğrulma sağlanır. Aynı 36 darbe/faz koşulunun 36'sı
ayakta beklemeyle tamamlandı. Yeni ayağa kalkma geçişi boyunca destek payı
pozitif kaldı; önceki gövde doğrultma geçişinin sınırı raporda ayrıca belirtilir.

```bash
python3 demo/step38_standing.py --sweep
```

Önceki recovery seçeneklerine ek olarak `recovery_stand=True` ile açılır.
Video: `outputs/step38_standing_comparison.mp4`.
[Başarı şartları, ölçümler ve sınırlar](docs/ADIM38_IKI_AYAKTA_DURUS.md).
Sırada ayakta beklemeden yürüyüşe kontrollü dönüş var.

## Adım 39 — Ayağa kalktıktan sonra yeniden adım atma

Bilge ayakta beklemeden ağırlık aktarımına, ilk adıma ve dönüşümlü yavaş
adımlara geçer. İniş gerçek ayak temasıyla doğrulanmadan yeni adım başlamaz.
Fiziksel bacaklar korunur; eski yürüyüş ankrajına veya ani poz değişimine
başvurulmaz. Henüz normal yürüyüş hızı ve kol salınımı hedeflenmez.

```bash
python3 demo/step39_walk_restart.py --sweep
```

Önceki recovery seçeneklerine ek olarak `recovery_walk=True` ile açılır.
Video: `outputs/step39_walk_restart_comparison.mp4`.
[Geçiş şartları, ölçümler ve sınırlar](docs/ADIM39_YURUYUSE_DONUS.md).

## Adım 30–39 birleşimi (Zehra + Gül Nihal)

Gül Nihal'in `feature/steps-30-39` dalı Adım 29'dan ayrılıp Adım 30'u
yeniden yazmış, 31–39'u onun üstüne kurmuştu; main'deki Adım 30 ile aynı
yığılma çözücüsünü değiştiriyordu. Birleşimde:

- **Tek koruyucu refleks: Adım 30 (kol kolonu)** — düşüşte de, yerden kalkma
  zincirinde de varsayılan. Aynı 36 koşuluk taramada (±2, ±2.5 m/s, ±150 px × 6
  faz; baş/omuz zemine 4 px yakınken en büyük düşüş hızı, px/kare):

  | Koşul | Pasif | Adım 30 (kol kolonu) | Adım 30b (impuls) |
  |---|---:|---:|---:|
  | İki yerçekimi, tek parça gövde — baş medyanı | 8.3 / 12.3 | **1.05** (18/18 iyileşme) | 9.7 (13 iyi / 6 kötü) |
  | Tek yerçekimi + omurga — baş medyan / ort. / maks. | 9.2 / 10.4 / 26.7 | **4.3 / 5.8 / 19.5** (25 iyi / 10 kötü) | 9.4 (17 iyi / 19 kötü) |
  | Tek yerçekimi + omurga — omuz medyan / ort. / maks. | 7.7 / 8.7 / 20.8 | **5.6 / 5.3 / 8.9** | — |

  Pasif sütunda iki değer: sırasıyla kol kolonu ve impuls sürümlerinin kendi
  yığılma çözücüleri. Tekil sahneler kaotik: örneğin ±150 px faz 0'da refleks
  pasiften kötü; iyileşme taramanın genelinde.
- **31–39 Gül Nihal'in dalından** gelir: düşüşte omurga, tek yerçekimi
  (`GravityPolicy`), denge/baş, el–diz desteği, ayağa kalkma, yürümeye dönüş.
- `bracing`: `True` (varsayılan, kol kolonu) · `"impulse"` (Adım 30b) · `False`
  (pasif). `brace=` eski adıyla çalışır. Gül Nihal'in nesnesi `sim.fall_bracing`;
  `sim.bracing` kol kolonu refleksinin durumu, `sim.fall_reflex_active` ikisini sorar.
- **Yığılma çözücüsü** (`fall_solver`): `"column"` Adım 30'un çözücüsü (24 tur,
  omurgada en az 64; yüke bağlı statik sürtünme, son kilit, giriş düzeltmesi,
  döngü içi boyun), `"gn"` Gül Nihal'in dalındakinin birebiri. Varsayılan: impuls
  refleksiyle ya da yerden kalkma açıkken `"gn"` (zincir bu çözücüyle ve onun
  yerde-kol çözümüyle — `gn_fallen_arms` — doğrulandı), diğerlerinde `"column"`.
  Eski koşullar (`gravity_mode="legacy", articulated_spine=False`) main'deki
  Adım 30 kanaryalarını, impuls modu Gül Nihal'in rapor sayılarını aynen üretir.
- Adım 31–33 bölümlerindeki sayılar impuls refleksiyle ölçüldü; varsayılan
  refleks değiştiği için o demoların yeni çıktıları farklıdır (testleri geçiyor).

### Refleks tetiği: baş/göğüs TTC'si

Tek yerçekiminde çöküş çoğu zaman bacak servosu devreye girmeden, kalça destek
yüksekliğinin altına inince algılanıyor; refleksin bacak "yield" tetiği oluşmuyor
ve refleks çöküş karesinde açılıyordu (−200 px: baş 4 kare sonra yerde, pasif 20.5
→ refleks 23.8 px/kare). Kollar artık **baş ve göğsün (omuz) zemine balistik
çarpma süresini** dinler (`contact_ttc`; Adım 24/25 ayak TTC kapısının gövde
karşılığı): TTC ≤ `BRACE_TTC_FRAMES` = 10 kare olunca refleks açılır. Ayakta
duran başın TTC'si ~15.4 kare (serbest düşüş), yürüyüşte tetiklenmez (ölçüldü).

| Tetik (36 koşu) | Baş medyan / ort. / maks. | Omuz medyan / ort. / maks. |
|---|---|---|
| Yalnız çöküş/yield (TTC yok) | 5.2 / 7.0 / 21.0 | 7.7 / 6.7 / 9.5 |
| Kalça hızı öngörüsü (önceki deneme, kaldırıldı) | 6.8 / 7.9 / 21.5 | 4.4 / 5.3 / 10.9 |
| TTC ≤ 4 | 5.2 / 7.0 / 21.0 (hiç etkisi yok — çöküş daha önce algılanıyor) | 7.7 / 6.7 / 9.5 |
| TTC ≤ 10, yön kapısız | 6.8 / 7.9 / 21.5 | 3.6 / 5.0 / 10.9 |
| **TTC ≤ 10 + yön kapısı** | **4.3 / 5.8 / 19.5** | **5.6 / 5.3 / 8.9** |
| TTC ≤ 12 / 14 + yön kapısı | 3.5 / 6.7 / 20.2 · 3.5 / 7.8 / 25.1 | 6.7 / 5.8 / 8.7 · 4.1 / 4.9 / 9.3 |

Baş/göğüs takasının nedeni sensör değil **yön kararıydı**: erken açılan refleks
dik çökmede (omuz kalçanın üstünde, yön belirsiz) dünyaya sabit el hedefini
kilitliyor, eller kalçanın altına iniyor, gövde sonra devrilince kolon omzu
tutamıyordu. Yön kapısı (`BRACE_DIR_MIN_PX` = 8 px): omuz–kalça farkı (3 kare
ileri) bundan küçükken hedef kilitlenmez, kollar omzun altına "hazır" uzanır.
TTC tetiği ve yön kapısı yalnız tek yerçekiminde (eski koşul kanaryaları aynen).

### Boyun: kamçı ölçümü ve çözücü içi sönüm (XPBD, bayrakla)

Kamçı ölçüldü (36 koşu, göğüs/omuz zemine ilk değdiği kare). Pasif düşüşte baş–gövde
göreli hızı temasta 349°/s, sonraki karelerde 854°/s'ye çıkıyor; boyun 60°'lik sert
sınıra çarpıyor. Adımlar arası sönüm (`spine.hinge_drive`, `NECK_FALL_*`) etkisizdi:
tepe, göğüs zemin çözücüsünün içinde tek karede durdurulurken oluşuyor.

Çözüm: boyun için **çözücü döngüsünün içinde** XPBD (Macklin 2016) sönümlü açı kısıtı
(`physics/spine.SoftHinge`; `NECK_XPBD_ENABLED`, `NECK_XPBD_COMPLIANCE` α,
`NECK_XPBD_BETA` β). Sönüm terimi ∇C·(x − xⁿ) karenin başından beri olan yer
değişimine bakar; zemin göğsü döngü içinde durdurduğunda boyun da aynı döngüde
frenlenir. Doğrusal momentum korunur (test).

| Ayar (36 koşu) | Pasif baş darbesi (medyan) | Pasif kamçı tepe | Pasif boyun açısı | Refleksli baş darbesi | Refleksli kamçı tepe |
|---|---:|---:|---:|---:|---:|
| XPBD yok (varsayılan) | 9.2 | 854°/s | 60.2° (sınırda) | **4.3** | **259°/s** |
| α 0.5 (β 1/4/16) | 8.5–9.6 | 832–1072°/s | 60.2° | 3.7–6.0 | 271–282°/s |
| α 0.05 (β 4/16) | 6.2–7.7 | 1049–1146°/s | 60.2–60.4° | 5.1–6.8 | 316–330°/s |
| **α 0.01, β 16** | **7.0** | 558°/s | 47.5° | 5.1 | 386°/s |
| α 0.02, β 32 | 7.3 | 884°/s | 57.6° | 4.7 | 293°/s |
| α 0.005, β 16 | 9.7 | **356°/s** | **36.6°** | 7.3 | 223°/s |

Yumuşak α'da (0.5) kısıt paydaya göre (~0.01) çok esnek, iterasyon başına neredeyse
düzeltme yapmıyor. α 0.01–0.005 kamçıyı %35–58 kırıyor ve boynu sınırdan uzak tutuyor;
pasif düşüşte baş darbesini de azaltıyor (α 0.01). Refleks açıkken baş, sertleşen boyunla
gövdeyle birlikte daha sert iniyor. Tek ayar her durumda kazandırmadığı için varsayılan
kapalı; açık işler: refleks moduna göre α seçimi.

### Kalkma override'ı (exception yok)

Refleks ile yerden kalkma her modda birlikte kurulur. Kalkma sürerken
(`waiting`/`needs_roll`/`failed` dışında) 4 kare üst üste:
- XCoM (KM + v/ω₀) bu karedeki temas noktalarının x aralığının 30 px dışında
  (yalnız KM ≥ 60 px iken — diz üstünden itibaren; normal kalkış/yürüyüşte en
  fazla 15.5 px, ölçüldü), ya da
- baş/göğüs zeminden > 80 px yukarıda, > 2 px/kare iniyor ve TTC ≤ 10

ise kalkma kesilir (`sim.recovery_aborts`), durum makinesi baştan kurulur, etkin
refleks yeniden açılır (`brace_trigger == "override"` ya da yeni `FallBracing`);
karakter durulunca zincir kendiliğinden yeniden başlar. `sim.push(px)` dış darbe.
Ölçülen (`tests/test_recovery_override.py`): ayaktayken −120 px → 4 karede kesme,
düşüş, ±150 px itkilerin ikisinde de ~2100 kare sonra yeniden ayakta; normal
kalkışta hiç kesme yok. Kalan `ValueError`'lar yalnızca kurulum ön koşullarıdır
(ör. `recovery_stand` için `recovery_rise`), çalışma sırasında durum engellemez.

### Kalkma telemetrisi (`demo/recovery_telemetry.py`)

Birim: 1 motor birimi = 0.291 Nm (480 = 140 Nm), 1 kuvvet birimi = 59.5 N.
Rapor: `docs/validation/recovery_telemetry_report.json` (±150 px, tam zincir).
- **140 Nm hiç aşılmaz — kodda sert kesilir; istenen aşar.** Uyluk motoru
  (diz→kalça) ileri düşüşten kalkışta 172.6 Nm ister, karelerin %12.9'unda
  tavanda; geri düşüşte bacak yerleştirmede 366 Nm ister. Baldır motoru
  (diz→ayak, tavan 35 Nm) ayak yerleştirmede karelerin %87'sinde tavanda
  (istenen 167 Nm). Ayağa kalkarken kalça motorları en fazla 116 Nm; bacak
  itme kuvveti bacak başına en fazla 388 N (ağırlık 687 N).
- **Asıl biyomekanik hile: net dış tork.** Motorlar eklemde iki parça arasında
  değil, tek parçaya dünyaya karşı uygulanan kuvvet çiftleri; gerçek vücutta iç
  torkların toplamı sıfırdır. Toplamları yerdeki/el–diz/tek diz fazlarında
  ortalama 96–174 Nm, tepede 464 Nm; hareketsiz el–diz tutuşunda bile sabit
  143.5 Nm. Gövdeyi doğrultmada 81–85 Nm; diz üstünde ve ayakta 12–22 Nm,
  yürürken 16–44 Nm.
- **Çoklu temas:** yerdeyken FABRIK çözülmez (bacaklar Verlet zinciri, IK yalnız
  çizim için fizikten okunur); yük paylaşımı PBD zemin temaslarından çıkar.
  Temas kayması 0 (statik sürtünme kilidi), çubuk boyu hatası ≤ 0.39 px, KM
  ivmesi kalkışta ≤ 9.3 m/s² (yürüyüş inişinde 3.7). Çekişme göstergesi motor
  iptali: torkların yerde %20–60'ı, ayakta %87'si birbirini götürüyor.
- Yukarıdaki sayılar varsayılan (dış torklu) motorlar içindir. İç tork modunda
  (`INTERNAL_TORQUES`) net dış tork 0 — bkz. "Dürüst fizik modu"; tepki torkları
  `recovery.reaction_log`'da, telemetri onları da toplar.

### Dürüst fizik modu (bayrakla; kalkış henüz çalışmıyor)

`physics.ground_recovery.INTERNAL_TORQUES = True` ve
`demo.step14_active_biped.COULOMB_RECOVERY = True` (varsayılan ikisi de `False`).

- **İç eklem torkları:** her kalkma motoru kendi parçasına +T, eklemin öbür
  tarafındaki parçaya −T verir (`reaction_segments`): gövde ve uyluk motorları kalça,
  baldır diz, baş boyun, kol omuz eklemi; yürüme kuvvetinin eksen dışı bileşeni kalça
  torkuna çevrilir. Net dış tork her karede tam 0 (test). Ayaklar tek nokta olduğu için
  ayak bileği torku yok. Diz motoru bu modda (ayak yerleştirmeden sonra) Adım 27'nin
  200 Nm diz ekstansörü tavanını kullanır (eski 35 Nm ile ön bacak ağırlık taşıyamıyordu).
- **Kare bütçeli Coulomb:** bir temas noktasının bir karedeki toplam teğetsel düzeltmesi
  ≤ μₛ·(toplam normal düzeltme); aşınca kinetik (μₖ = 0.6). Eski kilit (0.6 px'ten yavaş
  kayan her noktayı yükten bağımsız sabitleme) ve iterasyon başına kontrol, sürtünme
  kapasitesini ~iterasyon sayısı (64) kadar katlıyordu. Refleksin avuç kavraması yalnız
  refleks sürerken.
- **Ayak yerleştirme yeniden yazıldı** (iç tork modunda, `PLACE_KEYFRAMES`): kalça 82 px
  iken (uyluk 92 px) diz kalçanın altından ancak yerin içinden geçebiliyordu; eski hedefler
  ön dizi yük altında zeminde 130 px, ayağı 135 px sürüklüyordu. Yeni: (A) uyluklar dik,
  kalça 104 px; (B) diz ve ayak havada öne (ayak ≥ 30 px yukarıda, test); (C) basma.

Ne ortaya çıktı (hepsi ±150 px itki, tam zincir):

| Mod | Nerede duruyor | Neden |
|---|---|---|
| Dış tork + eski kilit (varsayılan) | ayakta, yürüyor | gökyüzü kancası: yerde ort. 96–174 Nm dış tork |
| İç tork + eski kilit | gövdeyi doğrultma | kalça ekstansörü 140 Nm tavanda, istenen 348–502 Nm |
| + diz 200 Nm, kalça 210 Nm, el itişi | gövde 163° → 25° dikleşiyor ama | KM destek dışında 19 px; ~2100 N'luk sürtünme çiftiyle tutuluyor (Coulomb sınırı ~270 N) |
| İç tork + kare bütçeli Coulomb | ayak yerleştirme (+150) / gövde doğrultma (−150) | yükü az eller 70–85 px kayıyor; senaryo dengeyi sağlamıyor |

Sonuç: Gül Nihal'in kalkışı dünyaya göre sabit açılara giden motorlarla yazılmış bir
senaryo; dış tork ve yapışkan/sızdıran temas kaldırılınca dengeyi kendisi kuramıyor.
Sıradaki iş: temas kuvvetlerini ve KM'yi her karede gözeten bir denge kontrolcüsü
(README'de ayrı bölüm açılacak).

#### Statik tork analizi, eklem uzayı denetimi, geniş taban (10 Ekim)

- **Statik ihtiyaç:** gövdeyi doğrultmada kalçanın statik torku (üst gövde ağırlığının
  kalçaya göre momenti, ellerin taşıdığı yük düşülerek) her karede **30–94 Nm**. 140 Nm
  tavanı yetiyor; "500 Nm" PD'nin büyük açı hatasıyla istediği tork, statik ihtiyaç değil.
- **Asıl sebep motor mimarisiydi:** parça başına dünya-açısı motorları iç tork modunda
  aynı kalça eklemine düşüp birbirini götürüyordu (gövde 140 Nm'de, uyluklar ona karşı).
  Gövdeyi doğrultmadan itibaren **eklem uzayı** (`JOINT_SPACE_STATES`): senaryonun hedef
  açıları toplanır, her eklemde (kalça, diz, boyun, omuz) tek motor göreli açıyı sürer.
  Yerdeki fazlarda parça motoru + eklem-komşusu tepkisi kalır (eklem uzayı orada
  gövdenin dünya yönünü belirleyemiyor, 'rising'de takılıyordu).
- **Geniş taban:** eski yerleştirme ön ayağı arka dizle aynı x'e koyuyordu (taban 1.4 px).
  Yeni yarım diz çökme: ön uyluk yatay, ön baldır dik, ön ayak arka dizin ~90 px önünde
  (`PLACE_KEYFRAMES` C, `WIDE_TRANSFER`); ölçüt "KM arka diz–ön ayak arasında ≥ 5 px".
- **Şu anki durma noktası:** iki yönde de tek diz üstüne kadar (iç tork, eski kilit) geliniyor;
  gövde doğrultmada **ön kalça eklemi** 140 Nm'de, istenen 145 → 270 Nm'ye tırmanıyor ve
  kalça yere iniyor. Nokta ayakta ayak bileği torku olmadığından yarım diz duruşu
  (arka diz + ön ayak + üç eklem) kapalı bir dört-çubuk; PD'nin yer çekimi ileri beslemesi
  olmadan sarkması ve arka uyluğun yükü sütun gibi taşıyamaması sıradaki iş.

### Diğer düzeltmeler (birleşim incelemesi)
- Omurga açıkken boyun sınırı bel–baş arasına uygulanır (önce uzunluk bel–omuz
  parçasıyla hesaplanıp kısıt kalça–baş arasına konuyordu).
- `demo/step30_bracing.measure()` kol kolonu refleksini de raporlar (`brace_mode`).
- Bilinen sınır (birleşimden önce de vardı): tek yerçekiminde 1 m/s geri itki
  bile çöküşle bitiyor; iki yerçekimli eski ayarda bitmiyordu.

Testler: 184/184 (tam paket). `tests/test_step30_impulse_bracing.py`
Gül Nihal'in Adım 30 testleridir.

## Üçüncü Taraf Kod Kullanımı ve Lisanslar

Bu projedeki fizik modülleri, sıfırdan yazılmak yerine bilinçli olarak
lisansı uygun iki açık kaynak projeden **uyarlanmıştır**. Her iki proje de
izin veren (permissive) lisanslara sahiptir ve her iki lisans da orijinal
telif bildiriminin ve lisans metninin korunmasını şart koşar — bu yüzden
orijinal lisans dosyaları `third_party_licenses/` altında bulunuyor ve
her uyarlanan dosyanın başında hangi projeden, hangi lisansla ve hangi
değişikliklerle alındığı açıkça belirtiliyor.

### `physics/verlet.py`
- **Kaynak:** [austinweis/python-verlet-integration](https://github.com/austinweis/python-verlet-integration)
  (`src/rag.py`)
- **Lisans:** Apache License 2.0 — bkz. [`third_party_licenses/LICENSE-apache-2.0-python-verlet-integration.txt`](third_party_licenses/LICENSE-apache-2.0-python-verlet-integration.txt)
- **Yapılan değişiklikler:** Python listeleri yerine numpy array'ler,
  opsiyonel sınır (bounds) çarpışması, tip belirteçleri, `Rag` →
  `VerletSystem` yeniden adlandırması ve `pin`/`add_point`/`add_stick`
  yardımcı metodları eklendi.

### `physics/fabrik.py`
- **Kaynak:** [Yanneeh/Fabrik-Inverse-kinematics](https://github.com/Yanneeh/Fabrik-Inverse-kinematics)
  (`fabrikSolver.py` — `Segment2D` / `FabrikSolver2D` sınıfları)
- **Lisans:** MIT License, Copyright (c) 2020 Yannick van Diermeen — bkz.
  [`third_party_licenses/LICENSE-mit-fabrik-inverse-kinematics.txt`](third_party_licenses/LICENSE-mit-fabrik-inverse-kinematics.txt)
- **Yapılan değişiklikler:** 3D sınıflar ve matplotlib görselleştirme
  kaldırıldı, sonsuz döngüyü önlemek için `max_iterations` limiti eklendi,
  hareketli karakterler için `set_base()` metodu eklendi.

### İncelenip **kullanılmayan** kaynaklar (telif/uygunluk nedeniyle)
Aşağıdaki iki proje de incelendi ancak koda dahil edilmedi:

- [harshaxnim/ragdoll](https://github.com/harshaxnim/ragdoll) — LICENSE
  dosyası yok (varsayılan "tüm hakları saklıdır"), ayrıca C/OpenGL dilinde
  ve depoda derlenmiş bir binary (`runThis`) barındırıyor. Sadece kavramsal
  referans (Thomas Jakobsen'in "Advanced Character Physics" makalesi) olarak
  faydalanıldı, kod alınmadı.
- [nbogie/p5js-ik-tentacles](https://github.com/nbogie/p5js-ik-tentacles) —
  LICENSE dosyası yok, `package.json` içinde de lisans alanı belirtilmemiş.
  JavaScript/p5.js dilinde (bizim Python pipeline'ımızla uyumsuz). Sadece
  "IK ile hedefe uzanma" konseptini görselleştirmek için referans alındı,
  kod kopyalanmadı.

FABRIK ve Verlet integration algoritmalarının kendileri akademik literatürde
yayınlanmış genel yöntemlerdir (Aristidou & Lasenby 2011; Jakobsen,
"Advanced Character Physics") — telif konusu yalnızca belirli kod
implementasyonları için geçerlidir, yukarıdaki attribution bu yüzden
mevcuttur.
