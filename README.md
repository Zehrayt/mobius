# Möbius — Video Modu (Prosedürel 2D Animasyon Motoru)

Karakterleri elle kare kare çizmek yerine; iskeletleri **Verlet integration**
(yerçekimi + mesafe kısıtlamalı nokta-çubuk fiziği) ve **FABRIK** (Inverse
Kinematics) ile prosedürel olarak hareket ettirip, sahneyi kare kare
`OpenCV` ile `.mp4` olarak render eden bir motor. Hiçbir kare elle
çizilmez / keyframe kullanılmaz — hareket tamamen fizik ve hedef-tabanlı
matematikten doğar.

**Yol haritası maddesi ↔ dosya eşlemesi netleşmemişse (ör. neden
`step6`/`step11` diye bir dosya yok) önce [`INDEX.md`](INDEX.md)'e bakın.**

## Kurulum

```bash
pip install numpy opencv-python
```

GPU gerekmez — bütün hesaplama numpy tabanlı skaler/vektör matematiği,
normal bir CPU'da gerçek zamanlıdan çok daha hızlı çalışır.

## Çalıştırma

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
```

## Mimari: çekirdek (`physics/`) vs. sahne (`demo/`)

`physics/` altı **jenerik, sahneden bağımsız** mekanizmalardan oluşur —
hiçbiri belirli bir karakter/sahne bilmez. `demo/` altındaki her `stepN_*.py`
ise bunları belirli sabitlerle (kalça yüksekliği, rüzgar gücü, buz bölgesi
x aralığı, ...) somut bir SAHNEYE bağlayan, kendi kendine çalışan bir
script'tir. Bu ayrım kullanıcı geri bildirimi üzerine eklendi — daha önce
çarpışma/ragdoll/denge mantığının bir kısmı `demo/` dosyalarında hardcode
idi (bkz. `INDEX.md`'deki refactor notu).

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
6. Sahne kayıt/render pipeline'ının (bu depo zaten `cv2.VideoWriter`
   kullanıyor) senaryo/diyalog sistemiyle genişletilmesi
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
