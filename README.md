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
