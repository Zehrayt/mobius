# Möbius — Video Modu (Prosedürel 2D Animasyon Motoru)

Karakterleri elle kare kare çizmek yerine; iskeletleri **Verlet integration**
(yerçekimi + mesafe kısıtlamalı nokta-çubuk fiziği) ve **FABRIK** (Inverse
Kinematics) ile prosedürel olarak hareket ettirip, sahneyi kare kare
`OpenCV` ile `.mp4` olarak render eden bir motor. Hiçbir kare elle
çizilmez / keyframe kullanılmaz — hareket tamamen fizik ve hedef-tabanlı
matematikten doğar.

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
```

## İçerik

- `physics/verlet.py` — genel amaçlı nokta/çubuk Verlet integration sistemi
  (`VerletSystem`) + `clamp_direction()` yardımcı fonksiyonu (bir segmentin
  yönünü sabit bir referansa göre sınırlar — bkz. aşağıdaki "double
  pendulum" notu). Zincir, ağaç ya da kapalı iskelet (torso + kollar +
  bacaklar) gibi keyfi graf yapılarını destekler.
- `physics/fabrik.py` — 2D FABRIK IK çözücü (`FabrikChain2D`) +
  `clamp_joint_angles()` (bir eklemin büküm açısını ve yönünü sınırlar —
  ör. dizin tersine bükülmemesi). Bir zincirin (ör. kalça→diz→ayak bileği)
  ucunu bir hedef noktaya ulaştırır.
- `physics/gait.py` — `FootPlantingLeg`: FABRIK + "ayak basma" (foot-
  planting) state machine'i. Bu projede sıfırdan yazılmıştır.
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
7. Momentum ve esneme (squash & stretch) — verlet zincirlerinin hıza/düşmeye
   tepki olarak uzayıp büzüşmesi
8. Çoklu nokta çarpışması (multi-node collision & raycasting) — baş/omuz/
   kalça/ayaktan zemine ve engellere ışın taraması
9. Aktif/pasif ragdoll harmanı — IK güdümlü "aktif" durum ile tamamen
   yerçekimine teslim "pasif ragdoll" durumu arasında pürüzsüz geçiş
10. 🔶 İkincil fizik nesneleri (secondary animation) — kuyruk/kulak/pelerin
    gibi hedefsiz, saf verlet + hava direnci (drag) ile sarkan parçalar.
    **Başlandı:** bkz. `demo/step10_secondary_wind.py` (tek pelerin +
    rüzgar). Sonraki olası genişletmeler: kuyruk/kulak gibi başka örnekler,
    karakter durunca "bir süre daha salınmaya devam etme" davranışının
    ayrıca doğrulanması.
11. Dinamik sürtünme/tutunma — zemine/eyleme göre değişen eklem sürtünmesi
    (buzda kayma, bir kenara tutunma)
12. Kütle merkezi (center of mass) dengesi — denge kaybında kollarla
    counter-balance

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
