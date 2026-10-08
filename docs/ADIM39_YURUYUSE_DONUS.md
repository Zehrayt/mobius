# Adım 39 — Ayakta duruştan yavaş yürüyüşe dönüş

Bilge iki ayak üzerinde dengede bekledikten sonra ağırlığını öndeki ayağa aktarır,
arkadaki ayağını kaldırıp öne basar ve sağ–sol adımları tekrarlar. Bu aşamanın
hedefi, düşüş → toparlanma → ayağa kalkma → yeniden ilerleme zincirini tamamlamaktır.
Adımlar henüz yavaştır; eski normal yürüyüş hızına ve kol salınımına dönülmez.

## Geçiş ve kontrol

`recovery_walk=True`, önceki bütün toparlanma bayraklarını gerektirir. Varsayılan
simülasyon ve önceki aşamalar aynı kalır. Ayakta 60 kare beklenir, ardından ayakta
tutma ve yürüme kuvvetleri 180 kare boyunca yumuşakça harmanlanır. Her adımda
120 kare ağırlık aktarımı, 120 kare salınım ve en az 80 kare iniş vardır (30 FPS).
Yeni iniş doğrulanmadan diğer bacağın adımı başlamaz.

Bacakların fiziksel noktaları, kütleleri, hızları, omurga ve yerçekimi korunur.
Eski yürüyüşün sabit destek ankrajı tekrar bağlanmaz; ayaklar zemine pinlenmez.
`collapsed` ve `fell` tarihsel bayrakları serbest gövde çözücüsünü seçmeye devam eder;
güncel davranış `recovery.state` içindeki `walk_*` durumlarından okunur.

Kalça–ayak kuvvet çiftleri konum ve hız hatasıyla hesaplanır. Yatay kuvvet ±4,
düşey kuvvet −14/+8 çözücü birimiyle sınırlıdır. Toplam doğrusal momentum korunur;
bunlar dünya eksenine göre çalışan basitleştirilmiş aktüatörlerdir, açısal momentumu
koruyan biyomekanik kas modeli değildir. Gövde ve kollar önceki duruş motorlarıyla
tutulur. Görsel ayakkabı, yalnızca ölçülen zemin teması varken düz basar.

## Başarı ölçütleri

- Ağırlık aktarımından sonra destek ayağı gerçekten yüklü ve kalça ayağa 8 pikselden yakın olmalı.
- Tamamlanan adımda ayak en az 8 piksel yükselmeli ve en az 25 piksel ilerlemeli.
- İnişte iki ayak en az beş kare yüklü olmalı; sayısal temas payı her ayakta en az %10 olmalı.
- Ayak dışında zemin teması, alçalan kalça, aşırı gövde eğimi, sürekli desteksizlik veya aşama zaman aşımı denemeyi başarısız kılar.
- Tarama kabulü en az üç dönüşümlü adım, 120 piksel ilerleme, adım başına 1 pikselden az destek ayağı hareketi ve kare başına 3 pikselden az kalça hareketi gerektirir.

Temas payları, kısıt çözücüsündeki zemin düzeltmelerinden hesaplanan tanı göstergesidir;
gerçek vücut ağırlığı yüzdesi değildir. Tek ayak üzerinde yürüyüş dinamik denge
kullanır; kütle merkezinin sıfır genişlikli ayak aralığında kalması başarı koşulu
sayılmaz. Önceki gövde doğrultma aşamasının kısa süreli negatif destek payı değişmedi.

## Tekrarlama

```sh
python3 demo/step39_walk_restart.py --sweep
python3 -m unittest discover -s tests -p 'test_step39_walk_restart.py'
```

Aynı altı itki ve altı faz birleşimi toplam 36 denemedir; her biri 3700 kare sürer.
Ölçümler: `docs/validation/step39_walk_restart_report.json`.
Karşılaştırma videosu: `outputs/step39_walk_restart_comparison.mp4` (simülasyonun
58–100. saniyeleri). Sol panel Adım 38'in ayakta bekleyişini, sağ panel Adım 39'un
adımlarını gösterir; alt paneller gerçek fizik iskeletidir.
