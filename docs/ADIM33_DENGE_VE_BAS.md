# Adım 33 — İki yönde denge toparlama ve baş duruşu

Adım 32'nin tek yerçekimi korunur. Ayağa kalkmaya geçmeden önce orta şiddette
itkilerdeki yön bağımlılığı ve sürekli eğik duran baş ele alındı.

## Değişiklikler

- **Güncel hız:** dış darbe `prev_points` üzerinden hızı değiştirdikten sonra
  kontrol hızı yeniden okunur. Ayak hedefi ve kapanma sensörü darbe öncesi
  hızı bir kare daha kullanmaz. Dönüş değeri olan darbe öncesi kalça hızı
  geçmiş kayıtlarla uyum için korunur.
- **Geri adım:** yakalama noktası destek ayağının gerisinde kaldığında da
  normal adım başlayabilir. Diğer ayak havadaysa destek ayağı bırakılmaz.
- **Kararlı iniş hedefi:** tork sınırlı acil adım başlangıçta seçilen yere
  iner. İleri kayan kalçayı takip ederek inişi sürekli ertelemez. Ayak
  ışınlanmaz; mevcut servo, Hill kuvvet-hız ilişkisi ve tork sınırı kullanılır.
  Sonraki adım yeni hedef seçebilir. Aynı salınım içinde ikinci bir darbe için
  yeniden planlama bu sürümde yoktur.
- **Başın kaynak eğimi:** baş kesitindeki iki göz merkezi `rig.json` içinde
  normalize işaret noktalarıdır. Yaklaşık 20.90° kaynak eğimi başın dönüş
  matrisinden çıkarılır. Dönüş boyun doğrultusuna bağlanır; gövde eğimi
  doğrudan yüze aktarılmaz. Örgünün kökü aynı baş matrisiyle taşınır.
- **Boyun duruşu:** yürüyüşte dünya dikeyini hedefleyen sönümlü PD kontrolü
  (`k=0.5`, `c=0.8`) baş ve omuza eşit/zıt kuvvet uygular. Net doğrusal
  momentum eklemez; mevcut gövde PD'si gibi dünya referanslı bir kontrol
  yaklaşımıdır, bütün vücudun açısal momentumunu koruyan kas modeli değildir.
  Çöküşte tamamen kapanır; mevcut düşüş boyun sınırı geçerlidir.
- **Yerde diz kararlılığı:** dizin ters bükülme sınırı, zemin ve kemik
  uzunluklarıyla aynı çözüm döngüsüne alındı. Üç ekleme kütle ağırlıklı
  düzeltme uygulanır. Eski çözüm sonrası yalnızca diz noktasını oynatma
  yaklaşımı, yeni geri düşüş pozlarında kemikleri kısaltıp sürünme ve
  aralıklı hareket üretiyordu. Eski fizik karşılaştırmaları eski yolu korur.

Yüz resmi yeniden üretilmedi, PNG dosyaları değiştirilmedi. Yerçekimi,
kalça/diz kuvvet-tork sınırları, koruyucu kol ve düşüş omurgası korunur.
Adım 32 fizik karşılaştırmaları açıkça `balance_recovery=False,
upright_head=False` kullanır. `gravity_mode="legacy"` önceki fizik
sonuçlarını korur. Eski baş görüntüsü `PhysicsBilgeRig(corrected_head=False)`
ile karşılaştırılabilir.

## Yeniden üretme

```sh
python3 demo/step33_balance_head.py --sweep
python3 -m unittest discover -s tests
python3 tests/smoke_demos.py
```

- `outputs/step33_balance_head_comparison.mp4`: önce/sonra; üstte karakter,
  altta fizik eklemleri; sonunda üç kat yavaş tekrar.
- `outputs/step33_head_comparison.png`: aynı yürüyüş karesinde baş yakın planı.
- `docs/validation/step33_balance_head_report.json`: eşleşmiş ölçümler.

Tarama orta itkilerde ±0.25, ±0.5, ±1; güçlü itkilerde ±2, ±2.5 nominal m/s
ve ±150 px darbe kullanır. Her büyüklük 0,5,10,15,20,25 kare fazında denenir.
Nominal m/s değerleri sabit momentum ölçeğidir; ölçülmüş COM hız değişimi
olarak yorumlanmamalıdır. Orta koşular 900, güçlü koşular 420 karedir.
Kuru ve buzlu zeminde darbesiz yürüyüş ayrıca 1800 kare gözlenir.

Video +1 nominal m/s, faz 0 örneğidir: Adım 32 kare 239'da çöker;
Adım 33 bu darbeyi adımla karşılar. Genel başarı tam taramayla ölçülür.

## Sınırlar

Model hâlâ IK ayakları ve Verlet gövdeyi birleştirir; destek çubuğu tam bir
kas-iskelet kuvvet çözümü değildir. Güçlü darbelerde düşme hâlâ beklenir.
Yerde yerleşme ölçümleri yalnızca çöken koşular için değerlendirilir;
yürüyen karakterin son konum değişimi yer kayması değildir. Sonraki aşama,
bu sonuçların video incelemesinden sonra, yerdeki karakterin ayağa kalkmasıdır.

## Sonuçlar

| Ölçüm | Adım 32 | Adım 33 |
|---|---:|---:|
| Orta darbe taraması: çöküş | 11 / 36 | 1 / 36 |
| Güçlü darbe taraması: çöküş | 36 / 36 | 36 / 36 |
| Darbesiz yürüyüşte en büyük göz hizası eğimi | 20.78° | 4.29° |
| Kuru zeminde son 300 karenin ortalama hızı | 1.919 px/kare | 2.015 px/kare |
| Buzlu zeminde son 300 karenin ortalama hızı | 1.925 px/kare | 2.015 px/kare |
| 60 saniye kuru/buzlu yürüyüşte çöküş | yok / yok | yok / yok |

Baş ölçümü yürüyüşün 30–209. karelerinde yapılır. Adım 33'ün ortalama baş
hizası yaklaşık −3.62°'dir; doğal hareket tamamen dondurulmaz. Tek kalan
orta darbe başarısızlığı −1 nominal m/s, faz 0 koşuludur. Bu nedenle yeni
kontrol her darbeyi karşılayan bir çözüm olarak değerlendirilmemelidir.

Yeni sekiz test; aynı karede darbe algısı, geriye adım, sabit iniş hedefinde
sürekli ayak hareketi, iki yönde toparlama, geri düşüşlerde yerleşme,
başın göz/ankraj geometrisi, uzun yürüyüş ve düşüşte boyun kontrolünün
kapanmasını kapsar. Önceki 116 test ve demo çalıştırma kontrolü de korunur.

Güçlü taramada 14 saniyenin son 60 karesinde 35/36 koşul 0.2 px/kare
hareket eşiğinin altındadır. +150 px, faz 25 koşulunda son uzuv hareketi
daha geç biter: 14 saniyelik pencerede en yüksek hız 1.161 px/kare,
16 saniyede 0.00543, 30 saniyede 0.000198 px/karedir. Bu uzatılmış
kontrol raporda ayrıca saklanır; ilk 14 saniyelik karşılaştırma korunur.
Güçlü taramada en yüksek son kalça kayması 0.0752 px altındadır; tüm
koşular sonludur ve zeminin altına geçiş yoktur. Eski sürümde güçlü
koşuların tamamı 14 saniyelik yerleşme eşiğini zaten sağlıyordu: yeni
sürümün bu tek koşuldaki yerleşmesi daha yavaştır.

## Boyun bağlantısı düzeltmesi

İlk baş düzeltmesi yüzü doğrultmuştu, ancak başı hâlâ yüz merkezinden
konumlandırıyordu. Kaynak resimdeki boynun alt ucu yakaya bağlı olmadığı
için dönünce yana kayıyordu. `neck_base` ve `neck_socket` işaretleri
artık aynı ekran noktasına eşlenir; baş, kendi boyun parçasıyla birlikte
boyun kökünden döner. Düşüşte yaka hedefi esnek gömleğin üst parçasından
hesaplanır. Saç örgüsü de başın yeni konumunu takip eder.

Aynı 360 fizik karesinde (yürüyüş ve düşüş), boyun kökü–yaka farkının
maksimumu 19.38 px'den sayısal yuvarlama düzeyine (1.14e-13 px) indi.
Bağlantı alanı en az 355 piksel örtüşür. Sadece parça örtüşmesi doğru
bağlantı için yeterli değildir: eski yerleşim de örtüşüyordu, fakat boyun
kökü yakanın merkezinden uzaktaydı. Yeni regresyon testi ikisini birlikte
kontrol eder. Fizik simülasyonu ve kaynak resim pikselleri değiştirilmedi.

```sh
python3 demo/neck_attachment_preview.py
```

Yeni yakın plan karşılaştırması: `outputs/neck_attachment_comparison.mp4`
(6 saniye, 30 FPS), `outputs/neck_attachment_comparison.png`.
Ölçümler: `docs/validation/neck_attachment_report.json`.
Deri, fizik-deri bağlantısı, baş duruşu ve yeni boyun bağlantısı için
çalıştırılan toplam 36 test geçti.
