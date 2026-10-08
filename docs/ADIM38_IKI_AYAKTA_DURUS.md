# Adım 38 — İki ayak üzerinde tam doğrulma

Bilge, dik yarım diz çökmeden gerideki ayağını zemin boyunca öne yaklaştırır,
iki dizini de yerden kaldırır ve iki ayağı üzerinde doğrulur. Bu aşamanın
sonucu ayakta beklemedir; yürüyüş yeniden başlatılmaz.

```python
ActiveBipedSim(ground_recovery=True, recovery_reposition=True,
               recovery_transfer=True, recovery_rise=True,
               recovery_stand=True)
```

Yeni seçenek varsayılan kapalıdır; önceki tüm aşamalar gerekir. Uygulama
`physics/stand_recovery.py` içindedir. Adım 37'nin davranışı ayağa kalkmanın
başladığı kareye kadar birebir korunur.

## Geçiş ve itiş

Dik yarım diz çökme 60 kare korunduktan sonra `standing_rising` başlar.
Bacak yönleri ve kalça–ayak açıklığı 300 kare / 10 saniyelik yumuşak geçişle
ayakta duruşa yaklaşır. Önceki motorların komut açıları korunarak başlanır;
mevcut destek torku bir anda kesilmez. Gerideki ayak bu prototipte zemin
boyunca yaklaşır; havada adım atma animasyonu olarak sunulmaz.

Adım 36'nın 2 birimlik düşük kuvvetli yük aktarma motoru bütün gövdeyi
kaldırmaya yetmedi. Yeni bacak desteği, toplam serbest kütle ve yerçekiminden
hesaplanan ağırlık desteğini, açıklık/hız geri bildirimiyle birleştirir:

- Kalça–ayak açıklık hedefi son durumda 180 px; fizik bacakları 92+92 px'dir.
- Ağırlık desteği iki bacağa bölünür ve bacak ekseninin düşey bileşeniyle
  düzeltilir. Boy/hız geri bildirimi aynı eksende uygulanır.
- Her bacağın kuvveti **14 çözücü kuvvet birimi** ile sınırlıdır. İlk hedefe
  geçişte ön bacağın eski 2 birimlik desteğinden, arka bacağın sıfır yeni
  itişinden yumuşakça geçilir. Ölçülen tepe yeni kuvvet 4.946 birim altındadır.
- Dürtüler kalça ile ayak arasında eşit/zıt ve ekseneldir: bu yeni kuvvet
  çifti net doğrusal/açısal dürtü eklemez, konumları doğrudan taşımaz.

Bunlar SI cinsinden ölçülmüş kas kuvvetleri değildir. Önceki dünya yönüne
referans veren duruş motorları basitleştirilmiş model olarak kalır. Zemin
tepkileri destek sağlar; yerçekimi, mevcut tork sınırları ve sürtünme kuralı
korunur. Yeni aşamada Adım 37'deki 128 tekrarlı kısıt çözümü kullanılır.

## Başarı şartları

`standing` için hedef geçişi bitmeli ve aşağıdaki şartlar 30 kesintisiz
sakin kare sağlanmalıdır:

- İki ayak güncel zemin çözümünde gerçek temas üretir; diz, el, dirsek veya
  başka bir gövde noktası zemin desteği vermez.
- Kütle merkezi iki ayağın yatay destek aralığında, kenarlardan en az
  8 px içeride kalır. Ayak açıklığı 25–100 px arasındadır.
- Dizlerin altı yerden 60 px'den fazla ayrılır; kalça 170 px, göğüs 210 px
  üzerinde ve göğüs kalçadan en az 40 px yukarıdadır. Dikeyden gövde sapması
  15° altında kalır.
- İki ayağın her birinin sayısal temas payı en az 0.15 olur; nokta hareketi
  0.02 px/kare altında kalır. Temas payı gerçek ağırlık yüzdesi değil,
  kütleyle ağırlıklandırılmış zemin düzeltmesinden hesaplanan göstergedir.

Temas/geometri kaybı başarıyı hemen kaldırır. Beş kare süren hareket artışı
başarıyı kaldırır; tek karelik sayısal sıçrama tolere edilir. Yeniden deneme
sırasında bacak desteği boşaltılmaz. Bir deneme 600 karede sonuçlanmazsa
`standing_timeout` kaydedilir ve motorlar kapanır.

Yürüme ankrajı yeniden bağlanmaz; yeni sabitleme veya poz ataması yoktur.
`sim.collapsed` serbest düşüş/gövde çözücüsünü kullanmak için açık kalır;
gerçek güncel duruş `sim.recovery.state == 'standing'` ile bildirilir.
Eski yürüyüş kontrolüne güvenli dönüş ayrı bir sonraki aşamadır.

## Yeniden üretme ve inceleme

```sh
python3 demo/step38_standing.py --sweep
python3 -m unittest discover -s tests
python3 tests/smoke_demos.py
```

- `outputs/step38_standing_comparison.mp4`: aynı +150 px/faz 0 koşulunda
  Adım 37 ve 38. Önceki düşüşü tekrar uzatmamak için simülasyonun 38–66.
  saniyelerini normal hızda gösterir; üstte karakter, altta gerçek fizik.
- `outputs/step38_standing_comparison.png`: diz desteği ve iki ayakta duruş.
- `docs/validation/step38_standing_report.json`: referans ve 36 koşul.

Tarama aynı 36 darbe/faz çiftini kullanır. Gözlem, geri düşüşten sonraki tüm
geçişleri ve son beklemeyi kapsamak için 2700 kare / 90 saniyedir. Son
60 karede duruş, temaslar, denge ve ayak kayması ayrıca ölçülür. İki ayağın
görünür ayakkabı tabanları, gerçek temas varsa zemine düz basacak şekilde
çizilir; fizik noktaları değiştirilmez. Başın görsel bağlantısı değiştirilmedi.

## Ölçülen sonuçlar

- **36/36** koşul iki ayakta duruşla tamamlandı ve son 60 kare korundu.
  Ayağa kalkma sonrasında destek kaybı kaydedilmedi.
- Ayakta duruş kare 1734–2253 arasında sağlandı (57.80–75.10 s).
- Son kalça yüksekliği 186.36–186.38 px, göğüs yüksekliği 239.49–239.50 px.
  Diz açıklığı en az 83.04 px; ayaklar dışında zemin teması yok.
- Son durumda destek kenarına en küçük kütle merkezi uzaklığı 25.25 px;
  ayak açıklığı 60.31–60.45 px; gövde eğimi en fazla 0.286°.
- Son 60 karede en yüksek nokta hareketi 0.000833 px/kare; ayakların toplam
  yatay hareketi en fazla 0.01641 px. Zemin altına geçiş yok; bütün koşullar
  sonlu; kuvvet/tork sınırları aşılmadı.
- Yeni bacak kuvveti en fazla 4.946 birim; 14 birim sınırının 0.354'ünden azı.
  Sayısal temas payının iki ayaktaki küçük değeri 0.40286 üzerinde kaldı.

+150 px/faz 0 referansı: ayağa kalkma kare 1386'da başladı, kare 1734'te
kararlı ayakta beklemeye ulaşıldı. Yeni hareket 11.60 saniye sürdü. Son
kalça yüksekliği 186.37 px, göğüs yüksekliği 239.49 px. Yerçekimi sabit
2.228444 px/kare² kaldı.

## Denge sınırının kapsamı

**Yeni ayağa kalkma geçişinde** kütle merkezi gerçek temas noktalarının
aralığında kaldı: en küçük destek payı 13.11 px. Her iki ayak dışında destek
kalmadıktan sonraki en küçük iki ayak denge payı 24.88 px.

**Önceki gövde doğrultma geçişindeki** kısa süreli sınır aşımı devam ediyor:
koşullara göre -6.47 ile -6.02 px. Bu iki ölçüm raporda ayrı alanlardır;
bütün düşüşten ayağa kalkış dizisi her an statik dengede veya tam biyomekanik
olarak doğrulanmış kabul edilmemelidir. Sonuçlar bu 36 düz zemin denemesi
ve belirtilen motor modeli içindir.

## Doğrulama

156 testin tamamı geçti. Yeni testler iki düşüş yönünden ayakta beklemeyi,
kalkış başlayana kadar Adım 37 fiziğiyle birebir eşitliği, eski ankrajın
bağlanmamasını, yeni eksenel kuvvet çiftinin konumları doğrudan taşımadan
kuvvet sınırına uymasını ve net doğrusal/açısal dürtüyü korumasını doğrular.
Bir ayağın teması kaybolursa veya diz destek verirse başarı kaldırılır;
kısa hareket gürültüsü ile sürekli hareket ayrılır. Süre sınırında motorların
kapanması ve seçenek bağımlılıkları da test edilir. Eski demo çalıştırma
kontrolleri geçti.

Karşılaştırma videosu H.264, 1280×820, 30 FPS, 28 saniye / 840 karedir.
Bütün kareler okunarak doğrulandı; diz desteğinin bırakılması, ara yükseliş
ve son duruş görsel olarak incelendi. İki ayakkabı tabanı görünür zemine
basar; dizler ve eller son pozda yerden ayrı görünür.
