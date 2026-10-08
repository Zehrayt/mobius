# Adım 37 — Gövdeyi doğrultma ve elleri serbest bırakma

Adım 36'nın el destekli yarım diz çökmesinden kalça/göğüs yükseltilir.
Eller yerden ayrılır; karakter öndeki sol ayak ve gerideki sağ diz üzerinde
bekler. Henüz iki ayak üzerinde duruşa veya yürüyüşe geçilmez.

```python
ActiveBipedSim(ground_recovery=True, recovery_reposition=True,
               recovery_transfer=True, recovery_rise=True)
```

Yeni seçenek varsayılan kapalıdır ve önceki üç aşamayı gerektirir.
Uygulama `physics/kneel_rise.py` içindedir; önceki aşamayı yeniden kullanır.

## Hareket

- Adım 36'da kararlı desteğe ulaşıldıktan sonra 60 kare beklenir.
- `torso_raising` başlarken önceki motorların **komut açıları** korunur.
  Ölçülen açıları yeni hedef yapmak mevcut destek torkunu kesip gövdeyi
  düşürdüğü için bu geçişte kullanılmaz. Konumlar taşınmaz.
- Sağ uyluğun hedefi 120 karede 20°'den 5°'ye yaklaşır; bacak desteği
  hazırlanır. Gövde, baş ve kol hedefleri 60 kare gecikmeyle başlayıp
  240 karede dik yöne yaklaşır. Toplam hedef geçişi 300 kare / 10 saniyedir.
- Adım 36'nın kalça–ayak uzama motoru aynı 115 px hedef ve 2 kuvvet birimi
  sınırıyla çalışmayı sürdürür. Eski tork/kol kuvvet sınırları korunur.

Hedef yönler bu iki boyutlu prototipin kontrol parametreleridir; anatomik
kas kapasiteleri değildir. Dünya yönüne referans veren çift motorlar net
doğrusal dürtü eklemez, fakat tam bir açısal momentum/kas modeli de değildir.
Yerçekimi değiştirilmez; yeni sabitleme, kinematik poz ataması veya yürüme
ankrajına dönüş yoktur.

## Başarı ve kayıp kontrolü

`upright_kneeling` için 300 karelik geçiş bitmeli ve aşağıdaki şartlar
30 kesintisiz kare sağlanmalıdır:

- Sol ayak ve sağ diz güncel zemin çözümünde gerçek temas üretir.
- Kütle merkezi **yalnızca bu iki noktanın** yatay destek aralığında,
  kenarlardan en az 5 px içeridedir. Arkadaki ayak pasif olarak değebilir;
  denge hesabına katılmaz.
- Hiçbir el veya dirsek zemin teması üretmez; iki elin altı zeminden
  en az 20 px ayrılmıştır. Öndeki dizin açıklığı 25 px üzerindedir.
- Kalça başlangıcından en az 5 px yükselmiş; göğüs 120 px'den yüksek ve
  kalçadan en az 40 px yukarıdadır. Gövde eğimi dikeyden 20° altında kalır.
- Öndeki ayağın sayısal temas payı en az 0.2; bütün serbest noktaların
  kareler arası hareketi 0.02 px/kare altındadır.

Temas payı, çözücünün kütleyle ağırlıklandırılmış zemin düzeltmelerinden
hesaplanır; gerçek kuvvet veya vücut ağırlığı yüzdesi değildir. Önceki yük
aktarımı sırasında kullanılan dört temas şartı bu aşamada uygulanmaz;
ellerin desteği bırakması ayrıca doğrulanır.

Temas veya geometrik denge kaybında başarı hemen kaldırılır ve doğrulma yeniden denenir.
Hız sınırının aşılmasında ise beş ardışık kare aranır; tek karelik sayısal
sıçrama kararlı durumu kaldırmaz. Başarıya ilk giriş hâlâ 30 kesintisiz
sakin kare gerektirir. Bir deneme
600 kare içinde sonuçlanmazsa `torso_raise_timeout` kaydedilir ve motorlar
kapanır. Karakter mevcut yerçekimi/kısıt çözümüyle hareket etmeye devam eder.

## Yeniden üretme

```sh
python3 demo/step37_kneel_rise.py --sweep
python3 -m unittest discover -s tests
python3 tests/smoke_demos.py
```

- `outputs/step37_kneel_rise_comparison.mp4`: +150 px/faz 0 düşüşünde
  Adım 36 ve 37; normal hızda simülasyonun 6–48. saniyeleri. Üstte karakter,
  altta gerçek fizik noktaları, kol teması, el açıklığı ve destek payı.
- `outputs/step37_kneel_rise_comparison.png`: son pozların karşılaştırması.
- `docs/validation/step37_kneel_rise_report.json`: referans ve tarama.

Tarama önceki aynı 36 güçlü darbe/faz çiftini kullanır. Gözlem bu yeni
hareketi ve son beklemeyi kapsamak için 2100 kare / 70 saniyedir. Son
60 karede destek, el açıklığı ve ayak/diz kayması ayrıca ölçülür. Geçiş
sırasındaki, eller serbestken ölçülen en küçük denge payı da rapordadır.
Son pozun dengesi ile hareketin her anında statik denge aynı iddia değildir.
Başın görsel bağlantısı değiştirilmedi. Bu sonuçlar düz zemindeki belirtilen
koşullar içindir; tüm zeminler ve başlangıç pozları için genel çözüm değildir.

## Sayısal yakınsama

İki noktadan destek alan yeni duruşta eski 64 tekrarlı ortak zemin/eklem
çözümü küçük ama sürekli bir artık kayma üretiyordu. İlk taramada 12 koşul,
son pozda kısa hız sıçramaları nedeniyle kararlı durumdan çıktı. Yeni
`torso_raising` ve `upright_kneeling` durumlarında çözüm 128 tekrara çıkarıldı;
önceki aşamalar 64 tekrar ve aynı fizik davranışıyla kalır. Başarıya giriş eşikleri,
tork/kuvvet sınırları veya sürtünme kuralları değiştirilmedi. Çok kısa hız
sıçramalarının durum değişimini tetiklememesi için yukarıda açıklanan beş
karelik hareket kaybı kontrolü ayrıca eklendi; gerçek temas/denge kaybı
bekletilmez.

Kontrol edilen -88.25185 px/faz 0 örneğinde son 60 karelik diz hareketi
0.41036 px'den 0.02426 px'e, en yüksek nokta hızı 0.01874 px/kareden
0.0004124 px/kareye indi. Bunun maliyeti, yeni yükseliş/bekleme aşamasında
kısıt çözümünün daha fazla işlem yapmasıdır.

Görsel olarak yeni yükseliş/bekleme durumunda gerçekten temas eden öndeki
ayağın ayakkabısı zemine düz basacak şekilde çizilir. Eski pasif düşüşteki
ayak bileği yönü, yük alan ayakkabıyı havada gösterebildiği için burada
kullanılmaz. Bu yalnızca deri dönüşümüdür; fizik ayağı/dizi taşınmaz.

## Doğrulama

149 testin tamamı geçti. Yeni testler iki düşüş yönünden eller serbest
kararlı duruşu, yükseliş başlayana kadar eski fizikle birebir eşitliği,
sınırlı motorların konumları doğrudan değiştirmediğini ve net doğrusal
dürtü eklemediğini doğrular. Ayak teması kaybı veya kol desteği başarının
hemen kaldırılmasına yol açar; kısa hız sıçraması tolere edilirken beş kare
süren hareket artışı başarıyı kaldırır. Deneme süresi sonunda motorların
kapanması ve seçenek bağımlılıkları da test edilir. Önceki demo çalıştırma
kontrolleri geçti.

## Ölçülen son sonuçlar

- **36/36** koşul eller serbest dik yarım diz çökmede tamamlandı ve son
  60 kare boyunca korundu. Son sürümde destek kaybı kaydedilmedi.
- Kararlı duruş kare 1326–1845 arasında oluştu (44.20–61.50 s).
- Son kalça yüksekliği 103.15–103.21 px, göğüs yüksekliği 156.27–156.32 px.
  Ellerin altı zeminden en az 85.27 px ayrıldı; hiçbir el/dirsek temas etmedi.
- Son durumda dikeyden gövde sapması en fazla 0.557°; kütle merkezinin
  ayak–diz destek sınırına uzaklığı en az 13.79 px.
- Son 60 karede en büyük nokta hareketi 0.001369 px/kare; ayağın toplam
  yatay hareketi en fazla 0.000916 px, dizinki 0.024537 px. Zemin altına
  geçiş yok; bütün koşullar sonlu; kuvvet/tork sınırı oranları en fazla 1.0.

+150 px/faz 0 referansı: doğrulma kare 992'de başladı, kare 1326'da kararlı
bekleme sağlandı; yeni hareket 11.13 saniye sürdü. Kalça 89.84 px'den
103.20 px'e yükseldi; göğüs 156.32 px yüksekliğe ulaştı. Yerçekimi
2.228444 px/kare² olarak korundu.

**Geçişin sınırı:** eller serbestken kütle merkezi hareket sırasında kısa
süreyle ayak–diz destek aralığının dışına çıkabiliyor. En küçük geçiş payı
koşullara göre -6.47 ile -6.02 px arasında. Bu nedenle sonuç, her an statik
dengede bir yükseliş veya tam biyomekanik doğrulama olarak sunulmamalı.
Motor destekli prototip geçişin ardından doğrulanan sonuç, iki noktadan
kararlı destek ve ellerin tamamen serbest kalmasıdır.

Son video H.264, 1280×820, 30 FPS, 42 saniye / 1260 karedir. Bütün kareler
okunarak doğrulandı; yükseliş başlangıcı, ellerin ayrılması ve son pozdan
kareler görsel olarak incelendi. Basan ayakkabının görünür tabanı zeminde,
eller son pozda açıkça yerden ayrıdır.
