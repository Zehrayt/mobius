# Adım 34 — Yerde durulma ve el–diz desteği

Ayağa kalkmanın ilk alt aşaması: karakter yerde sakinleşir, uygun yatış
pozundan gövdesini kaldırıp elleri ve dizleri üzerinde bekler. Tam ayağa
kalkma, ayağı gövdenin altına yerleştirme ve yürüyüşe dönüş henüz yoktur.

Bu aşama `ActiveBipedSim(ground_recovery=True)` ile açılır. Varsayılanın
kapalı olması önceki düşüş karşılaştırmalarını ve ölçümlerini korur.
Yeni demo bu özelliği açar. Tek yerçekimi, kollar, esnek omurga ve Adım 33
bacak denge düzeltmeleri gerekir. Başın görsel bağlantısındaki kullanıcı
geri bildirimi bu aşamada ele alınmadı; sonraya bırakıldı.

## Durumlar ve geçiş şartları

- `waiting`: çöküşten en az 60 kare sonra, bütün serbest noktaların gerçek
  kareler arası hareketi 0.15 px/karenin altında 30 kesintisiz kare kalmalı;
  koruyucu kol refleksi de bitmiş olmalı. Hareket artarsa sayaç sıfırlanır.
- `needs_roll`: alt bacaklar öne uzanmışsa bu ilk kontrol uygun değildir.
  Dönme/bacakları yeniden yerleştirme aşaması gereklidir. Kontrol kuvveti
  uygulanmaz; mevcut pasif yığılma devam eder. Bu bir başarı sayılmaz.
- `rising`: mevcut eklem yönleri başlangıç alınır; hedef yönler 120 karelik
  yumuşak geçişle değişir. İlk 30 karede motor gücü kademeli açılır.
- `supported`: iki el ve iki diz güncel zemin çözümünde gerçekten temas
  üretmeli; kalça yüksekliği 45 px, göğüs yüksekliği 40 px üzerinde olmalı;
  toplam kütle merkezi bu dört noktanın yatay destek aralığında kalmalı.
  Bütün noktalar 0.3 px/karenin altında 30 kare kalınca destek kabul edilir.
  Destek kaybolursa yeniden `rising` durumuna geçilir.
- `failed`: 600 karelik denemede kararlı destek sağlanamazsa motorlar kapanır.
  Karakter mevcut yerçekimi ve temas çözümüyle tekrar sakinleşir.

## Hareketin uygulanması

`physics/ground_recovery.py` eklem çiftlerine eşit/zıt hız dürtüleri uygular.
Noktaların konumu doğrudan değiştirilmez; hiçbir el/diz/kalça sabitlenmez.
Eski yürüyüş ankrajının çubuğu çöküşte devreden çıktığı gibi kalır; yeni bir
askı veya kinematik sürücü eklenmez. Omurga ve bacak topolojisi korunur.

Motor tork sınırları mevcut motor ölçeğinde gövde 480, her kalça–diz 480,
her diz–ayak 120, her el–omuz 300 ve boyun 100'dür. El–omuz uzama kuvveti
16 ile sınırlıdır. Bunlar bu prototipin açık kontrol parametreleridir;
ölçülmüş insan kası kapasiteleri olarak yorumlanmamalıdır. PD motorları
net doğrusal momentum eklemez; dünya yönünü referans aldıklarından tüm
vücudun açısal momentumunu koruyan eksiksiz kas modeli değildirler.
Gövde duruş kontrolüyle aynı sadeleştirme kullanılır.

Zemin temasının varlığı kısıt çözücüsünün pozitif düzeltmesinden okunur.
Düzeltme toplamı gerçek zemin kuvveti değildir. Gösterilen destek ve kütle
merkezi ölçümleri bir sonraki ayağa kalkma adımını değerlendirmek içindir.

## Yeniden üretme ve inceleme

```sh
python3 demo/step34_ground_support.py --sweep
python3 -m unittest discover -s tests
python3 tests/smoke_demos.py
```

- `outputs/step34_ground_support_comparison.mp4`: aynı +150 px darbede
  pasif yığılma ve yeni destek geçişi. Simülasyonun 6–22. saniyeleri;
  üstte karakter, altta gerçek fizik eklemleri ve durum/temas bilgisi.
- `outputs/step34_ground_support_comparison.png`: destek pozunun karşılaştırması.
- `docs/validation/step34_ground_support_report.json`: referans ve 36 koşul.

Tarama, önceki güçlü darbe karşılaştırmasındaki ±2, ±2.5 nominal m/s ve
±150 px dürtüleri 0,5,10,15,20,25 kare zaman farkıyla uygular. Nominal m/s
etiketi sabit momentum ölçeğidir; ölçülmüş hız sıçraması değildir. Her koşul
1000 kare sürer. Destek başarısı yalnızca son durumda `supported` olan ve
son 60 karede de desteği koruyan koşular için değerlendirilir.

Yeni testler motorlarda konum koruma/net doğrusal dürtü sıfırı, kuvvet/tork
sınırları, durulmadan önce eski fizik ile birebir eşitlik, dört gerçek temas,
kütle merkezi, gövde yüksekliği, kararlı bekleme, durulma sayacının sıfırlanması,
uygunsuz yatış pozunda pasif kalma ve süre sonunda motorların kapanmasını
kapsar. Sonraki hareket bu destek pozundan bir ayağı gövdenin altına almaktır;
geri yatış pozları için ayrıca dönme/yeniden yerleşme geçişi gereklidir.

## Ölçülen sonuçlar

| 36 güçlü darbe / 1000 kare | Koşul sayısı |
|---|---:|
| Kararlı el–diz desteği, son 60 karede de korunuyor | 16 |
| Dönme/yeniden yerleşme gerektiği için motorlar açılmıyor | 13 |
| Deneme başlıyor, dört destek sağlanamıyor ve süre dolunca kapanıyor | 7 |

Bu sonuç **genel bir ayağa kalkma çözümü değildir**. Başarısız yedi koşulda
son ölçümde üç destek noktası vardır; yalnızca gövdenin kısmen yükselmesi
başarı olarak sayılmadı. Dönme ve eksik uzvun yeniden yerleştirilmesi henüz
eksiktir. Sonraki alt aşamaya bu sınır bilinerek geçilmelidir.

Referans +150 px/faz 0: çöküş kare 218 (7.27 s), motor başlangıcı kare 307
(10.23 s), kararlı destek kare 454 (15.13 s). Aktif geçiş 4.90 saniye sürer.
Son durumda kalça 82.27 px, göğüs 49.56 px, baş 62.29 px yerden yüksektedir.
Dört el/diz teması vardır; kütle merkezinin destek aralığı kenarına en küçük
uzaklığı 53.70 px'dir. Son 60 karede en yüksek hareket 3.46e-9 px/kare
altındadır. Geçiş sırasında en yüksek tek nokta hızı 10.51 px/karedir.

Bütün 36 koşul sonlu kaldı; zemin altına geçiş olmadı. Tork sınırı kullanım
oranı en fazla 1.0, uzama kuvveti kullanım oranı 0.0887 altındadır. Yerçekimi
sabit 2.228444 px/kare² (9.81 m/s²) kaldı. 132 test ve eski demo çalıştırma
kontrolleri geçti. Karşılaştırma videosu H.264, 1280×820, 30 FPS / 480 karedir.
