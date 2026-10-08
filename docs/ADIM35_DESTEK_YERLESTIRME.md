# Adım 35 — Bacakları ve eksik diz temasını yerleştirme

Adım 34'ün iki engeli ele alındı: öne uzanmış bacakların destek pozuna
geçememesi ve bazı düşüşlerde bir dizin zeminin hemen üzerinde kalması.
`ground_recovery=True, recovery_reposition=True` ile açılır. Eski Adım 34
aynı seçeneklerle yeniden üretilebilir; yeni seçenek varsayılan kapalıdır.

## Geçiş

İlk durulma koşulları korunur. Öne uzanmış alt bacak varsa, bacaklar sırayla
üst taraftan dolaştırılarak arkaya alınır. Kısa açısal yol zemine bastırdığı
ve dizin izin verilen bükülme yönüyle uyuşmadığı için kullanılmaz. Hareket
240 karelik yumuşak hedef geçişidir. Diğer bacak başlangıç yönünü korumaya
çalışır; gövde destek için yönlenir. Sonraki bacağa geçmek için en az 240
kare geçmeli, ayak dizin arkasında olmalı ve tüm serbest noktaların hareketi
0.15 px/karenin altına inmelidir. Bu şartlar 600 karede sağlanmazsa motorlar
kapanır ve `leg_placement_timeout` kaydedilir.

Bacaklar yerleştikten sonra önceki el–diz yükselişi uygulanır. Havada kalan
dizin uyluk hedefi, zemin açıklığı başına 15 derece ve en fazla 25 derece
olmak üzere dik yöne düzeltilir. Mevcut 480 tork sınırı korunur. Böylece
zemine değmeyen dizi temas etmiş saymak yerine gerçek temas oluşturulur.

Başarı koşulu değişmez: güncel zemin çözümünde iki el ve iki diz teması,
yükselmiş kalça/göğüs, destek aralığında kütle merkezi ve 30 sakin kare.
Destek kaybı başarı durumunu geri alır. Yükselişin ayrıca 600 kare sınırı vardır.

Bu, fizik motoru üzerinde bir bacak yerleştirme prototipidir; anatomik bir
yan yuvarlanma animasyonu değildir. Hareket yavaştır. Motorlar eşit/zıt hız
dürtüleri uygular; konumlar doğrudan taşınmaz, yeni sabitleme eklenmez,
yerçekimi değişmez. Dünya yönüne referans veren motorlar tam bir kas/açısal
momentum modeli değildir. Başın görsel bağlantısı bu adımda değiştirilmedi.

## İnceleme

```sh
python3 demo/step35_support_placement.py --sweep
python3 -m unittest discover -s tests
python3 tests/smoke_demos.py
```

Video: `outputs/step35_support_placement_comparison.mp4`. Aynı -150 px,
faz 0 düşüşünde Adım 34 ve 35 yan yana; üstte karakter, altta gerçek fizik
noktaları gösterilir. Simülasyonun 6–40. saniyeleri normal hızda gösterilir.
Tarama önceki aynı 36 darbe/faz çiftini kullanır. İki bacağın sırayla
hareketine zaman tanımak için gözlem 1000 kareden 1500 kareye uzatılmıştır;
eski ölçümler 33.3 saniye, yenileri 50 saniyedir. Referans karşılaştırmasının
iki kolu da 1500 kare çalışır. Tam ayağa kalkma ve yürüyüşe dönüş henüz yoktur.

## Ölçülen sonuçlar

- 36/36 koşulda kararlı dört noktalı destek; son 60 kare boyunca korunuyor.
  Adım 34'te 16 destek, 13 dönme bekleyen ve 7 başarısız koşul vardı.
- Bütün yeni koşullar kare 454–967 arasında desteğe ulaştı (15.13–32.23 s).
  Yani ilk başarıların tamamı eski 1000 karelik gözlem penceresine de sığıyor;
  ek süre kararlı beklemeyi doğruluyor. Hiçbirinde destek kaybı kaydedilmedi.
- Bütün koşullar sonlu, noktalarda zemin altına geçiş 0 px. Yerçekimi sabit
  2.228444 px/kare². Tork sınırı kullanım oranı en fazla 1.0, uzama kuvveti
  oranı 0.09627. Son 60 karede en yüksek nokta hareketi 0.00001311 px/kare;
  kütle merkezinin destek kenarına uzaklığı en az 50.32 px.
- Geçişlerde en yüksek tek nokta hızı 15.56 px/kare. Bu sonuç yalnızca bu
  36 düz zemin denemesi içindir; farklı başlangıç pozları ve engeller için
  genel başarı iddiası yoktur.

-150 px/faz 0 referansı: çöküş 219, ilk bacak 308, ikinci bacak 548,
yükseliş 788, kararlı destek 934. Aktif yerleşme ve yükseliş toplam
20.87 saniye. Son durumda kalça 82.29 px, göğüs 49.55 px yüksekte;
kütle merkezi destek sınırından 53.67 px içeride.

Makine okunur sonuçlar: `docs/validation/step35_support_placement_report.json`.

Doğrulama: 136 testlik tam koşuda 135 test geçti; yeni diz dalı testinin
sıfıra yakın çapraz çarpım beklentisi, ilk zemin çarpışmasında çözücünün
-0.0173° artık hatasını reddetti. Kontrol 0.05° açısal toleransla düzeltildi;
fizik değiştirilmedi. Yeni dört test yeniden çalıştırıldı ve tamamı geçti.
Eski demo çalıştırma kontrolleri de geçti. Video H.264, 1280×820, 30 FPS,
34 saniye / 1020 kare; bütün kareleri okunarak doğrulandı. Geçiş ve son
pozdan örnek kareler görsel olarak incelendi.
