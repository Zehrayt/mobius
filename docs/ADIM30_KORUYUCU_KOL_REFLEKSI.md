# Adım 30 — Koruyucu kol refleksi

Bu aşama, `db45453` üzerindeki pasif düşüşe koruyucu kollar ekler. Kullanıcının
seçtiği ilerleme biçimi: her aşama sonunda video ve ölçümleri incelemek.
Sıra korunuyor:

1. **Koruyucu kol refleksi — bu değişiklik.**
2. Yalnızca düşüşte devreye giren omurga — sonraki incelemeden sonra.
3. Tek yerçekimi değeri kararı ve yürüyüşün buna göre ayarlanması.
4. Ayağa kalkma.

## Davranış

`ActiveBipedSim(gravity_mode="legacy", articulated_spine=False, bracing=True)`
Adım 30 koşulunu yeniden üretir.
`gravity_mode="legacy", bracing=False, articulated_spine=False`, Adım 29'un pasif kol davranışını ve eski düşüş kol
kısıtlarını geri getirir; önce/sonra karşılaştırmasının kontrol koşuludur.

- Yürüyüş denetimi aynıdır. Refleks ancak `collapsed` durumunda çalışır.
- Göğsün yere ulaşma süresi, mevcut düşey hız, zemin mesafesi ve düşüş
  yerçekimiyle tahmin edilir. 12 kare veya daha az kalınca uzanma başlar.
- Omuzun düşüş yönündeki hızı, ellerin yere uzanacağı yönü belirler.
  Omuz açısına sınırlı PD sürüşü uygulanır; hiçbir el yere pinlenmez.
- Omuz–el arasındaki sınırlı yay/sönüm kuvveti kolu uzatır, yere temas
  sonrasında sıkışmaya direnç verir. İki kemik rijit kalırken dirsek bükülebilir.
  Kuvvet tavanı mevcut Hill eğrisiyle ölçeklenir.
- El ve omuza eşit/zıt impuls uygulanır. Kontrolcü konumları doğrudan
  değiştirmez; toplam doğrusal momentumu korur.
- Baş zemine yaklaşınca veya 36 karelik süre dolunca sürüş kapanır.
  Sonrasında pasif yerleşme devam eder; ayağa kalkma davranışı eklenmemiştir.

Düşüşte yürüyüşün tek taraflı dirsek düzeltmesini kullanmak kaymayı
artırıyordu. Bu nedenle yeni yolda üç noktalı, kütle ağırlıklı dirsek sınırı
kemik ve zemin kısıtlarıyla aynı döngüde çözülür. Düşüş çözümü 16 tur kullanır;
eski karşılaştırma koşulu 8 turu korur. Omuz/dirsek sürücüsüyle bu çözücü
**birlikte** değerlendirilmektedir; sonuçlar yalnızca motor torkunun etkisi
olarak yorumlanmamalıdır.

## Referans koşu

150 px itki, 7. saniye; 420 simülasyon karesi. Temas hızları pozitif düşey
**zemin çözücüsü giriş hızlarıdır**, px/kare cinsindedir. Fiziksel yaralanma
veya ölçülmüş biyomekanik kuvvet iddiası taşımaz.

| Ölçüm | Pasif (Adım 29) | Koruyucu kollar |
|---|---:|---:|
| Çöküş karesi | 222 | 222 |
| İlk el–zemin teması | 230 | 229 |
| İlk baş–zemin teması | 235 | 235 |
| İlk göğüs–zemin teması | 234 | 242 |
| Baş tepe temas hızı | 20.434 | 13.036 (**%36.2 azalma**) |
| Göğüs tepe temas hızı | 11.328 | 5.081 (**%55.1 azalma**) |
| Son 60 karede en hızlı nokta | 0.001315 | 0.000848 |
| Son 60 karede kalça kayması | 0.0776 px | 0.0024 px |

Baş hâlâ yere temas eder. Bu refleks darbeyi azaltır; baş temasını tamamen
önlediği iddia edilmez.

## 36 koşuluk tarama

İtkiler: ±2 ve ±2.5 m/s karşılığı motor impulsları ile ±150 px. Her biri
7. saniyeden başlayarak 0, 5, 10, 15, 20, 25 kare faz kaydırmasıyla çalıştırılır.
Her koşul pasif ve aktif olarak ayrı simüle edilir.

- Her iki sürümde de 19 koşu çöküşle sonuçlanır; refleks çöküşten sonra açılır.
- Çöken 19 koşunun **15'inde baş**, **16'sında göğüs** tepe temas hızı azalır.
- Tüm çöken koşular yerleşir: son 60 karede en yüksek nokta hızı
  **0.0642 px/kare**, en yüksek kalça kayması **0.329 px** altındadır.
- Bazı koşullarda hız artar: örneğin +2.5 m/s, +5 kare fazda baş
  13.73 → 15.23; göğüs 4.29 → 6.41 px/kare. -150 px'in bazı fazlarında
  da baş hızı artar. Dolayısıyla her yönde/her fazda daha iyi olduğu söylenemez.

JSON raporu her koşulu ayrı içerir. Temas olmaması `first_frame: null` ile
ayrılır; bu durumda hız alanındaki sıfır, sıfır hızlı bir çarpışma demek değildir.

## Darbe paylaşımı ölçümünün sınırı

`hand_ground_projection_share_pct`, ilk 45 düşüş karesindeki PBD zemin
düzeltmelerinin kütleyle çarpılmış toplamı içinde ellerin payıdır. Referans
koşuda %10.67 → %7.75 çıkar. Bu sayı **omuzdan geçen gerçek darbe yüzdesi
değildir**; çözüm turu, kısıtlar, temas süresi ve statik ağırlık desteğini de içerir.
Baş/göğüs hızları azalmış olsa da bu orandan “yükün şu kadarı kollara aktarıldı”
sonucu çıkarılmamalıdır. `shoulder_actuator_upward_impulse` ise herhangi bir
elin temas algılandığı karelerde kontrolcünün omza verdiği yukarı impulsların
toplamıdır; pasif kemik/eklem tepkilerini içermez. Gerçek omuz darbe yüzdesi
bu motorun mevcut ölçümleriyle doğrulanmış değildir.

## Yeniden üretme

```bash
python3 -m unittest discover -s tests
python3 demo/step30_bracing.py --sweep
```

Çıktılar:

- `outputs/step30_bracing_comparison.mp4`: 12 saniye normal hız + düşüşün
  3 kat yavaş tekrarı; toplam 16.6 saniye, 1280×820, 30 FPS.
- `outputs/step30_bracing_comparison.png`: karşılaştırma karesi.
- `outputs/step30_bracing_report.json`: referans ölçümler ve 36 koşuluk tarama.

Videoda üstte karakter derisi, altta gerçek fizik noktaları bulunur.
Mevcut derinin zemin üstünde tutma düzeltmeleri korunur; fizik temasının
kanıtı alt panel ve sayısal rapordur.

Bu çalışma sırasında eksik Git LFS varlıkları indirildi. Yerel LFS aracı
`.venv/bin/git-lfs` konumundadır; filtre ayarı yalnızca bu depoya yapıldı.
Görseller Git'te değiştirilmedi.

## Doğrulama kapsamı

Tam test paketi: **101/101 geçti**. Üretilen H.264 videonun 498 karesinin
tamamı yeniden okunarak 1280×820 ve 30 FPS olduğu doğrulandı.
`python3 tests/smoke_demos.py` kapsamındaki mevcut fizik demoları da geçti.
36 koşunun ham ölçümleri: [JSON raporu](validation/step30_bracing_report.json).

Yeni testler: çöküş öncesi birebir eşitlik; 30 saniye itkisiz yürüyüşün
birebir eşitliği; referans darbede baş/göğüs hız azaltma; ellerin önce teması;
kontrolcünün doğrusal momentum koruması; dirsek düzeltmesinin kütle merkezi
koruması; refleksin bırakması ve durgunluk; kolsuz mod.
Adım 29'un sayısal kanaryası `gravity_mode="legacy", bracing=False, articulated_spine=False` ile ayrıca korunur.

Omurga hâlâ tek parçadır. İki yerçekimi değeri arasındaki eski geçiş bu
adımda değişmez. Bir sonraki çalışma, bu sonuçların incelenmesinden sonra
**yalnızca düşüşte esneyen omurga** olacaktır.
