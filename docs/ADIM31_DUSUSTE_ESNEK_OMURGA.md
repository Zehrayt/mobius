# Adım 31 — Yalnızca düşüşte esneyen omurga

İkinci aşama tamamlandı: yürüyüşteki tek kalça–omuz çubuğu korunur; çöküş
anında ortasına bir bel noktası eklenir. Koruyucu kol refleksi iki
karşılaştırma koşulunda da açıktır.

## Uygulama

- `physics/spine.py`: iki 27.5 px gövde parçası, kütle ağırlıklı bel menteşesi,
  sınırlı yay/sönüm sürüşü ve açı sınırları.
- Bel açısı için −30° / +75° sınırları kullanılır. Bunlar animasyon modelinin
  sınırlarıdır; anatomik doğrulama iddiası taşımaz.
- Eski kalça–omuz çubuğu gerçekten kaldırılır; yeni eklemi kilitleyen çapraz
  çubuk bırakılmaz. Boyun referansı üst gövde parçasından okunur.
- Bel, kemikler, dirsekler ve zemin aynı çözüm döngüsündedir. Yeni topoloji
  64 tur ister; Adım 30 karşılaştırması 16 turunu korur. Böylece tur sayısının
  etkisi de dahil olan **tam uygulamalar** karşılaştırılır.
- Gövde görseli mevcut gömlek varlığının iki maskelenmiş yarısından çizilir.
  İki yarı belde birleşir; baş üst, pelvis alt gövde yönünü izler. Varlık
  dosyaları değiştirilmez. Alt video panelinde gerçek fizik eklemleri görünür.

Kullanım:

```python
ActiveBipedSim(gravity_mode="legacy")  # Adım 31: koruyucu kollar + esnek omurga
ActiveBipedSim(gravity_mode="legacy", articulated_spine=False)  # Adım 30: koruyucu kollar, rijit gövde
ActiveBipedSim(gravity_mode="legacy", bracing=False, articulated_spine=False)  # Adım 29
```

Adım 30 karşılaştırma komutu eski koşullarını açık parametrelerle korur.

## Geçişte korunanlar ve enerji sınırı

Bel için dışarıdan yeni kütle eklenmez. İki gövde uç noktasından eşit miktar
kütle alınarak orta noktaya aktarılır: standart modelde 0.1 + 0.1 → 0.2.
Eski noktalar yer değiştirmez; toplam kütle, kütle merkezi, doğrusal ve açısal
momentum korunur. İç bel kuvvetleri de net doğrusal/açısal momentum üretmez.

**Kinetik enerji tam korunmaz.** Kütleyi orta noktaya taşımak dönme ataletini
azaltır. Açısal momentumu koruyan hız düzeltmesi referans geçişte gövde
kinetik enerjisini 287.153 → 292.561 motor birimine çıkarır (**%1.88**).
Dolayısıyla “geçişte bütün nokta hızları/enerji birebir korunuyor” denemez.
Bu ölçüm raporda `transition` alanında saklanır. Birim testleri kütle merkezi
ve iki momentumun korunduğunu ayrıca denetler.

## Referans koşu

150 px itki, 7. saniye, 420 kare. İki koşulda da çöküş kare 222'de ve
koruyucu kol refleksi aynı karede başlar. Çöküş öncesi hareket birebir aynıdır.

| Ölçüm | Adım 30: rijit gövde | Adım 31: esnek omurga |
|---|---:|---:|
| İlk el teması | 229 | 229 |
| İlk baş teması | 235 | 236 |
| İlk göğüs teması | 242 | 243 |
| Baş tepe temas hızı (px/kare) | 13.036 | 11.918 (%8.58 azalma) |
| Göğüs tepe temas hızı (px/kare) | 5.081 | 5.057 (%0.47 azalma) |
| En büyük bel bükülmesi | 0° | yaklaşık 30° |
| Son bel açısı | 0° | −14.48° |
| Gövde parçası en büyük boy hatası | — | 0.276 px |
| Son 60 karede en hızlı nokta (px/kare) | 0.000848 | 0.001255 |
| Son 60 karede kalça kayması | 0.00237 px | 0.00217 px |

Temas hızları, zemin çözücüsüne giren pozitif düşey hızlardır. Baş zemine
hâlâ temas eder; bu veriler bir yaralanma veya gerçek biyomekanik darbe ölçümü
değildir. PBD darbe paylaşımının sınırları Adım 30 raporundakiyle aynıdır.

## 36 koşuluk tarama

±2 / ±2.5 m/s karşılığı impulslar ve ±150 px; her biri 0, 5, 10, 15, 20, 25
kare faz kaydırmasıyla çalıştırılır. Karşılaştırma koşulları aynı çöküş
karelerini verir; 36 koşunun 19'unda çöküş vardır.

- 19 çöken koşunun tamamında noktalar zemin üstündedir ve sistem sonlu kalır.
- Son 60 karede en yüksek nokta hızı **0.00812 px/kare**, kalça kayması
  **0.1608 px** altındadır.
- Bel açıları **−30.012°…75.001°** aralığında kalır. Çözüm toleransı dahil
  en büyük gövde segmenti boy hatası **0.431 px** altındadır.
- Baş tepe temas hızı 19 koşunun **12'sinde**, göğüs hızı **10'unda** azalır.
  Her koşulda daha düşük darbe iddiası yoktur. Örneğin −150 px, +25 kare fazda
  baş hızı 13.30 → 18.58 px/kare artar.

Bu aşamanın ana kazanımı gövdenin iki fiziksel parçayla bükülmesi ve bu
hareketin kararlı çözülmesidir. Düşüş korumasının her yönde iyileştirilmesi
ayrı bir kontrol ayarı gerektirir.

## Yeniden üretme ve inceleme

```bash
python3 demo/step31_spine.py --sweep
python3 -m unittest discover -s tests
python3 tests/smoke_demos.py
```

- `outputs/step31_spine_comparison.mp4`: 12 saniye normal hız + 3 kat yavaş
  düşüş tekrarı, toplam 16.6 saniye, 1280×820, 30 FPS.
- `outputs/step31_spine_comparison.png`: karşılaştırma karesi.
- [36 koşunun ham ölçümleri](validation/step31_spine_report.json).

Yeni testler: geçişte kütle/konum/kütle merkezi/momentum; bel kuvvetlerinin
momentumu koruması; açı sınırları; çöküş öncesi birebir eşitlik; 30 saniye
itkisiz yürüyüş; kemik boyları ve durgunluk; iki parçalı deri bağlantısı.
Tam test paketi **108/108 geçti**. Videonun 498 karesinin tamamı yeniden
okunarak boyut, kare sayısı ve 30 FPS doğrulandı.
Mevcut fizik demoları da `tests/smoke_demos.py` denetiminden geçti.

Sıradaki aşama **tek yerçekimi değeri kararı ve yürüyüşün buna göre
ayarlanmasıdır**. Bu aşamada yerçekimi geçişi ve ayağa kalkma değiştirilmedi.
