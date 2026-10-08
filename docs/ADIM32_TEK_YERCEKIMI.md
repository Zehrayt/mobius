# Adım 32 — Tek yerçekimi

Varsayılan biped artık yürürken, denge kaybederken ve düşerken aynı
**9.81 m/s²** yerçekimini kullanır. Motorun 184 px = 0.9 m ve 30 Hz
ölçeğinde bu **2.228444 px/kare²** değeridir. Çöküşte ivme değiştirilmez.

Bu aşama fizik koşullarını tutarlı hale getirir; her itkiye daha dayanıklı
bir karakter üretmiş değildir. Yeni ölçekte güçlü itkilere dayanıklılık
azaldı. Bu sonuç aşağıda ve ham raporda açıkça korunur.

## Değişiklikler

`physics/gravity.py` tek değer kaynağıdır. `GravityPolicy` şu tüketicileri
aynı değere bağlar:

- Gövde, kol, dinamik bacak ve omurga Verlet integrasyonu.
- Salınan bacağın ters dinamik yükü.
- Yakalama adımının yerçekimine karşı kullanabildiği kalça torku.
- Diz kuvveti / ağırlık dengesi ve bacak uzatma kontrolü.
- Koruyucu kol refleksinin yere temas süresi tahmini.

Adım 31'de kas yükü zaten gerçek ölçekliydi; gövde, bacak yükü ve yakalama
servosu ise 0.065 kullanıyordu. Bu fark giderildi. Kalça ve dizin mevcut
kuvvet/tork tavanları yükseltilmedi.

Yürüyüş yeniden ayarlandı:

| Ayar | Tarihsel sürüm | Tek yerçekimi |
|---|---:|---:|
| Yürüme yerçekimi | 0.065 | 2.228444 |
| Düşme yerçekimi | 2.228444 | 2.228444 |
| Capture-point frekansı | ampirik 0.045 | sqrt(g/184) = 0.110050 |
| Adım bırakma payı | 60 px | 35 px |
| Yeni ayak hedef payı | −15 px | 0 px |
| Tutunma hesabının ivme referansı | itki tavanı 1.5 | aynı yerçekimi 2.228444 |

Güçlü itkide yeni bir geçiş eksikliği ortaya çıktı: kalça, bir yakalama
ayağı basmadan önce zemine doğru düşebiliyordu. Eski yol yalnızca `fell`
işaretini koyuyor, dinamik yere yığılmayı başlatmıyordu. Tek yerçekimi yolunda,
öngörülen kalça yüksekliği mevcut dizin en fazla bükülme yüksekliğine
ulaştığında aynı ragdoll geçişi açılır (`support_height`). Kuvvet yetersizliği
nedeniyle olan eski geçiş (`leg_force`) de korunur. Böylece tek başına bir
etiket değişikliği yerine zemin, koruyucu kollar ve omurga birlikte devreye girer.

## Uyumluluk ve çalışma hızı

> **Birleşim notu:** Bu satırlar bu belgenin yazıldığı daldaki API'dir. main'de varsayılan
> koruyucu refleks Zehra'nın kol kolonu refleksidir; aynı sonuçları üretmek için
> `bracing="impulse"` (pasif koşu için `bracing=False, fall_solver="gn"`) ekleyin.
> Ayrıntı: README "Adım 30–39 birleşimi".

```python
ActiveBipedSim()  # tek yerçekimi, koruyucu kollar, esnek omurga
ActiveBipedSim(gravity_mode="legacy")  # Adım 31'i tekrar üretir
ActiveBipedSim(gravity_mode="legacy", articulated_spine=False)  # Adım 30
ActiveBipedSim(gravity_mode="legacy", articulated_spine=False, bracing=False)  # Adım 29
```

`legacy` yalnızca tarihsel karşılaştırma içindir. Adım 30/31 karşılaştırma
komutları bu seçeneği açıkça kullanır. Önceki testlerin tarihsel koşulları
aynı seçenekle korunur; beklentileri gevşetilmedi veya silinmedi. Yeni
varsayılan davranış ayrıca test edilir.

Bu kontrolcü **30 Hz simülasyon** için ayarlıdır. Tek yerçekimi modunda
başka `fps` verilmesi açık hata üretir; bütün denetleyiciler henüz zaman
adımından bağımsız değildir. Video yeniden örnekleme ayrı bir işlemdir.

## Bir dakikalık yürüyüş

İtki kapalı, 1800 kare; hız ortalaması son 900 kareden alınır.

| Koşul | Ortalama kalça hızı | Adım sayısı | Düşüş |
|---|---:|---:|---:|
| Eski sürüm, buz bölgesi dahil | 1.902 px/kare | 122 | yok |
| Tek yerçekimi, buz bölgesi dahil | 1.923 px/kare | 150 | yok |
| Tek yerçekimi, kuru zemin | 1.921 px/kare | 149 | yok |

Hedef 2 px/karedir. Anlık kalça hızı dalgalanır (yeni sürüm standart sapması
1.015 px/kare); ortalama hız sabit bir hız eğrisi anlamına gelmez. Topuk,
tam taban ve parmak ucu fazları görülür. En düşük kalça yüksekliği 165.1 px;
yeni kuru zemin koşusunda kayma kaydı yoktur.

## Güçlü itki taraması — açık gerileme

Önceki aşamalarla aynı ±2 / ±2.5 m/s karşılığı impulslar ve ±150 px,
altı farklı adım fazında: **36 koşu**.

- Adım 31'de 19/36, tek yerçekiminde **36/36 çöküş** oluştu.
- Tüm yeni koşular sonlu kaldı ve zemin ihlali olmadı.
- Çöküşlerin tamamı yerleşti: son 60 karede en yüksek nokta hızı
  **0.00865 px/kare**, kalça kayması **0.0916 px** altında.
- Yerçekimi bütün koşuların bütün karelerinde **2.228444** kaldı.

Buradaki m/s etiketleri nominal toplam kütleye göre çevrilen kalça
impulslarını ifade eder; karakterin ölçülmüş bütün-gövde hızının tam olarak
o kadar değiştiği anlamına gelmez.

### Daha küçük itkiler

Her büyüklük altı fazda çalıştırıldı; koşular 900 kare sürdü. Varsayılan
3. saniyedeki küçük tökezleme de açıktır.

| Nominal itki | Düşüş / 6 |
|---|---:|
| +0.25 m/s | 0 |
| −0.25 m/s | 0 |
| +0.5 m/s | 0 |
| −0.5 m/s | 4 |
| +1.0 m/s | 1 |
| −1.0 m/s | 6 |

Özellikle geri yöndeki itkilere dayanıklılık sınırlıdır. Bu sürümün kuvvet
ve adım kontrolünün her itki için yeniden optimize edildiği iddia edilmez.

## Referans video: 150 px itki

| Ölçüm | Adım 31 | Tek yerçekimi |
|---|---:|---:|
| Çöküş karesi | 222 | 218 |
| Geçiş nedeni | bacak kuvveti | destek yüksekliği |
| İlk el teması | 229 | 219 |
| İlk baş teması | 236 | 222 |
| Baş tepe temas hızı | 11.918 | 24.199 px/kare |
| Göğüs tepe temas hızı | 5.057 | 8.135 px/kare |

Yeni sürüm daha erken ve daha sert düşer. Buradaki hızlar zemin çözücüsü
girişindeki pozitif düşey hızlardır; yaralanma veya ölçülmüş gerçek darbe
kuvveti değildir. Darbe artışı gizlenmemiştir.

## Yeniden üretme

```bash
python3 demo/step32_gravity.py --sweep
python3 -m unittest discover -s tests
python3 tests/smoke_demos.py
```

- `outputs/step32_gravity_comparison.mp4`: 12 saniye normal hız + düşüşün
  üç kat yavaş tekrarı; 16.6 saniye, 1280×820, 30 FPS. Alt panellerde
  karede kullanılan yerçekimi görünür.
- `outputs/step32_gravity_comparison.png`: karşılaştırma karesi.
- [Yürüyüş ve itki ölçümleri](validation/step32_gravity_report.json).

Yeni testler; tüm tüketicilerin aynı değeri kullanmasını, çöküş öncesi ve
sonrası serbest düşüş ivmesini, yerçekiminin yakalama torkuna etkisini,
tarihsel modu, bir dakikalık kuru/buzlu yürüyüşü, küçük itkiyi ve ±300 px'e
kadar sert itkilerde dinamik düşüş geçişini denetler.
Tam test paketi **116/116 geçti**. Videonun 498 karesinin tamamı yeniden
okundu; 1280×820 boyut ve 30 FPS doğrulandı.
Mevcut fizik demoları da `tests/smoke_demos.py` denetiminden geçti.

## Kalan sınırlar ve sonraki aşama

Motor hâlâ IK/Verlet birleşimidir. Normal bacak salınımı ve destek boyu
kontrolü tamamen kuvvetle çözülen bir kas-iskelet sistemi değildir. Bu aşama
bu sınırı değiştirmez. Omurga geçişinin enerji sınırı da Adım 31'deki gibidir.

Sıradaki aşama **ayağa kalkma**dır. Tek yerçekimi, bu geçişte farklı iki
ivme arasında gidip gelme sorununu kaldırmıştır; itki dayanıklılığındaki
kayıp ise ayrı ve açık bir kontrol ayarı ihtiyacı olarak durmaktadır.
