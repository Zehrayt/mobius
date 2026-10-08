# Adım 36 — Bir ayağı yerleştirme ve yük aktarma

Bilge, Adım 35'in kararlı el–diz desteğinden bir ayağını gövdesinin altına
alır, o bacağa kontrollü itiş uygular ve ellerinden/diğer dizinden destek
alarak bekler. Bu aşamada eller yerden kalkmaz; tam doğrulma veya yürüyüşe
dönüş yapılmaz. Sol bacak öne alınır; sağ/sol seçimi henüz uyarlanabilir değildir.

`ActiveBipedSim(ground_recovery=True, recovery_reposition=True,
recovery_transfer=True)` ile açılır. Yeni seçenek varsayılan kapalıdır.
Önceki aşamaların fiziği, kararlı destekten sonraki 60 karelik bekleme
bitene kadar birebir korunur. Başın görsel bağlantısı değiştirilmedi.

## Geçiş ve başarı şartları

1. `supported`: Adım 35'in dört gerçek temas ve denge kontrolünden geçtikten
   sonra 60 kare beklenir. Destek bozulursa önceki aşama tekrar çalışır.
2. `foot_placing`: sol bacağın hedef yönleri 240 karelik yumuşak geçişle
   değiştirilir. Sağ diz ve eller destek sağlar. En az 240 kare geçmeden
   yük aktarımına başlanmaz.
3. `transferring`: kalça–sol ayak çiftine, 115 px açıklık hedefli ve en
   fazla 2 motor kuvvet birimi büyüklüğünde eşit/zıt dürtüler uygulanır.
   İtiş 120 karede yumuşakça açılır. Bu, kapalı zincirde bacak uzaması için
   basitleştirilmiş bir motor modelidir; konumları doğrudan değiştirmez.
4. `half_kneeling`: destek ve aktarım şartları 30 kesintisiz sakin kare
   sağlanırsa ellerle desteklenen yarım diz çökme kabul edilir. Motorlar
   sınırlı itişle pozu korur; noktalar sabitlenmez.

Yerleştirme ve aktarma için **iki el, sağ diz ve sol ayak** güncel zemin
çözümünde gerçekten temas üretmelidir. Kütle merkezi bu destek noktalarının
yatay aralığında kalmalı; öndeki dizin altı zeminden 25 px'den fazla ayrılmalı;
kalça 65 px, göğüs 30 px üzerinde olmalıdır. Ayak yatayda kalçanın 50 px,
kütle merkezinin 20 px yakınında bulunmalıdır. Bütün serbest noktaların
kareler arası hareketi 0.15 px/kare altında olmalıdır.

Aktarımda ayrıca sol ayağın toplam temas düzeltmesindeki payı en az 0.25
olmalı ve yerleştirme sonundaki değerinden en az 0.08 artmalıdır. Bu pay,
çözücünün kütleyle ağırlıklandırılmış zemin düzeltmelerinden hesaplanan bir
**sayısal temas göstergesidir; gerçek kuvvet veya vücut ağırlığı yüzdesi
olarak yorumlanmamalıdır.** Kütle merkezi, gerçek temas, diz açıklığı ve
kayma kontrolleriyle birlikte kullanılır; yalnızca görünüş yeterli değildir.

Yerleştirme ve aktarımın ayrı ayrı 600 kare sınırı vardır. Süre dolarsa
başarı kaydedilmez; motorlar kapanır. Son pozda destek kaybolursa başarı
kaldırılır, aktarım yeniden denenir ve aynı süre sınırına tabidir.

## Fizik ve kapsam

Yeni kontrol `physics/foot_transfer.py` içinde önceki destek kontrolünü
kullanır. Mevcut tork sınırları korunur. Kalça–ayak motoru yalnızca hız
geçmişine eşit/zıt dürtü ekler: yeni sabitleme, ışınlama veya yerçekimi
artışı yoktur. Dünya yönüne referans veren duruş motorları önceki gibi tam
bir kas/açısal momentum modeli değildir. Hareket hâlâ yavaş bir prototiptir.
Düz zemindeki bu pozdan sonra elleri boşaltma ve gövdeyi yükseltme gerekir.

## Yeniden üretme

```sh
python3 demo/step36_foot_transfer.py --sweep
python3 -m unittest discover -s tests
python3 tests/smoke_demos.py
```

- `outputs/step36_foot_transfer_comparison.mp4`: aynı +150 px/faz 0 düşüşünde
  Adım 35 ve 36. Normal hızda simülasyonun 6–40. saniyeleri; üstte karakter,
  altta fizik noktaları, temas göstergesi ve kütle merkezi–ayak farkı.
- `outputs/step36_foot_transfer_comparison.png`: son destek pozları.
- `docs/validation/step36_foot_transfer_report.json`: referans ve tarama.

Tarama, Adım 35'teki aynı 36 güçlü darbe/faz çiftini kullanır. Gözlem süresi,
önceki yerleştirme ve yeni aktarımı kapsamak için 1800 kareye (60 saniye)
çıkarılmıştır. Başarı, son 60 kare boyunca korunmasıyla ayrıca doğrulanır.
Bu sonuçlar farklı zeminler, engeller veya bütün başlangıç pozları için
bir genelleme değildir.

## Ölçülen sonuçlar

- **36/36** koşul el destekli yarım diz çökmede tamamlandı; tümünde son
  60 kare boyunca dört destek korundu. Aktarım sonrası destek kaybı olmadı.
- Kararlı son poz kare 932–1445 arasında oluştu (31.07–48.17 s).
- Kütle merkezi ile basan ayağın yatay farkı 2.15–3.40 px; öndeki dizin
  yerden açıklığı 55.10–56.20 px. Kütle merkezi destek kenarından en az
  40.94 px içeride kaldı.
- Son 60 karede en yüksek nokta hareketi 0.000000572 px/kare; basan ayağın
  toplam yatay hareketi en fazla 0.0000194 px. Bütün koşullar sonlu kaldı,
  zemin altına geçiş olmadı; kuvvet/tork sınırları aşılmadı.
- Ayağın sayısal temas payı son durumda 0.361–0.371. Referansta destek
  pozundaki 0.030'dan, ayak yerleştirilince 0.155'e, aktarım sonunda
  0.365'e çıktı. Bunlar gerçek vücut ağırlığı oranları değildir.

+150 px/faz 0 referansı: ilk el–diz desteği kare 454; ayağı yerleştirme 514;
yük aktarımı 783; kararlı son poz 932. Yeni yerleştirme/aktarma 13.93 saniye
sürüyor. Son kalça yüksekliği 89.84 px, göğüs yüksekliği 48.10 px. Yerçekimi
2.228444 px/kare² olarak korunuyor. Hareket hâlâ yavaş ve eller destek veriyor.

142 testlik tam koşu geçti. Bunlar eski aşamayla başlangıç fiziğinin birebir
eşitliğini, iki düşüş yönünde kararlı desteği, kuvvet/tork sınırlarını,
konumların doğrudan değişmemesini, net doğrusal dürtünün sıfır olmasını,
ayak teması kaybında başarının kaldırılmasını, aktarım artışı olmadan
başarı verilmemesini ve deneme süresi sınırını kapsıyor. Eski demo
çalıştırma kontrolleri de geçti.

Görsel incelemede öne alınan sol bacağın gömlek arkasında gizlendiği görüldü;
yalnızca bu yeni durumlarda sol bacak gömleğin önünde çizilir. Fizik
noktaları ve baş bağlantısı aynı kalır. Alttaki iskelet gerçek fizik
noktalarını ayrıca gösterir.

Son çizim sırası değişikliğinden sonra mevcut iki deri/temas testi yeniden
geçti. Karşılaştırma videosu yeniden üretildi; 1020 karesinin tamamı okundu
(1280×820, 30 FPS, 34 saniye, H.264). Yerleştirme, aktarım ve son pozdan
kareler görsel olarak incelendi. Büyük baş/kol çizimleri öndeki bacağı
kısmen örttüğü için hareket alttaki fizik iskeletinde daha açık görülür;
çizim oranlarının iyileştirilmesi bu fizik aşamasının dışında kalır.
