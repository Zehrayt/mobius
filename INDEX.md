# Dizin — yol haritası maddesi ↔ dosya eşlemesi

Bu dosya, kullanıcı geri bildirimi üzerine eklendi: yol haritasındaki
(README'deki "Yol haritası" bölümü) madde numaraları ile `demo/` altındaki
dosya adları BİREBİR örtüşmüyor (ör. `step8_collision_friction.py` hem
madde 8'i hem madde 11'i kapsıyor). Bu tablo, hangi maddenin hangi
dosya(lar)da yaşadığını ve neden bazı numaraların "kendi dosyası" olmadığını
açıkça gösteriyor.

## Madde → dosya tablosu

| Yol haritası maddesi | Demo dosyası | Durum |
|---|---|---|
| 1. Verlet zinciri (organik vs. robotik) | `demo/step1_verlet_chain.py` | ✅ |
| 2. FABRIK + foot-planting yürüyüş | `demo/step2_leg_reach.py` | ✅ |
| 3. Tam iskelet (gövde+baş+kol+bacak) | `demo/step3_full_skeleton.py` | ✅ |
| 4. Yürüyüş ince ayarı (karşı-kol, gravity/friction) | `demo/step4_gait_tuning.py` | ✅ |
| 5. Parallax arka plan + bacak erişilebilirlik düzeltmesi | `demo/step5_parallax.py` | ✅ |
| 6. Senaryo/diyalog sistemi | **yok** | ⏳ henüz başlanmadı — bu yüzden `step6_*.py` diye bir dosya YOK (silinmedi, hiç yazılmadı) |
| 7. Momentum ve esneme (squash & stretch) | `demo/step7_squash_stretch.py` | 🔶 |
| 8. Çoklu nokta çarpışması (zemin) | `demo/step8_collision_friction.py` | 🔶 (11 ile birleşik, aşağıya bkz.) |
| 9. Aktif/pasif ragdoll harmanı | `demo/step9_ragdoll_blend.py` | 🔶 |
| 10. İkincil fizik nesneleri (rüzgar/pelerin) | `demo/step10_secondary_wind.py` | 🔶 |
| 11. Dinamik sürtünme/tutunma | `demo/step8_collision_friction.py` | 🔶 **madde 8 ile AYNI dosyada** — bilinçli: ikisi de aynı `collide_ground()` mekanizmasını kullanıyor, biri olmadan diğerini anlamlı göstermek zordu (bkz. o dosyanın kendi docstring'i) |
| 12. Kütle merkezi (center of mass) dengesi | `demo/step12_balance.py` | 🔶 |
| 13. Tam entegrasyon stres testi | `demo/step13_full_integration_test.py` | 🔶 (yol haritasında numaralı bir "madde" değil, ayrı bir doğrulama/stres testi — bkz. aşağıdaki not) |

**Özetle:** `step6` yok çünkü 6 numaralı iş hiç başlamadı; `step11` yok
çünkü 11 numaralı iş `step8` ile aynı dosyada yaşıyor. Hiçbir demo dosyası
silinmedi/kaybolmadı.

## `step13_full_integration_test.py` neden farklı

Diğer tüm `stepN` dosyaları yol haritasındaki NUMARALANMIŞ bir maddeye
karşılık geliyor (yeni BİR mekanik, izole bir demoda). `step13` farklı bir
amaca hizmet ediyor: yeni bir mekanik TANITMIYOR, bunun yerine 7, 8, 9, 10,
11, 12 numaralı maddelerin HEPSİNİ AYNI ANDA, aynı karakter üzerinde
çalıştırıp birbirleriyle çakışıp çakışmadığını (ör. ragdoll blend'in
squash&stretch ile, rüzgarın denge mekanizmasıyla) ve farklı `dt`/FPS
değerlerinde stabil kalıp kalmadığını sınayan bir ENTEGRASYON/stres testi.
Bu yüzden ayrı bir yol haritası maddesi numarası almadı; bkz. README'deki
"Master entegrasyon testi" notu.

## `physics/` çekirdek modülleri (mekanik ↔ modül)

Kullanıcının "sorumlulukların ayrılması" (separation of concerns) geri
bildirimi üzerine, daha önce demo dosyalarında hardcode olan bazı mantık
çekirdek modüllere çıkarıldı:

| Modül | İçerik | Kullanan demo(lar) |
|---|---|---|
| `physics/verlet.py` | `VerletSystem` (nokta/çubuk motoru, gravity/wind/friction, `add_stick(compliance=...)`), `clamp_direction()` | hepsi |
| `physics/fabrik.py` | `FabrikChain2D`, `clamp_joint_angles()` | step2-9, 12, 13 |
| `physics/gait.py` | `FootPlantingLeg` | step2-9, 12, 13 |
| `physics/collision.py` | `collide_ground(body, floor_fn, friction_fn)` — VerletSystem'den bağımsız zemin çarpışması | step7, 8, 9, 13 |
| `physics/environment.py` | `Terrain` (zemin + bölgesel sürtünme), `GustWind` (düzensiz rüzgar) | step7, 8, 9, 10, 13 |
| `physics/ragdoll.py` | aktif/pasif blend yardımcıları (`blend_point`, `blended_max_angle`, `blended_friction`, `driver_follow_target`) | step9, 13 |
| `physics/balance.py` | CoM-destek farkı + orantılı kol tepkisi (`upper_body_com_x`, `support_x`, `counter_balance_offset`) | step12, 13 |

Bu refactor SIRASINDA hiçbir sayısal davranış değişmedi -- her taşınan
mantık, taşınmadan önceki/sonraki demo çalıştırmalarının BİREBİR aynı
sayısal çıktıyı ürettiği doğrulanarak (regresyon) yapıldı (bkz. git commit
mesajı).
