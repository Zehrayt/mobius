"""
collision.py -- VerletSystem'den BAĞIMSIZ zemin çarpışma tepkisi.

Refactor notu (kullanıcının "sorumlulukların ayrılması" / separation of
concerns geri bildirimi üzerine)
--------------------------------
`collide_ground()` daha önce `physics/verlet.py` içinde
`VerletSystem.collide_ground()` adlı bir METOD olarak yaşıyordu. Mantık
BİREBİR AYNI kaldı (satır satır buraya taşındı, hiçbir sayı/davranış
değişmedi -- bkz. `demo/step7_squash_stretch.py`, `step8_collision_friction.py`,
`step9_ragdoll_blend.py`'nin bu refactor öncesi/sonrası aynı sayısal
çıktıları), sadece nerede yaşadığı değişti: `VerletSystem` (genel bir
nokta/çubuk fizik motoru) artık "zemin" veya "çarpışma" kavramından hiç
haberdar değil -- bu, ayrı bir "çarpışma sistemi" modülüne taşındı ve
artık bir `body: VerletSystem` parametresi alan bağımsız bir fonksiyon.
Bu, `VerletSystem`'in kendisini (torso/kuyruk/pelerin gibi HERHANGİ bir
nokta-çubuk grafiği için kullanılabilecek) jenerik kalmaya devam
ettiriyor; zemin/bölgesel sürtünme gibi SAHNEYE özgü kavramlar
`physics/environment.py`'deki `Terrain` ile birlikte burada yaşıyor.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from physics.verlet import VerletSystem


def collide_ground(body: "VerletSystem", floor_fn, friction_fn=None) -> None:
    """Serbest (pinned olmayan) her noktayı `floor_fn(x)` ile tanımlı
    zemine karşı çarpıştırır -- bir nokta o x'teki zemin yüksekliğinin
    (y, ekranda aşağı = artış) ALTINA sızarsa yüzeye geri itilir.
    `floor_fn` her nokta için ayrı ayrı çağrılır (kavramsal olarak
    her noktadan aşağı tek bir "raycast"); sabit bir zemin için sabit
    değer döndüren bir lambda yeterlidir, ama x'e bağlı bir yükseklik
    de (basamak/rampa) desteklenir.

    `friction_fn(x)` verilirse, temas eden noktanın yatay hızı
    (points - prev_points farkı) `1 - friction_fn(x)` ile ölçeklenir --
    bu, `VerletSystem.friction`'dan TAMAMEN BAĞIMSIZ, sadece "zemine
    değdiği anda" geçerli ayrı bir sürtünme katsayısıdır ve x'e göre
    bölgesel olarak değişebilir (ör. buzlu bir şerit vs. normal toprak).
    Dikey hız da her zaman sıfırlanır (nokta zeminden "sekmesin" diye).

    Bu, `body.step()` çağrısından SONRA (tıpkı `clamp_direction` gibi)
    elle çağrılır -- bacaklar zaten `FootPlantingLeg` ile zemine hedef-
    güdümlü bağlı olduğu için buna ihtiyaç duymaz; bunun asıl amacı
    kuyruk/pelerin gibi uzun serbest zincirlerin ya da serbest
    parçacıkların (ör. "puck") zeminin altına gömülmeden üzerinde
    durmasını/sürüklenmesini sağlamaktır.

    DÜRÜST BİR SINIR (sayısal olarak doğrulandı, bkz.
    demo/step8_collision_friction.py): `friction_fn` bir noktanın
    HIZINI söndürür, ama bir nokta BAŞKA noktalara sabit uzunlukta
    çubuklarla (stick) bağlıysa, bu hız söndürmesinin çok az etkisi
    olur -- çünkü `_satisfy_sticks()` her karede (hatta her relaksasyon
    iterasyonunda) noktayı komşusuna göre TAMAMEN GEOMETRİK olarak
    yeniden konumlandırır ve bunu yaparken önceki hıza/sürtünmeye hiç
    bakmaz. Sürtünmenin görsel olarak anlamlı bir fark yaratması için
    nokta HİÇBİR çubuğa bağlı olmayan gerçekten SERBEST bir parçacık
    olmalı (bkz. aynı demodaki "taş/puck" -- orada aynı `friction_fn`
    buzlu bölgede çok daha uzun kayma mesafesi olarak açıkça görülüyor)."""
    for i in range(len(body.points)):
        if i in body.pinned:
            continue
        x, y = body.points[i]
        floor_y = float(floor_fn(x))
        if y > floor_y:
            if friction_fn is not None:
                vx = body.points[i, 0] - body.prev_points[i, 0]
                damp = max(1.0 - float(friction_fn(x)), 0.0)
                body.points[i, 0] = body.prev_points[i, 0] + vx * damp
            body.points[i, 1] = floor_y
            body.prev_points[i, 1] = floor_y
