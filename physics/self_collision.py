"""
self_collision.py -- bir Verlet karakterinin İKİNCİL zincirlerini (pelerin,
kuyruk, saç vb.) kendi ANA gövdesinin segmentlerinden iten basit bir
kendi-kendine çarpışma (self-collision) mekanizması.

Neden gerekli (kullanıcı geri bildirimi -- "ikincil animasyon dengesizliği"):
`physics/collision.py`'deki `collide_ground()` sadece ZEMİNE karşı çalışır;
karakterin kendi noktaları/segmentleri arasında (ör. pelerin ile gövde)
HİÇBİR çarpışma kontrolü yoktu (grep ile doğrulandı) -- bu yüzden pelerin,
gövdeyi serbestçe kesip geçebiliyordu (step13'te 360 karenin ~%1-4'ünde
ölçüldü). Bu modül, `collide_ground()` ile AYNI ruhta (bağımsız, `body:
VerletSystem` parametresi alan, sahne/karaktere özel BİLGİ içermeyen genel
bir fonksiyon) minimal bir düzeltme sağlar -- tam bir fizik motoru
kalitesinde kalın-gövde/kapsül çarpışması DEĞİL, nokta-vs-doğru-parçası
itme (point-vs-segment repulsion).

DÜRÜST BİR SINIR (bu fonksiyon için hâlâ geçerli): `push_points_off_
segment()` pelerin/kuyruk gibi İNCE, tek zincirli ikincil nesneler için
yeterli (görsel olarak gövdeyi kesmemesini sağlar), ama KALIN/hacimli bir
çarpışma (ör. iki kalın uzvun birbirine göre hacim kaplaması) modellemez
-- segmentin İKİ UCU arasındaki en yakın noktaya göre tek bir itme
mesafesi (`min_dist`) uygulanır. ÖNEMLİ SINIR: sadece `point_indices`
listesindeki noktaları iter -- segmentin `seg_a_idx`/`seg_b_idx` UÇLARI
HİÇ değişmez (sabit kabul edilir) VE (daha da önemlisi) itilen noktalar
arasındaki ARA BÖLGE (ör. dirsek-el arasındaki ön kol çizgisinin ORTASI)
hiç kontrol edilmez -- sadece uç noktalar (`elbow`, `hand`) tek tek nokta
olarak test edilir.

**Kapsül Tabanlı Öz-Çarpışma (5. tur sonrası kullanıcı isteği --
"iki çizginin birbirine en yakın noktasını hesaplayıp... gövdenin içinden
geçmesini fiziksel olarak imkansız hale getiren matematiksel kısıtlama"):**
`push_segment_off_segment()` (aşağıda) bu sınırı gideriyor -- artık iki
SEGMENTİN (ör. ön-kol: dirsek->el VE gövde: kalça->omuz) aralarındaki GERÇEK
en yakın nokta çiftini (segmentlerin sadece uçları değil, ARALARINDAKİ HER
NOKTASI dahil) hesaplayıp, ikisi de (pinned olmayan taraf) birbirinden
itiliyor. Bu, `push_points_off_segment()`'i GEÇERSİZ KILMIYOR -- pelerin
gibi çok-segmentli ince zincirlerde nokta-bazlı yaklaşım hâlâ makul bir
performans/doğruluk dengesi sunuyor (bkz. aşağıdaki `push_segment_off_
segment()` dokstring'indeki kendi dürüst sınırı) -- iki fonksiyon da
kullanılabilir, hangisinin uygun olduğu çarpışan nesnenin şekline bağlı.
"""
from __future__ import annotations

import numpy as np


def _closest_point_on_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, float]:
    """`p` noktasının `a-b` doğru parçası üzerindeki en yakın noktasını ve
    o noktaya olan projeksiyon oranını (t, [0,1] aralığında) döndürür."""
    ab = b - a
    ab_len_sq = float(np.dot(ab, ab))
    if ab_len_sq < 1e-9:
        return a.copy(), 0.0
    t = float(np.clip(np.dot(p - a, ab) / ab_len_sq, 0.0, 1.0))
    return a + ab * t, t


def push_points_off_segment(
    points: np.ndarray,
    prev_points: np.ndarray,
    point_indices: list[int],
    seg_a_idx: int,
    seg_b_idx: int,
    min_dist: float,
) -> int:
    """`point_indices` listesindeki her noktayı, `seg_a_idx`/`seg_b_idx`
    noktalarının tanımladığı doğru parçasından en az `min_dist` uzakta
    kalacak şekilde iter (segmentin İKİ UCU pinned/sabit kabul edilir --
    bu fonksiyon onları HİÇ değiştirmez, sadece `point_indices`'i iter).
    Kaç noktanın itildiğini (çarpıştığını) döndürür -- tanılama/doğrulama
    için kullanışlı.

    İtilen her noktanın `prev_points`'i de yeni konuma eşitlenir (bkz.
    `clamp_direction`) ki bu düzeltme sahte bir hız sıçraması yaratmasın
    -- kendi-kendine çarpışma zaten "itilme" hissini görsel olarak
    veriyor, ek bir sekme/zıplama hızı gerekmiyor."""
    a = points[seg_a_idx]
    b = points[seg_b_idx]
    n_pushed = 0
    for i in point_indices:
        p = points[i]
        closest, _ = _closest_point_on_segment(p, a, b)
        delta = p - closest
        dist = float(np.linalg.norm(delta))
        if dist < min_dist:
            n_pushed += 1
            if dist < 1e-6:
                # Nokta tam segment üzerindeyse yön belirsiz -- rastgele
                # olmayan sabit bir yönde (yukarı) it.
                direction = np.array([0.0, -1.0])
            else:
                direction = delta / dist
            points[i] = closest + direction * min_dist
            prev_points[i] = points[i]
    return n_pushed


def _closest_points_segment_segment(
    p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """İki 2D doğru parçası (`p1->q1` ve `p2->q2`) arasındaki EN YAKIN
    nokta çiftini ve bu noktaların kendi segmentleri üzerindeki
    enterpolasyon oranlarını (`s`, `t`, ikisi de [0,1] aralığında; en
    yakın nokta `p1 + s*(q1-p1)` / `p2 + t*(q2-p2)`) döndürür.

    Kaynak: Ericson, "Real-Time Collision Detection" (2004), Bölüm 5.1.9
    "Closest Point of Two Line Segments" -- standart, sayısal olarak
    kararlı (paralel/dejenere segmentleri de ele alan) algoritma, burada
    2D'ye uyarlandı (3D versiyonuyla matematiksel olarak birebir aynı,
    sadece nokta boyutu 2)."""
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))
    EPS = 1e-9

    if a <= EPS and e <= EPS:
        return p1.copy(), p2.copy(), 0.0, 0.0
    if a <= EPS:
        s = 0.0
        t = float(np.clip(f / e, 0.0, 1.0)) if e > EPS else 0.0
    else:
        c = float(np.dot(d1, r))
        if e <= EPS:
            t = 0.0
            s = float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b
            s = float(np.clip((b * f - c * e) / denom, 0.0, 1.0)) if denom > EPS else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = float(np.clip(-c / a, 0.0, 1.0))
            elif t > 1.0:
                t = 1.0
                s = float(np.clip((b - c) / a, 0.0, 1.0))

    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    return c1, c2, s, t


def push_segment_off_segment(
    points: np.ndarray,
    prev_points: np.ndarray,
    seg1: tuple[int, int],
    seg2: tuple[int, int],
    min_dist: float,
    pinned: "set[int] | None" = None,
    masses: "np.ndarray | None" = None,
) -> bool:
    """İki SEGMENTİN (`seg1=(iA,iB)`, `seg2=(iC,iD)`) aralarındaki GERÇEK
    en yakın nokta çiftini (`_closest_points_segment_segment()` ile --
    sadece uçlar değil, aradaki HER nokta dahil) hesaplayıp, mesafe
    `min_dist`'in altındaysa iki segmenti birbirinden iter.

    NEDEN GEREKLİ (5. tur sonrası kullanıcı isteği -- "Kapsül Tabanlı
    Öz-Çarpışma"): `push_points_off_segment()` (yukarıda) sadece
    `point_indices`'teki UÇ noktaları segmentin uçlarına göre test eder;
    ör. bir ön-kolun (dirsek->el) TAM ORTASI gövde segmentini kesse bile,
    dirsek ve el kendileri segmentten yeterince uzaktaysa hiçbir çarpışma
    algılanmazdı. Bu fonksiyon iki segmentin ARALARINDAKİ gerçek geometrik
    mesafeyi (nokta-bazlı değil, doğru-parçası-vs-doğru-parçası) kullanır
    -- kullanıcının "iki çizginin birbirine en yakın noktasını hesaplayıp"
    dediği tam olarak bu.

    Düzeltme, en yakın nokta çiftinin kendi segmentleri üzerindeki `s`/`t`
    enterpolasyon oranlarıyla uç noktalara (`A`/`B` ve `C`/`D`) DAĞITILIYOR
    (`1-s,s` ve `1-t,t` ağırlıklarıyla -- bir ucu hareket ettirmenin en
    yakın noktayı ne kadar kaydırdığının doğrusal yaklaşıklığı), ve iki
    segment arasındaki toplam düzeltme, `_satisfy_sticks()`'teki AYNI
    ters-kütle formülüyle (`w=1/kütle`) paylaşılıyor -- `masses` verilirse
    her segmentin "kütlesi" kendi iki ucunun ORTALAMASI olarak alınıyor
    (tam nokta-başına ters-kütle DEĞİL, segment başına TEK bir birleşik
    kütle -- aşağıdaki dürüst sınıra bakın). Pinned uçlar (`pinned`
    verilirse) hiç hareket ettirilmez; bir segmentin İKİ ucu da pinned'se
    o segment tamamen sabit kabul edilir (`_satisfy_sticks()`'teki
    `i_pinned`/`j_pinned` mantığıyla AYNI ruhta).

    **Momentum-koruyan düzeltme** (5. tur'daki ΔY-toplayıcı tanısından
    çıkan derse göre BİLİNÇLİ bir tasarım kararı -- bkz. `physics/
    verlet.py`'nin `clamp_direction(..., preserve_momentum=)` notu):
    uygulanan ofset hem `points`'e hem `prev_points`'e AYNI MİKTARDA
    eklenir, yani düzeltme noktanın `points-prev_points` hızını SİLMEZ --
    `push_points_off_segment()`'in (yukarıda, ışınlama+hız-sıfırlama)
    davranışından KASITLI OLARAK farklı; bu tur bu tür bir "hız sıfırlayan"
    yeni bir kısıtlama daha eklememek için baştan momentum-koruyan
    tasarlandı.

    DÜRÜST SINIRLAR: (1) tek bir çarpışma yinelemesi (relaksasyon değil)
    -- `_satisfy_sticks()` gibi birden fazla yumuşak iterasyon YOK, bu
    yüzden aynı karede birden fazla çakışan segment çifti varsa tam
    yakınsamayabilir (pratikte demo sahnelerinde yeterli görüldü, bkz.
    doğrulama). (2) segment başına TEK birleşik kütle (iki ucun
    ortalaması) kullanılıyor, `_satisfy_sticks()`'teki gibi nokta başına
    tam ters-kütle DEĞİL -- ör. çok ağır bir uç ile çok hafif bir uç aynı
    segmentte olduğunda, düzeltme o segmentin İKİ UCUNA da (kütle farkı
    gözetmeksizin) `s`/`(1-s)` oranıyla dağılıyor. (3) segmentler
    "sonsuz ince çizgi" olarak modelleniyor -- gerçek bir kapsül (silindir/
    stadium) çarpışması değil, iki en-yakın-nokta arasındaki mesafeye göre
    tek yönlü bir itme; ama `min_dist` parametresi pratikte bir "kapsül
    yarıçapı toplamı" gibi davranıyor (bkz. çağıran kodun `ARM_SELF_
    COLLISION_DIST` gibi sabitleri)."""
    iA, iB = seg1
    iC, iD = seg2
    A, B, C, D = points[iA], points[iB], points[iC], points[iD]
    c1, c2, s, t = _closest_points_segment_segment(A, B, C, D)
    delta = c1 - c2
    dist = float(np.linalg.norm(delta))
    if dist >= min_dist:
        return False
    direction = delta / dist if dist > 1e-6 else np.array([0.0, -1.0])
    overlap = min_dist - dist

    pinned = pinned if pinned is not None else set()

    def _mass(i: int) -> float:
        if masses is not None and i < len(masses):
            return max(float(masses[i]), 1e-9)
        return 1.0

    seg1_pinned = iA in pinned and iB in pinned
    seg2_pinned = iC in pinned and iD in pinned
    if seg1_pinned and seg2_pinned:
        return True  # ikisi de sabit -- geometrik olarak "çarpıştı" ama itilecek hiçbir şey yok

    mass1 = 0.5 * (_mass(iA) + _mass(iB))
    mass2 = 0.5 * (_mass(iC) + _mass(iD))
    w1 = 0.0 if seg1_pinned else 1.0 / mass1
    w2 = 0.0 if seg2_pinned else 1.0 / mass2
    total_w = w1 + w2
    frac1 = w1 / total_w
    frac2 = w2 / total_w

    if not seg1_pinned:
        push1 = direction * (overlap * frac1)
        if iA not in pinned:
            offset = push1 * (1.0 - s)
            points[iA] = points[iA] + offset
            prev_points[iA] = prev_points[iA] + offset
        if iB not in pinned:
            offset = push1 * s
            points[iB] = points[iB] + offset
            prev_points[iB] = prev_points[iB] + offset
    if not seg2_pinned:
        push2 = direction * (-(overlap * frac2))
        if iC not in pinned:
            offset = push2 * (1.0 - t)
            points[iC] = points[iC] + offset
            prev_points[iC] = prev_points[iC] + offset
        if iD not in pinned:
            offset = push2 * t
            points[iD] = points[iD] + offset
            prev_points[iD] = prev_points[iD] + offset
    return True


def apply_drag(points: np.ndarray, prev_points: np.ndarray, point_indices: list[int], drag_coeff: float) -> None:
    """Verilen noktaların hızını (`points - prev_points`) `1 - drag_coeff`
    ile ölçekler -- `VerletSystem.friction`'dan BAĞIMSIZ, ekstra bir hava
    sürtünmesi/direnci terimi (kullanıcının "sürtünmeyi artır" isteği).
    `VerletSystem.friction` zaten HER karede TÜM sisteme uygulanıyor
    (bkz. `_integrate()`); bu fonksiyon sadece BELİRLİ noktalara (ör.
    pelerin) EK bir sönümleme uygulamak için var, gövdeyi etkilemeden."""
    for i in point_indices:
        velocity = points[i] - prev_points[i]
        prev_points[i] = points[i] - velocity * max(1.0 - drag_coeff, 0.0)
