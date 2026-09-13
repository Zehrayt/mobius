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

DÜRÜST BİR SINIR: bu, pelerin/kuyruk gibi İNCE, tek zincirli ikincil
nesneler için yeterli (görsel olarak gövdeyi kesmemesini sağlar), ama
KALIN/hacimli bir çarpışma (ör. iki kalın uzvun birbirine göre hacim
kaplaması) modellemez -- segmentin İKİ UCU arasındaki en yakın noktaya
göre tek bir itme mesafesi (`min_dist`) uygulanır.
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
