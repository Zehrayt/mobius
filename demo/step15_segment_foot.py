"""
Adim 15 (kullanicinin 13. tur eki 2'ye getirdigi 3. elestiri -- "nokta
temasi limitine carptin"): ayagin TEK bir Verlet noktasi (`anchor`,
step8-14'te kullanilan mimari) olmasi yerine, topuk (heel) ve parmak
ucu (toe) olmak uzere IKI ayri Verlet noktasindan olusan RIJIT bir
"kapsul ayak"a donusumun ilk izole doğrulamasi.

KULLANICI ALINTISI (13. tur eki 2 sonrasi): "Ayagin tek bir nokta
olmasi, ayak bileği torku uretememesi ve kayarken topugun yeri
kazıyarak yavaslayamamasi (friction spike), su anki tum sahte
yamalarin asil sebebidir. step15_segment_foot.py adinda yeni bir
izole laboratuvar acip, tek noktali (point) ayagi, topuk ve parmak
ucu (heel-to-toe) olmak uzere iki Verlet dugumunden olusan gercek bir
kapsul-ayaga (Segment Foot) donusturmenin vakti geldi."

BU DOSYANIN KAPSAMI -- "CHECKPOINT 1" (durust sinir en basta):
bu, TAM bir yuruyus entegrasyonu DEGIL. `step14_active_biped.py`'nin
tum stance/swing state machine'i, capture-point/IK hedefleme, kayma
(Stribeck) modeli vs. burada YOK -- kalca KASITLI OLARAK kinematik/
pinned ve kontrollu, yavas (quasi-statik) bir supurme ile hareket
ettiriliyor (gercek yuruyusteki gibi dinamik/momentum-tabanli DEGIL).
AMAC: `physics/verlet.py` VEYA `physics/collision.py`'ye TEK BIR
SATIR bile dokunmadan -- rijit bir ankle-topuk-parmak-ucu ucgeni +
degismemis `collide_ground()` + yercekimi ile, GERCEK bir topuk-parmak
ucu yuvarlanmasinin (heel-to-toe roll) hicbir "kaldirma" kurali
YAZMADAN kendiliginden (emergent) ortaya cikip cikmadigini olcmek.

MIMARI: dort nokta -- `hip` (pinned, script-guduml), `ankle` (SERBEST,
`hip`'e `LEG_LEN` uzunlugunda rijit bir cubukla bagli), `heel` ve `toe`
(SERBEST, `ankle`'a rijit cubuklarla + BIRBIRLERINE de rijit bir
cubukla (`FOOT_LEN`) bagli -- bu uc nokta sabit bir ucgen/ayak
geometrisi olusturuyor). `heel`/`toe` zeminle `physics/collision.py`'nin
DEGISTIRILMEMIS `collide_ground()` fonksiyonuyla etkilesiyor -- yani
"ayagin hangi ucu yerde" sorusunun cevabi HICBIR YERDE script'lenmiyor,
sadece fizikten (kisit + carpisma) cikiyor.

BULGU (bkz. README "Adim 15 -- checkpoint 1" bolumu icin tam sayilar):
kalca ayagin tam ustunden (duz taban, heel_y~toe_y~GROUND_Y, aci~0°)
parmak ucunun ilerisine dogru yavasca supurulduğunde, TOE zeminde
KİLİTLİ kalirken (`toe_y` tam `GROUND_Y`'de doyuyor) HEEL_Y DUZENLI/
MONOTONIK olarak yukseliyor (zeminden kalkiyor) ve ayak acisi 0°'den
~44°'ye kadar SUREKLI artiyor -- yani gercek bir topuk-kalkisi (heel-
off) / parmak-ucu-itisi (toe-off) yuvarlanmasi, EMERGENT olarak, hicbir
ek kural olmadan ortaya cikiyor. Bu, kullanicinin "ayak bileği torku ve
yuzey alani olmadan sürtünme fizigini daha fazla ileri goturemezsin"
tespitinin dogru cozum yonunu (nokta yerine rijit iki-nokta kapsul)
sayisal olarak dogruluyor.

SIRADAKI ADIMLAR (bu dosyada YAPILMADI, gelecekteki turlar):
  - Bu iki-nokta ayagin `step14_active_biped.py`'nin stance/swing state
    machine'ine (FootPlantingLeg/ActiveFootPlantingLeg) entegrasyonu --
    "planted" tek bir x-koordinati degil, heel_x/toe_x cifti olmali.
  - Kayma (Stribeck) modelinin heel/toe'ya AYRI AYRI uygulanmasi -- bir
    "topuk kazima" (heel-scrape) friction-spike modeli icin heel'in
    kendi surtunme/kayma durumu toe'dan bagimsiz olmali.
  - IK/FABRIK zincirinin ayak-ucu hedefi olarak tek nokta yerine bu
    ayak segmentini kullanacak sekilde guncellenmesi.
  - Ayak bilegi ACISININ kendisinin bir yay-sonumleme (Hooke) kisitiyla
    sinirlanip sinirlanmayacagi (gercek bir insan ayak bilegi sonsuz
    donmez) -- bu checkpoint'te BILEREK sinirsiz birakildi (bkz. yukarida
    hip_x cok ileri gittiğinde ~44°'de tepe yapip sonra tuhaf bir sekilde
    kismen geri sarmasi -- bu, checkpoint'in ANATOMIK OLARAK ANLAMLI
    aralik disina (hip_x, foot uzunlugunun 4+ kati kadar ileri) tasindiginda
    ortaya cikan bir sinir-disi artefakt, gercek yuruyuste HIC bu kadar
    ileri gidilmeyecek cunku foot cok daha once swing'e gecerdi).

Cikti: outputs/step15_segment_foot_checkpoint1.mp4
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from physics.verlet import VerletSystem
from physics.collision import collide_ground
from physics.environment import Terrain

W, H = 640, 300
FPS = 30
GROUND_Y = 220.0
ANKLE_HEIGHT = 10.0       # ankle'in zeminden yuksekligi (ayak "duz" iken)
FOOT_LEN = 26.0           # topuk-parmak ucu arasi mesafe -- FOOT_HALF_LEN*2 (physics.balance) ile ayni mertebede
LEG_LEN = 140.0           # kalca-ankle mesafesi (rijit) -- bu checkpoint'te step14'un ARM_LENGTH'inden bagimsiz, kucuk/hizli test icin secildi
SWEEP_PX_PER_FRAME = 0.6  # kalcanin quasi-statik supurme hizi (dinamik/momentum artefaktlarini minimize etmek icin YAVAS)
N_FRAMES = 260

TUNED_GRAVITY = np.array([0.0, 0.12])
GROUND_FRICTION = 0.35


def build_body() -> tuple[VerletSystem, dict]:
    sys_ = VerletSystem.empty()
    sys_.gravity = TUNED_GRAVITY.copy()
    sys_.friction = 0.02
    idx: dict = {}
    idx["hip"] = sys_.add_point([0.0, GROUND_Y - ANKLE_HEIGHT - LEG_LEN], pinned=True)
    idx["ankle"] = sys_.add_point([0.0, GROUND_Y - ANKLE_HEIGHT], mass=1.0)
    idx["heel"] = sys_.add_point([-FOOT_LEN / 2.0, GROUND_Y], mass=0.4)
    idx["toe"] = sys_.add_point([FOOT_LEN / 2.0, GROUND_Y], mass=0.4)
    sys_.add_stick(idx["hip"], idx["ankle"], length=LEG_LEN, compliance=0.0)
    heel_dist = float(np.hypot(FOOT_LEN / 2.0, ANKLE_HEIGHT))
    sys_.add_stick(idx["ankle"], idx["heel"], length=heel_dist, compliance=0.0)
    sys_.add_stick(idx["ankle"], idx["toe"], length=heel_dist, compliance=0.0)
    sys_.add_stick(idx["heel"], idx["toe"], length=FOOT_LEN, compliance=0.0)
    return sys_, idx


def to_screen(p, camera_offset):
    return (int(p[0] + camera_offset), int(p[1]))


def main() -> None:
    body, idx = build_body()
    hip, ankle, heel, toe = idx["hip"], idx["ankle"], idx["heel"], idx["toe"]
    terrain = Terrain(ground_y=GROUND_Y, default_friction=GROUND_FRICTION, zones=[])

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step15_segment_foot_checkpoint1.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    hip_x0 = body.points[hip][0]
    hip_x_log, heel_y_log, toe_y_log, angle_log = [], [], [], []

    for f in range(N_FRAMES):
        hip_x = hip_x0 - FOOT_LEN * 1.2 + f * SWEEP_PX_PER_FRAME
        body.set_pinned_position(hip, [hip_x, body.points[hip][1]])

        body.step(dt=1.0)
        collide_ground(body, terrain.floor_fn, terrain.friction_fn)

        heel_toe_vec = body.points[toe] - body.points[heel]
        foot_angle_deg = float(np.degrees(np.arctan2(heel_toe_vec[1], heel_toe_vec[0])))

        hip_x_log.append(float(hip_x))
        heel_y_log.append(float(body.points[heel][1]))
        toe_y_log.append(float(body.points[toe][1]))
        angle_log.append(foot_angle_deg)

        frame = np.full((H, W, 3), 22, dtype=np.uint8)
        camera_offset = W / 2 - hip_x
        cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (85, 85, 85), 2)
        cv2.line(frame, to_screen(body.points[hip], camera_offset), to_screen(body.points[ankle], camera_offset), (200, 200, 200), 3)
        cv2.line(frame, to_screen(body.points[heel], camera_offset), to_screen(body.points[toe], camera_offset), (60, 180, 255), 4)
        cv2.line(frame, to_screen(body.points[ankle], camera_offset), to_screen(body.points[heel], camera_offset), (140, 140, 255), 2)
        cv2.line(frame, to_screen(body.points[ankle], camera_offset), to_screen(body.points[toe], camera_offset), (140, 140, 255), 2)
        for pname, color in [("hip", (255, 255, 255)), ("ankle", (0, 255, 255)), ("heel", (60, 180, 255)), ("toe", (60, 180, 255))]:
            cv2.circle(frame, to_screen(body.points[idx[pname]], camera_offset), 5, color, -1)
        writer.write(frame)

        if not np.all(np.isfinite(body.points)):
            print(f"UYARI: NaN/inf @ frame {f}")
            break

    writer.release()
    print(f"wrote {out_path}")

    hip_x_a = np.array(hip_x_log)
    heel_y_a = np.array(heel_y_log)
    toe_y_a = np.array(toe_y_log)
    angle_a = np.array(angle_log)

    print()
    print(f"=== Adim 15 -- Segment Foot checkpoint 1 raporu ({N_FRAMES} kare) ===")
    print(f"hip_x araligi: {hip_x_a.min():.1f} -> {hip_x_a.max():.1f}")
    print(f"heel_y araligi: min={heel_y_a.min():.2f} max={heel_y_a.max():.2f}  (GROUND_Y={GROUND_Y})")
    print(f"toe_y araligi:  min={toe_y_a.min():.2f} max={toe_y_a.max():.2f}")
    print(f"ayak acisi araligi: min={angle_a.min():.2f} max={angle_a.max():.2f} derece (0=duz taban)")

    heel_center_x, toe_center_x = -FOOT_LEN / 2.0, FOOT_LEN / 2.0
    i_heel = int(np.argmin(np.abs(hip_x_a - heel_center_x)))
    i_mid = int(np.argmin(np.abs(hip_x_a - 0.0)))
    i_toe = int(np.argmin(np.abs(hip_x_a - toe_center_x)))
    print(f"\nkalca TOPUK ustunde (hip_x={hip_x_a[i_heel]:.1f}): heel_y={heel_y_a[i_heel]:.2f} toe_y={toe_y_a[i_heel]:.2f} aci={angle_a[i_heel]:.2f}")
    print(f"kalca ORTADA       (hip_x={hip_x_a[i_mid]:.1f}): heel_y={heel_y_a[i_mid]:.2f} toe_y={toe_y_a[i_mid]:.2f} aci={angle_a[i_mid]:.2f}")
    print(f"kalca PARMAK UCUNDA (hip_x={hip_x_a[i_toe]:.1f}): heel_y={heel_y_a[i_toe]:.2f} toe_y={toe_y_a[i_toe]:.2f} aci={angle_a[i_toe]:.2f}")


if __name__ == "__main__":
    main()
