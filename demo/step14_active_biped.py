"""
Adim 14 (8. tur -- kullanicinin "Kinematik-Pin Kalca" elestirisi ve
"Active Ragdoll" talebi): kalcanin (hip) ARTIK bir `driver`/pin noktasina
degil, SADECE zeminde duran bacagin kendi rijit-kol geometrisine (ters
sarkac) + bir "kalca-uzatici kas" itkisine (push-off) bagli, TAMAMEN
SERBEST bir Verlet noktasi oldugu ilk calisir iki-bacakli dinamik yuruyus
prototipi.

MIMARI (bkz. `physics/active_gait.py`'nin tam dokstring'i icin gerekce/
capture-point matematigi): `gait.py`/`FootPlantingLeg` DEGISTIRILMEDI --
step1-13 kanaryalari korunuyor. Bu demo, ondan kalitim alan yeni
`ActiveFootPlantingLeg` sinifini kullaniyor. Kalca-destek fizigi: `hip`
serbest bir Verlet noktasi, `anchor` (o an zeminde duran bacagin
`planted` konumuna HER karede tasinan, pinned) bir nokta, ikisi arasinda
`chain.arm_length` uzunlugunda rijit bir cubuk -- bu, kalcayi "o an
zeminde duran ayagin ustunde donen bir ters sarkac" gibi davranmaya
zorluyor (robotikte "compass gait" olarak bilinen standart basitlestirme).

ITKI (push-off): gercek insan yuruyusundeki kalca-uzatici/baldir kasi
itkisinin kaba bir modeli -- HER karede (sadece bir bacak zemindeyken)
kalcaya, HEDEF hizla (`TARGET_VX`) mevcut hiz arasindaki farka ORANTILI
(P-kontrolcu) bir ileri/geri ivme uygulaniyor. DURUST NOT: ilk denemede
bu SABIT/kosulsuz bir itki olarak denendi -- sonuc SINIRSIZ hizlanan bir
karakterdi (itkinin geri besleme olmadan surekli enerji eklemesi, hicbir
sey onu frenlemiyordu). Hedef-hiz farkina orantili hale getirilince
(asagidaki THRUST_GAIN/TARGET_VX) sistem kendiliginden KARARLI bir
sabit-hiz yuruyuse yakinsiyor -- bkz. commit mesaji/README'deki tanilama.

Cikti: outputs/step14_active_biped.mp4
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from physics.verlet import VerletSystem, clamp_direction
from physics.active_gait import ActiveFootPlantingLeg

W, H = 640, 400
FPS = 30
DURATION_S = 12
N_FRAMES = FPS * DURATION_S

HIP_Y = 170.0
GROUND_Y = 330.0
TORSO_LEN = 55.0
HEAD_STICK_LEN = 30.0
HEAD_RADIUS = 15

LEG_SEGMENT_LEN = 92.0
ARM_LENGTH = LEG_SEGMENT_LEN * 2.0  # chain.arm_length ile ayni (iki segment)
SWING_DURATION_FRAMES = 10
LIFT_HEIGHT = 22.0
KNEE_LIMITS = (8.0, 150.0)
KNEE_BEND_SIGN = -1.0

UP = np.array([0.0, -1.0])
TORSO_MAX_LEAN_DEG = 12.0
NECK_MAX_TILT_DEG = 18.0

TUNED_GRAVITY = np.array([0.0, 0.065])
TUNED_FRICTION = 0.045

# -- Dinamik kalca-itkisi (bkz. modul dokstring'i -- P-kontrolcu) -------
TARGET_VX = 2.0     # hedef ileri hiz (px/kare) -- eski WALK_SPEED=60px/s'in
                     # 30 FPS'teki karsiligi ~2.0px/kare ile KARSILASTIRILABILIR
                     # kilmak icin bilincli olarak secildi (birebir esitlemek
                     # ZORUNLU degil -- bu artik EMERGENT bir hiz, DIKTE
                     # EDILEN degil).
THRUST_GAIN = 1.0
THRUST_CAP = 1.5     # tek karede uygulanabilecek max itki/fren (kararlilik siniri)

# -- Capture-point (destek/adim) parametreleri (bkz. active_gait.py) ----
# DURUST BULGU (izole tanilama sirasinda kesfedildi): LIP formulunun
# TEORIK degeri omega0=sqrt(g/L) = sqrt(0.065/184) = 0.0188 buraya
# DOGRUDAN konulunca sistem KARARSIZ cikti (ort. hip_vx hedefin 1.8 kati,
# ara sira 25px/kare'ye varan sicramalar) -- cunku bu motor HER ZAMAN
# dt=1.0 ile calisiyor (bkz. "Mimari refactor" bolumundeki onceden
# belgelenmis "kare hizindan bagimsiz degil" siniri); LIP formulunun
# surekli-zaman (saniye bazli) turetimi bu ayrik/kare-bazli sisteme
# DOGRUDAN aktarilmiyor. Ampirik bir tarama (0.03/0.045/0.08) 0.045'in
# kararli oldugunu gosterdi -- bu deger o yuzden TEORIK degil, OLCULEREK
# secildi (bkz. commit mesaji/README).
OMEGA0 = 0.045
SUPPORT_MARGIN = 6.0
SWING_LEAD_MARGIN = 12.0

# -- Sahne olaylari: bir kucuk (ABSORBE EDILEN) ve bir buyuk (GERCEK
#    DUSMEYE yol acan) darbe.
#    DURUST BULGU (izole harness_preserve.py taramasi ile bulundu):
#    preserve_momentum=True duzeltmesinden SONRA (gercek govde
#    eylemsizligi ile), eski STUMBLE_KICK_PX=30 degeri artik guvenli
#    degil -- gecikmeli bir dusmeye yol aciyordu (frame 141, tokezleme
#    frame 90'dan ~50 kare sonra). Kucuk itkiler taramasi:
#      5px, 10px, 15px  -> hayatta kaldi
#      20px, 25px       -> dustu (frame 163, 171)
#    Gercek absorbe/dusme siniri 15px-20px arasinda; STUMBLE_KICK_PX
#    bu nedenle 15px'e cekildi (dogrulanmis en buyuk 'hayatta kalan'
#    deger). Eski 30px degeri artik 'buyuk itki' kategorisine daha
#    yakindi ve bu adimin gostermek istedigi seyi (kucuk bir
#    tokezlemenin gercekten absorbe edilmesini) dogru temsil etmiyordu.
STUMBLE_T = 3.0
STUMBLE_KICK_PX = 15.0
BIG_PUSH_T = 7.0
BIG_PUSH_KICK_PX = 150.0

FALL_HIP_Y_THRESHOLD = GROUND_Y - 10.0  # bkz. main() -- sadece raporlama icin


def build_body() -> tuple[VerletSystem, dict]:
    sys_ = VerletSystem.empty()
    sys_.gravity = TUNED_GRAVITY.copy()
    sys_.friction = TUNED_FRICTION
    idx: dict = {}

    # ESKI mimarideki `driver` (kinematik/pin, scriptlenmis hizla suruklenen)
    # noktasi YOK -- `hip` artik TAMAMEN serbest. `anchor`, HER karede o an
    # zeminde duran bacagin `planted` konumuna tasinan (pinned) bir nokta;
    # `hip`'e ARM_LENGTH uzunlugunda RIJIT bir cubukla bagli -- bu, kalcayi
    # "o an zeminde duran ayagin ustunde donen bir ters sarkac" gibi
    # davranmaya zorluyor (bkz. modul dokstring'i).
    idx["hip"] = sys_.add_point([0.0, HIP_Y], mass=1.0)
    idx["anchor"] = sys_.add_point([0.0, GROUND_Y], pinned=True)
    sys_.add_stick(idx["anchor"], idx["hip"], length=ARM_LENGTH, compliance=0.0)

    idx["shoulder"] = sys_.add_point([0.0, HIP_Y - TORSO_LEN])
    sys_.add_stick(idx["hip"], idx["shoulder"], length=TORSO_LEN)

    idx["head"] = sys_.add_point([0.0, HIP_Y - TORSO_LEN - HEAD_STICK_LEN])
    sys_.add_stick(idx["shoulder"], idx["head"], length=HEAD_STICK_LEN)

    return sys_, idx


def make_leg(hip_pos: np.ndarray, initial_planted_offset: float) -> ActiveFootPlantingLeg:
    return ActiveFootPlantingLeg(
        hip_pos,
        segment_lengths=[LEG_SEGMENT_LEN, LEG_SEGMENT_LEN],
        ground_y=GROUND_Y,
        swing_duration_frames=SWING_DURATION_FRAMES,
        lift_height=LIFT_HEIGHT,
        initial_planted_offset=initial_planted_offset,
        knee_limits=KNEE_LIMITS,
        knee_bend_sign=KNEE_BEND_SIGN,
        capture_gain=1.0,
        support_margin=SUPPORT_MARGIN,
        swing_lead_margin=SWING_LEAD_MARGIN,
        omega0=OMEGA0,
    )


def draw_frame(body: VerletSystem, idx: dict, legs: list[ActiveFootPlantingLeg],
               camera_offset: float, hip_vx: float, fell: bool) -> np.ndarray:
    frame = np.full((H, W, 3), 22, dtype=np.uint8)
    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (85, 85, 85), 2)

    def to_screen(p):
        return (int(p[0] + camera_offset), int(p[1]))

    body_color = (90, 220, 90) if not fell else (90, 90, 220)
    leg_colors = [(90, 220, 90), (230, 160, 60)]

    cv2.line(frame, to_screen(body.points[idx["hip"]]), to_screen(body.points[idx["shoulder"]]), body_color, 4, cv2.LINE_AA)
    cv2.line(frame, to_screen(body.points[idx["shoulder"]]), to_screen(body.points[idx["head"]]), body_color, 3, cv2.LINE_AA)
    cv2.circle(frame, to_screen(body.points[idx["head"]]), HEAD_RADIUS, body_color, 2, cv2.LINE_AA)

    for leg, color in zip(legs, leg_colors):
        pts = leg.chain.points
        for i in range(len(pts) - 1):
            cv2.line(frame, to_screen(pts[i]), to_screen(pts[i + 1]), color, 4, cv2.LINE_AA)
        for p in pts:
            cv2.circle(frame, to_screen(p), 5, (255, 255, 255), -1, cv2.LINE_AA)

    status = "DUSTU" if fell else "yuruyor"
    color_txt = (110, 110, 255) if fell else (210, 210, 210)
    cv2.putText(frame, f"hip_vx: {hip_vx:+.2f}px/kare (hedef {TARGET_VX:.1f})   durum: {status}",
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color_txt, 1, cv2.LINE_AA)
    cv2.putText(frame, "Adim 14 (8. tur): kalca ARTIK kinematik-pin DEGIL -- capture-point tabanli dinamik biped",
                (12, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    body, idx = build_body()
    hip = idx["hip"]
    anchor = idx["anchor"]

    half = ARM_LENGTH * 0.15
    left_leg = make_leg(body.points[hip].copy(), -half)
    right_leg = make_leg(body.points[hip].copy(), +half)
    # sag bacak baslangicta swing'de -- alternatif adimla baslamasi icin.
    right_leg.state = "swing"
    right_leg.swing_start = right_leg.planted.copy()
    right_leg.swing_target = np.array([half + 20.0, GROUND_Y])
    right_leg.swing_t = 0.0

    dt = 1.0 / FPS
    stumbled = False
    big_pushed = False
    fell = False
    fall_frame = None
    step_events = []

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step14_active_biped.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    hip_x_log = []
    hip_y_log = []
    hip_vx_log = []

    for f in range(N_FRAMES):
        t = f * dt
        hip_pos_before = body.points[hip].copy()
        hip_prev = body.prev_points[hip].copy()
        hip_vx = hip_pos_before[0] - hip_prev[0]

        if not stumbled and t >= STUMBLE_T:
            body.prev_points[hip][0] -= STUMBLE_KICK_PX
            stumbled = True
        if not big_pushed and t >= BIG_PUSH_T:
            body.prev_points[hip][0] -= BIG_PUSH_KICK_PX
            big_pushed = True

        stance_leg = None
        for leg in (left_leg, right_leg):
            if leg.state == "stance":
                stance_leg = leg
        if stance_leg is not None and not fell:
            body.set_pinned_position(anchor, [stance_leg.planted[0], GROUND_Y])
            thrust = THRUST_GAIN * (TARGET_VX - hip_vx)
            thrust = max(-THRUST_CAP, min(THRUST_CAP, thrust))
            body.prev_points[hip][0] -= thrust

        body.step(dt=1.0)
        # DURUST BULGU (izole tanilama ile kesfedildi -- bkz. commit
        # mesaji/README): clamp_direction()'i VARSAYILAN (preserve_
        # momentum=False) cagirmak, govde/boyun uzerinden kalcanin
        # KENDI hizina da dolayli bir "sifirlama" sizdiriyordu -- 150px'lik
        # BIG_PUSH_KICK_PX bile ARTIK hicbir zaman dusmeye yol acmiyordu
        # (400px'e kadar test edildi, hep hayatta kaldi) -- yani govde/bas
        # eklenmeden ONCE gercek olan "60px+ dusme" bulgusu, bu eksik
        # parametreyle SESSIZCE maskeleniyordu. `preserve_momentum=True`
        # (5. turda ayni sinif sorun icin zaten kurulan duzeltme deseni)
        # ile dusme esigi govdenin GERCEK ek eylemsizligini yansitan,
        # daha fizik-tutarli bir araliga (~100px) geri donuyor.
        clamp_direction(body.points, body.prev_points, hip, idx["shoulder"], UP, TORSO_MAX_LEAN_DEG, preserve_momentum=True)
        torso_dir = body.points[idx["shoulder"]] - body.points[hip]
        clamp_direction(body.points, body.prev_points, idx["shoulder"], idx["head"], torso_dir, NECK_MAX_TILT_DEG, preserve_momentum=True)

        hip_pos = body.points[hip]
        # NOT: bu dongu bilerek SIRALI (once sol, sonra sag) calisir --
        # `other_leg_swinging` bayragi sag bacak icin SOL BACAGIN BU KAREDE
        # ZATEN GUNCELLENMIS durumunu kullanir. Bunu "simetrik" hale getirip
        # her iki bacaga da guncelleme-oncesi ayni anlik goruntuyu vermeyi
        # denedim (bkz. git gecmisi) -- bu, iki bacagin da ayni karede stance
        # oldugunu gorup AYNI ANDA swing'e gecmesine izin vererek çift-havada
        # (double-swing) durumunu geri getirdi ve anlik dusmeye yol acti
        # (dogrulama: frame 72'de ani dusme, buyuk itkiden -- t=7.0s/frame210
        # -- COK once). Sirali degerlendirme, sag bacagin swing kararini HER
        # ZAMAN sol bacagin bu karedeki nihai durumuna gore vermesini
        # saglayarak tek-destek (single-support) kuralini garanti eder; bunun
        # bedeli, bir tokezlemeden hemen sonra bir bacagin (planted noktasi
        # capture-point tarafindan asiri ileri itilirse) birkac kare boyunca
        # ust uste hizlica adim atmasi olabilir (asagidaki tokezleme testinde
        # gozlemlendi) -- bu, cift-destek kaybından cok daha az tehlikeli bir
        # gecici salinimdir, o yuzden duzeltilmedi.
        for leg in (left_leg, right_leg):
            other = right_leg if leg is left_leg else left_leg
            was_stance = leg.state == "stance"
            leg.update(hip_pos, hip_vx=hip_vx, other_leg_swinging=(other.state == "swing"))
            if was_stance and leg.state == "swing":
                step_events.append((f, "l" if leg is left_leg else "r"))

        if not fell and hip_pos[1] > FALL_HIP_Y_THRESHOLD:
            fell = True
            fall_frame = f

        hip_x_log.append(hip_pos[0])
        hip_y_log.append(hip_pos[1])
        hip_vx_log.append(hip_vx)

        camera_offset = W / 2 - hip_pos[0]
        frame = draw_frame(body, idx, [left_leg, right_leg], camera_offset, hip_vx, fell)
        writer.write(frame)

        if not np.all(np.isfinite(body.points)):
            print(f"UYARI: NaN/inf @ frame {f}")
            break

    writer.release()
    print(f"wrote {out_path}")

    hip_x_log = np.array(hip_x_log)
    hip_y_log = np.array(hip_y_log)
    hip_vx_log = np.array(hip_vx_log)

    st = int(STUMBLE_T * FPS)
    bp = int(BIG_PUSH_T * FPS)
    print()
    print(f"=== Adim 14 -- dinamik biped raporu ({N_FRAMES} kare, {DURATION_S}s) ===")
    print(f"toplam adim sayisi: {len(step_events)}  -> {step_events}")
    # DURUST DUZELTME: onceki kontrol "dusme karesi tokezlemeden >20
    # kare sonra ise EVET (absorbe edildi)" diyordu -- ama bu, buyuk
    # itkiden (t=BIG_PUSH_T) ONCE gerceklesen bir dusmeyi de yanlislikla
    # "absorbe edildi" olarak raporluyordu (orn. frame 141'deki dusme,
    # aslinda STUMBLE_KICK_PX=30 tokezlemesinden kaynaklaniyordu, ama
    # st+20=110'dan buyuk oldugu icin yanlislikla EVET yazdiriyordu).
    # Dogru mantik: dusme, buyuk itki karesinden (bp) ONCE olduysa bu
    # tokezlemenin kendisinden kaynaklanmis demektir -- absorbe edilmemis.
    stumble_caused_fall = fell and fall_frame is not None and fall_frame < bp
    print(f"tokezleme (t={STUMBLE_T}s, {STUMBLE_KICK_PX:.0f}px): "
          f"frame {st}-{st+15} hip_vx araligi=[{hip_vx_log[st:st+15].min():.2f}, {hip_vx_log[st:st+15].max():.2f}]"
          f"  (absorbe edildi mi: {'HAYIR' if stumble_caused_fall else 'EVET'})")
    print(f"buyuk itki (t={BIG_PUSH_T}s, {BIG_PUSH_KICK_PX:.0f}px) sonrasi: dustu={fell}"
          f"  (dusme karesi: {fall_frame}, t={fall_frame/FPS if fall_frame else None})")
    print(f"son hip_y: {hip_y_log[-1]:.2f} (GROUND_Y={GROUND_Y}, dusme esigi={FALL_HIP_Y_THRESHOLD})")
    print(f"ortalama hip_vx (buyuk itkiden ONCE, kararli yuruyus): {hip_vx_log[:bp].mean():.3f}px/kare (hedef={TARGET_VX})")


if __name__ == "__main__":
    main()
