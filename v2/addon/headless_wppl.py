# -*- coding: utf-8 -*-
"""헤드리스 주행 + KPI 수집. 렌더·스트리밍 없이 물리만 돌린다.

    python v2/addon/headless_wppl.py --n 12 \
        --stage v2/warehouse_v59_wppl12.usd \
        --traj  v2/traj_wppl/fleet_12/trajectories.json \
        --seconds 420 --out v2/kpi_wppl12.json

[왜 필요한가]
스트리밍 앱(live_wppl12.py)은 창고 전체를 매 프레임 RTX 로 그리고 NVENC 로 인코딩한다.
그 비용이 로봇 수와 거의 무관한 고정비라, 12대 실측 배속이 0.33배에 묶인다
(2대 0.44배 -> 12대 0.33배. 로봇이 6배인데 25%만 느려진 것이 그 증거다).
420초 시나리오 하나에 벽시계 21분이라 fleet 크기·시드를 바꿔가며 반복하기 어렵다.

여기서는 render=False 로 돌린다. **물리 결과는 스트리밍 실행과 동일하다** —
적분은 physics_dt 단위로 이뤄지고 벽시계와 무관하기 때문이다. 달라지는 건 속도뿐이다.

[화면이 필요할 때와 아닐 때]
    눈으로 거동 확인          -> live_wppl12.py (근접선 표시 포함)
    반복 실험 / 수량 산정     -> 이 스크립트

[출력 4종]
    콘솔          진행 로그 (--log 로 파일에도 남긴다)
    --out    JSON KPI 요약 + 위반 이벤트 전체 + 목표 도달 이벤트 + 상태시간 분해
    --trace  NPZ  **매 스텝 시계열** — 아래 [전체 로그] 참조
    --record NPZ  30Hz 자세만. replay_wppl.py / render_video_wppl.py 재생용

[전체 로그 — --trace]
사후에 어떤 KPI 정의로도 다시 계산할 수 있도록 **원시값**을 남긴다. 기획안이
"가동률 정의를 1차 팀과 합의하기 전까지는 가능한 정의를 모두 계산해 둔다"고
못박아 둔 항목이라, 집계값이 아니라 시계열을 남기는 것이 맞다.

    t          [T]        시뮬 시각
    pose       [T,N,3]    x, y, yaw
    cmd        [T,N,4]    v_cmd, omega_cmd, wl, wr      (컨트롤러가 명령한 값)
    ctrl       [T,N,4]    err, lag, brake, turning      (컨트롤러 내부 상태)
    prog       [T,N,2]    ci(경로 인덱스), done
    phys       [T,N,5]    vx, vy, wz, wl_act, wr_act    (--trace-physics, PhysX 실측)
    dmin       [T]        그 스텝의 로봇 간 최소거리

`phys` 는 로봇마다 파이썬 왕복이 3회 더 늘어 느려진다. 기본은 꺼져 있다.
선회 응답(명령 각속도 도달 시간) 같은 물리 자체를 재려면 켠다.
"""
import argparse
import json
import math
import os
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--stage", default="v2/warehouse_v59_wppl12.usd")
ap.add_argument("--traj", default="v2/traj_wppl/fleet_12/trajectories.json")
ap.add_argument("--n", type=int, default=12)
ap.add_argument("--seconds", type=float, default=420.0)
ap.add_argument("--physics-dt", type=float, default=1.0 / 60.0)
ap.add_argument("--drv", default=os.path.expanduser("~/khs/wh"),
                help="amr_driver_v2.py 가 있는 폴더")
ap.add_argument("--out", default="")
ap.add_argument("--every", type=float, default=10.0, help="진행 로그 간격 [시뮬 초]")
ap.add_argument("--record", default="",
                help="실제 자세를 이 .npz 로 기록. replay_wppl.py 로 실시간 재생용")
ap.add_argument("--record-hz", type=float, default=30.0, help="기록 주기 [Hz]")
# --- 전체 로그 ---
ap.add_argument("--trace", default="",
                help="매 스텝 시계열을 이 .npz 로. 파일 상단 [전체 로그] 참조")
ap.add_argument("--trace-hz", type=float, default=0.0,
                help="시계열 기록 주기 [Hz]. 0 이면 **매 물리 스텝** (기본)")
ap.add_argument("--trace-physics", action="store_true",
                help="PhysX 실측 속도(선속·각속·실제 바퀴속도)도 기록. 느려진다")
ap.add_argument("--log", default="",
                help="콘솔 출력을 이 파일에도 남긴다 (파이썬 print 한정)")
ap.add_argument("--goals", default="",
                help="goals.json 경로. 주면 목표 도달 시각을 이벤트로 남긴다")
ap.add_argument("--goal-eps", type=float, default=0.35, help="목표 도달 판정 [m]")
args = ap.parse_args()


class _Tee:
    """print 출력을 화면과 파일에 동시에 쓴다.

    **Kit(C++) 부팅 로그는 못 잡는다** — 그쪽은 파이썬 sys.stdout 을 거치지 않고
    fd 1 에 직접 쓴다. 콘솔 전체를 통째로 남기려면 셸에서 리다이렉션할 것:
        python headless_wppl.py ... 2>&1 | tee logs/hl_$(date +%Y%m%d_%H%M%S).log
    """

    def __init__(self, path, stream):
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        self.f = open(path, "w", encoding="utf-8", buffering=1)
        self.s = stream

    def write(self, x):
        self.s.write(x)
        self.f.write(x)

    def flush(self):
        self.s.flush()
        self.f.flush()


if args.log:
    sys.stdout = _Tee(args.log, sys.stdout)

from isaacsim import SimulationApp                      # noqa: E402
simulation_app = SimulationApp({"headless": True})

# amr_driver_v2 는 --drv 폴더 또는 그 아래 path/ 에 있다 (2026-09-06 재구성).
for _p in (os.path.join(os.path.abspath(args.drv), "path"), os.path.abspath(args.drv)):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
import numpy as np                                      # noqa: E402
from amr_driver_v2 import (PathFollower, load_paths, yaw_from_quat,  # noqa: E402
                           fleet_safety, LEFT_IDX, RIGHT_IDX,
                           WHEEL_R, WHEEL_BASE)
from isaacsim.core.api import World                     # noqa: E402
from isaacsim.core.utils.stage import open_stage        # noqa: E402
from isaacsim.core.prims import SingleArticulation      # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

# 로봇 간 안전거리. **드라이버는 이 이름을 내보내지 않는다** — 예전에 여기서
# `from amr_driver_v2 import SAFETY_DIST` 를 하는 바람에 ImportError 로 즉사했다
# (2026-09-03 서버 첫 실행에서 발견). 드라이버가 나중에 정의하면 그 값을 쓰고,
# 없으면 config.py 와 같은 2 * ROBOT_RADIUS = 1.577 을 쓴다.
SAFETY_DIST = float(getattr(sys.modules["amr_driver_v2"], "SAFETY_DIST", 1.577))
PLAN_MIN = 1.697                    # 계획이 보장하는 최소 = PITCH / sqrt(2)

stage_p = os.path.abspath(args.stage)
traj_p = os.path.abspath(args.traj)
print(f"[hl] stage {stage_p}", flush=True)
print(f"[hl] traj  {traj_p}", flush=True)
if not os.path.isfile(stage_p):
    raise SystemExit(f"USD 를 찾을 수 없습니다: {stage_p}")

open_stage(stage_p)
world = World(stage_units_in_meters=1.0,
              physics_dt=args.physics_dt, rendering_dt=args.physics_dt)
world.reset()

paths = load_paths(traj_p, args.n)
arts, fols = [], []
for i, (rid, wp) in enumerate(paths):
    a = SingleArticulation(f"/World/Robots/amr_{i}", name=f"hl_{i}")
    a.initialize()
    arts.append(a)
    fols.append(PathFollower(wp))
N = len(arts)
print(f"[hl] 로봇 {N}대 초기화 완료", flush=True)

# 목표열 — 있으면 도달 시각을 남긴다. 없으면 이 기능만 꺼진다.
goals = [[] for _ in range(N)]
if args.goals:
    with open(os.path.abspath(args.goals), encoding="utf-8") as f:
        gj = json.load(f)["robots"]
    for i in range(N):
        goals[i] = [tuple(p) for p in gj.get(str(i), [])]
    print(f"[hl] 목표열 로드: 대당 {len(goals[0])}개", flush=True)
gidx = [0] * N
arrivals = []

# 첫 스텝에서 물리 뷰가 만들어지므로 한 번 돌리고 핸들을 다시 잡는다
world.step(render=False)
for a in arts:
    a.initialize()

steps = int(args.seconds / args.physics_dt)
log_every = max(1, int(args.every / args.physics_dt))
t_sim = 0.0
dmin, dmin_t, viol = 1e9, 0.0, 0
viol_events = []
active = {}
rec_every = max(1, int(round((1.0 / args.record_hz) / args.physics_dt)))
rec_t, rec_pose = [], []

# --- 전체 로그 버퍼: 미리 잡는다 (리스트 append 는 25,200스텝에서 GC 부담이 크다) ---
tr_every = 1 if args.trace_hz <= 0 else max(
    1, int(round((1.0 / args.trace_hz) / args.physics_dt)))
tr_cap = (steps // tr_every) + 2 if args.trace else 0
tr_n = 0
if args.trace:
    tr_t = np.zeros(tr_cap, dtype=np.float32)
    tr_dmin = np.zeros(tr_cap, dtype=np.float32)
    tr_pose = np.zeros((tr_cap, N, 3), dtype=np.float32)
    tr_cmd = np.zeros((tr_cap, N, 4), dtype=np.float32)
    tr_ctrl = np.zeros((tr_cap, N, 4), dtype=np.float32)
    tr_prog = np.zeros((tr_cap, N, 2), dtype=np.float32)
    tr_phys = np.zeros((tr_cap, N, 5), dtype=np.float32) if args.trace_physics else None
    mb = sum(x.nbytes for x in (tr_t, tr_dmin, tr_pose, tr_cmd, tr_ctrl, tr_prog)) / 1e6
    mb += tr_phys.nbytes / 1e6 if tr_phys is not None else 0
    print(f"[hl] 시계열 기록: {tr_cap}프레임 x {N}대  (버퍼 {mb:.0f} MB, "
          f"{'매 스텝' if tr_every == 1 else f'{args.trace_hz:g}Hz'}"
          f"{', PhysX 실측 포함' if args.trace_physics else ''})", flush=True)

# 스텝별 err/lag 은 누적합만 있으므로 직전 값과의 차로 되돌린다
prev_err_sum = [0.0] * N
prev_lag_sum = [0.0] * N
prev_nerr = [0] * N
wall0 = time.time()

for n in range(1, steps + 1):
    poses = []
    bad = False
    for a in arts:
        p, q = a.get_world_pose()
        if p is None or q is None:
            bad = True
            break
        poses.append((float(p[0]), float(p[1]), yaw_from_quat(q)))
    if bad:
        print(f"[hl] pose 핸들 무효 (n={n}) — 중단", flush=True)
        break

    close = []
    step_dmin = 1e9
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            d = math.hypot(poses[i][0] - poses[j][0], poses[i][1] - poses[j][1])
            if d < step_dmin:
                step_dmin = d
            if d < dmin:
                dmin, dmin_t = d, t_sim
            if d < SAFETY_DIST:
                viol += 1
            if d < PLAN_MIN:
                close.append((i, j, d))

    # 위반 진입/해제만 이벤트로 남긴다 (매 스텝 기록하면 파일이 못 쓰게 된다)
    dmap = {(i, j): d for i, j, d in close}
    now = {k for k, d in dmap.items() if d < SAFETY_DIST}
    for k in now - set(active):
        active[k] = [t_sim, dmap[k]]
    for k in set(active) - now:
        t0, dlow = active.pop(k)
        viol_events.append(dict(pair=[k[0], k[1]], t_start=round(t0, 2),
                                t_end=round(t_sim, 2), d_min=round(dlow, 3)))
    for k in now:
        if dmap[k] < active[k][1]:
            active[k][1] = dmap[k]

    # 목표 도달 — 순서대로만 전진시킨다
    for i in range(N):
        if gidx[i] < len(goals[i]):
            gx, gy = goals[i][gidx[i]]
            if math.hypot(poses[i][0] - gx, poses[i][1] - gy) < args.goal_eps:
                arrivals.append(dict(robot=i, goal=gidx[i], t=round(t_sim, 2),
                                     x=round(gx, 2), y=round(gy, 2)))
                gidx[i] += 1

    fleet_safety(poses, fols)

    cmds = []
    for a, f, ps in zip(arts, fols, poses):
        wl, wr = f.step(ps[0], ps[1], ps[2], args.physics_dt, t_sim)
        cmds.append((wl, wr))
        a.apply_action(ArticulationAction(
            joint_velocities=np.array([wl, wr]),
            joint_indices=np.array([LEFT_IDX, RIGHT_IDX])))

    if args.record and (n % rec_every == 0 or n == 1):
        rec_t.append(t_sim)
        rec_pose.append([(p_[0], p_[1], p_[2]) for p_ in poses])

    if args.trace and (n % tr_every == 0 or n == 1) and tr_n < tr_cap:
        tr_t[tr_n] = t_sim
        tr_dmin[tr_n] = step_dmin if step_dmin < 1e8 else np.nan
        for i, (f, ps, (wl, wr)) in enumerate(zip(fols, poses, cmds)):
            tr_pose[tr_n, i] = ps
            # 바퀴 명령 -> (v, omega). 차동구동 역변환이라 손실이 없다.
            tr_cmd[tr_n, i] = ((wr + wl) * WHEEL_R / 2.0,
                               (wr - wl) * WHEEL_R / WHEEL_BASE, wl, wr)
            de = f.err_sum - prev_err_sum[i]
            dl = f.lag_sum - prev_lag_sum[i]
            dn = f.n_err - prev_nerr[i]
            prev_err_sum[i], prev_lag_sum[i], prev_nerr[i] = f.err_sum, f.lag_sum, f.n_err
            tr_ctrl[tr_n, i] = (de / dn if dn else 0.0, dl / dn if dn else 0.0,
                                f.brake, 1.0 if f.turning else 0.0)
            tr_prog[tr_n, i] = (f.ci, 1.0 if f.done else 0.0)
            if tr_phys is not None:
                lv = arts[i].get_linear_velocity()
                av = arts[i].get_angular_velocity()
                jv = arts[i].get_joint_velocities(
                    joint_indices=np.array([LEFT_IDX, RIGHT_IDX]))
                tr_phys[tr_n, i] = (
                    float(lv[0]) if lv is not None else np.nan,
                    float(lv[1]) if lv is not None else np.nan,
                    float(av[2]) if av is not None else np.nan,   # yaw rate
                    float(jv[0]) if jv is not None else np.nan,
                    float(jv[1]) if jv is not None else np.nan)
        tr_n += 1

    world.step(render=False)
    t_sim += args.physics_dt

    if n % log_every == 0:
        ss = [f.stats() for f in fols]
        wall = time.time() - wall0
        print(f"[hl] t={t_sim:.0f}s  평균진행 {sum(s['pct'] for s in ss)/len(ss):.1f}%  "
              f"완주 {sum(1 for s in ss if s['done'])}/{len(ss)}  "
              f"총이동 {sum(s['travel_m'] for s in ss):.0f}m  "
              f"최대평균오차 {max(s['err_mean'] for s in ss):.2f}m  "
              f"|| 최소 {dmin:.3f}m(t={dmin_t:.0f}s) 위반 {viol}스텝  "
              f"|| 목표도달 {len(arrivals)}회"
              f"|| 벽시계 {wall:.0f}s 배속 {t_sim/max(wall,1e-9):.2f}x", flush=True)

for k, (t0, dlow) in active.items():                    # 종료 시점에 열려 있던 위반
    viol_events.append(dict(pair=[k[0], k[1]], t_start=round(t0, 2),
                            t_end=round(t_sim, 2), d_min=round(dlow, 3)))

# --- 상태 시간 분해 ---
# 가동률 정의가 1차 팀과 아직 합의되지 않았다. 그래서 **분모·분자를 나눠 놓고**
# 가능한 정의를 모두 계산해 둔다 (기획안 1.6 "전제와 위험" 참조).
#   주행    바퀴 명령 선속도가 0.05 m/s 를 넘음
#   선회    제자리 선회 중 (v=0, omega!=0)
#   혼잡대기 fleet_safety 가 제동을 걸어 멈춤 (brake < 1)
#   계획대기 계획이 요구한 정지 = 스테이션 작업(dwell). 제동이 아닌 정지
#   완주    경로 끝
state_s = {}
if args.trace and tr_n:
    dt_tr = args.physics_dt * tr_every
    v = tr_cmd[:tr_n, :, 0]
    w = tr_cmd[:tr_n, :, 1]
    br = tr_ctrl[:tr_n, :, 2]
    tn = tr_ctrl[:tr_n, :, 3] > 0.5
    dn = tr_prog[:tr_n, :, 1] > 0.5
    moving = (np.abs(v) > 0.05) & ~dn
    turning = tn & ~moving & ~dn
    stopped = ~moving & ~turning & ~dn
    braked = stopped & (br < 0.999)
    dwell = stopped & (br >= 0.999)
    tot = tr_n * N * dt_tr
    state_s = dict(
        total_robot_s=round(tot, 1),
        move_s=round(float(moving.sum()) * dt_tr, 1),
        turn_s=round(float(turning.sum()) * dt_tr, 1),
        congestion_wait_s=round(float(braked.sum()) * dt_tr, 1),
        plan_wait_s=round(float(dwell.sum()) * dt_tr, 1),
        done_s=round(float(dn.sum()) * dt_tr, 1),
    )
    busy = state_s["move_s"] + state_s["turn_s"]
    state_s["util_move_only"] = round(100.0 * state_s["move_s"] / tot, 2)
    state_s["util_move_turn"] = round(100.0 * busy / tot, 2)
    state_s["util_incl_work"] = round(
        100.0 * (busy + state_s["plan_wait_s"]) / tot, 2)

ss = [f.stats() for f in fols]
wall = time.time() - wall0
kpi = dict(
    stage=stage_p, traj=traj_p, goals=os.path.abspath(args.goals) if args.goals else "",
    n_robots=N,
    sim_seconds=round(t_sim, 2), wall_seconds=round(wall, 2),
    realtime_factor=round(t_sim / max(wall, 1e-9), 3),
    physics_dt=args.physics_dt,
    done=sum(1 for s in ss if s["done"]),
    pct_mean=round(sum(s["pct"] for s in ss) / len(ss), 2),
    travel_total_m=round(sum(s["travel_m"] for s in ss), 1),
    err_mean_max=round(max(s["err_mean"] for s in ss), 3),
    err_max=round(max(s["err_max"] for s in ss), 3),
    dmin_m=round(dmin, 3), dmin_t=round(dmin_t, 2),
    safety_dist=SAFETY_DIST, plan_min=PLAN_MIN,
    violation_steps=viol, violation_events=len(viol_events),
    goals_reached=len(arrivals),
    goals_per_robot=[sum(1 for a_ in arrivals if a_["robot"] == i) for i in range(N)],
    state_seconds=state_s,
    per_robot=[dict(id=i, **s) for i, s in enumerate(ss)],
    # 위반은 **전부** 남긴다. 예전엔 상위 50건만 남겨서 사후 분석이 막혔다.
    events=sorted(viol_events, key=lambda e: e["d_min"]),
    arrivals=arrivals,
)
print("\n[hl] ===== 요약 =====", flush=True)
print(f"  배속        {kpi['realtime_factor']}x  (시뮬 {kpi['sim_seconds']}s / 벽시계 {kpi['wall_seconds']}s)", flush=True)
print(f"  완주        {kpi['done']}/{kpi['n_robots']}  평균진행 {kpi['pct_mean']}%", flush=True)
print(f"  추종오차    평균최대 {kpi['err_mean_max']}m  최대 {kpi['err_max']}m", flush=True)
print(f"  로봇간최소  {kpi['dmin_m']}m (t={kpi['dmin_t']}s)  기준 {SAFETY_DIST}m", flush=True)
print(f"  위반        {kpi['violation_steps']}스텝 / {kpi['violation_events']}건", flush=True)
if goals[0]:
    print(f"  목표도달    {kpi['goals_reached']}회  대당 {kpi['goals_reached']/N:.2f}", flush=True)
if state_s:
    print(f"  가동률      이동만 {state_s['util_move_only']}%  "
          f"이동+선회 {state_s['util_move_turn']}%  "
          f"작업포함 {state_s['util_incl_work']}%", flush=True)
    print(f"  대기        혼잡 {state_s['congestion_wait_s']}s  "
          f"계획(작업) {state_s['plan_wait_s']}s", flush=True)
for e in kpi["events"][:5]:
    print(f"    amr_{e['pair'][0]}-amr_{e['pair'][1]}  {e['d_min']}m  "
          f"t={e['t_start']}~{e['t_end']}s", flush=True)

if args.record:
    rp = os.path.abspath(args.record)
    np.savez_compressed(rp,
                        t=np.asarray(rec_t, dtype=np.float32),
                        pose=np.asarray(rec_pose, dtype=np.float32),
                        stage=np.array(stage_p))
    print(f"[hl] 자세 기록: {rp}  ({len(rec_t)} 프레임 @ {args.record_hz:.0f}Hz)", flush=True)

if args.trace and tr_n:
    tp = os.path.abspath(args.trace)
    os.makedirs(os.path.dirname(tp) or ".", exist_ok=True)
    cols = dict(
        t=tr_t[:tr_n], dmin=tr_dmin[:tr_n], pose=tr_pose[:tr_n],
        cmd=tr_cmd[:tr_n], ctrl=tr_ctrl[:tr_n], prog=tr_prog[:tr_n],
        dt=np.float32(args.physics_dt * tr_every), stage=np.array(stage_p),
        traj=np.array(traj_p),
        cmd_cols=np.array(["v_cmd", "omega_cmd", "wl", "wr"]),
        ctrl_cols=np.array(["err", "lag", "brake", "turning"]),
        prog_cols=np.array(["ci", "done"]),
    )
    if tr_phys is not None:
        cols["phys"] = tr_phys[:tr_n]
        cols["phys_cols"] = np.array(["vx", "vy", "wz", "wl_act", "wr_act"])
    np.savez_compressed(tp, **cols)
    print(f"[hl] 시계열 기록: {tp}  ({tr_n} 프레임 x {N}대, "
          f"{os.path.getsize(tp)/1e6:.1f} MB)", flush=True)

if args.out:
    with open(os.path.abspath(args.out), "w", encoding="utf-8") as f:
        json.dump(kpi, f, ensure_ascii=False, indent=1)
    print(f"[hl] 저장: {os.path.abspath(args.out)}", flush=True)

simulation_app.close()
