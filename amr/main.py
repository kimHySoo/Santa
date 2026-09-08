#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AMR 시뮬레이션 단일 진입점.

    python amr/main.py list                       계획기 목록
    python amr/main.py plan   --planner wppl      계획 (로컬, GPU 불필요)
    python amr/main.py verify --planner wppl      검증
    python amr/main.py cmds   --planner wppl      서버에서 칠 명령 출력
    python amr/main.py run    --planner wppl      서버에서 **실제로 실행**

계획기를 바꾸려면 `--planner` 하나만 바꾼다. 새 계획기는 `amr/path/` 에
모듈을 추가하면 자동으로 잡힌다 — 이 파일은 손대지 않는다.

[왜 이 파일이 있나]
계획기가 셋(wppl · astar · pibt_h)인데 각각 CLI 인자와 산출물 위치가 달랐다.
어느 게 현행인지, 서버에서 무슨 명령을 쳐야 하는지가 파일 이름에 안 드러나서
매번 문서를 뒤져야 했다. 여기 한 곳만 보면 되게 한다.

[실행]
`run` 은 서버에서만 쓴다. `paths.sh` 를 먼저 source 해야 하므로 셸 껍데기
(`v2/server/run.sh`)가 하나 있는데, **그 안에는 계획기 정보가 없다.** 환경만
잡고 이 파일로 넘긴다 — 계획기 표가 두 곳에 생기면 나눈 의미가 없다.

[계획과 실행의 경계]
계획은 GPU 가 필요 없어 **로컬에서 끝낸다.** 씬 빌드와 주행은 Isaac Sim 이
필요해 **서버에서** 한다. 그래서 `cmds` 는 실행하지 않고 명령만 찍는다 —
로컬에서 서버 명령을 흉내 내면 경로가 어긋난다 (2026-09-06 에 상대경로로
USD 참조가 깨진 적이 있다).
"""
import argparse
import os
import sys

# 윈도우 콘솔은 기본 cp949 라 '—' 같은 문자에서 UnicodeEncodeError 로 죽는다.
# 출력만 UTF-8 로 돌린다 (파이프로 넘겨도 동일하게 나온다).
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amr import path as P                                  # noqa: E402


def cmd_list(args):
    reg = P.registry()
    print(f"계획기 {len(reg)}개\n")
    for name in sorted(reg):
        m = reg[name]
        drive = {"time": "시각 추종 (amr_driver_v2)",
                 "adg": "ADG 순서 (isaac_drive)"}.get(m.DRIVE, m.DRIVE)
        print(f"  {name:8s} {m.DESC}")
        print(f"  {'':8s}   실행 {drive}")
        print(f"  {'':8s}   산출 {m.OUT}")
        if hasattr(m, "GOOD_SEEDS"):
            print(f"  {'':8s}   완주 확인 시드 {m.GOOD_SEEDS}")
        print()
    print(f"기본 맵: {P.default_map()}")


def cmd_plan(args):
    m = P.get(args.planner)
    print(f"== 계획  [{m.NAME}]  {m.DESC}")
    r = m.plan(n=args.n, seed=args.seed, seconds=args.seconds,
               map_dir=args.map, extra=args.extra)
    print(f"\n== 산출물: {r['out']}")
    if r.get("note"):
        print(f"   주의: {r['note']}")


def cmd_verify(args):
    m = P.get(args.planner)
    print(f"== 검증  [{m.NAME}]")
    if not hasattr(m, "verify"):
        print("   이 계획기에는 검증기가 없습니다.")
        return
    ok = m.verify(n=args.n, map_dir=args.map)
    print("\n== " + ("통과" if ok else "★ 실패 — 위 항목을 보세요"))
    if not ok:
        raise SystemExit(1)


def cmd_cmds(args):
    m = P.get(args.planner)
    print(f"# {m.NAME} — {m.DESC}")
    print("# 서버(jupyter04)에서 실행. 경로는 전부 절대경로여야 한다.\n")
    print("cd ~/khs/wh")
    print("source ~/khs/wh/paths.sh"
          "      # W · SCENE · MAP · ROBOT · STAGE · PLAN · LAN_IP · GPU 정의")
    print()
    print("# 1) 씬 빌드")
    for ln in m.scene_cmd(args.n):
        print(ln)
    print("\n# 2) 주행 + WebRTC")
    print("pkill -f 'kit/kit'; pkill -f isaacsim; sleep 5")
    for ln in m.run_cmd(args.n):
        print(ln)


def cmd_run(args):
    """서버에서 실제로 실행한다. `cmds` 가 찍는 것과 **같은 정의**를 쓴다."""
    import subprocess

    m = P.get(args.planner)
    n = args.n
    need = ["W", "SCENE", "ROBOT", "PLAN", "STAGE", "LAN_IP", "GPU"]
    miss = [k for k in need if not os.environ.get(k)]
    if miss:
        raise SystemExit(f"환경변수 없음: {', '.join(miss)}\n"
                         "  source ~/khs/wh/paths.sh 를 먼저 실행하세요.")

    from amr.path._common import expand, server_paths
    p = {k: expand(v) for k, v in server_paths(m.SUB, m.USD % n, n).items()}
    print(f"== {m.NAME} · {n}대")

    # 입력 확인. 여기서 막아야 원인이 보인다 — 궤적이 없으면 빌드가 빈 USD 를
    # 남기고, 다음 단계에서는 "Failed to get crate info" 만 보인다 (2026-09-06).
    miss = [f for f in (os.environ["SCENE"], os.environ["ROBOT"],
                        p["traj"], p["starts"]) if not os.path.exists(f)]
    if miss:
        for f in miss:
            print(f"  ★ 없음: {f}")
        raise SystemExit(f"\n  계획 산출물은 로컬에서 만들어 올린다:\n"
                         f"    python amr/main.py plan --planner {m.NAME} --n {n}")

    # 씬 빌드 — 궤적이 USD 보다 새것일 때만
    usd = p["usd"]
    stale = (args.rebuild or not os.path.exists(usd) or os.path.getsize(usd) == 0
             or os.path.getmtime(p["traj"]) > os.path.getmtime(usd))
    if stale:
        print(f"-- 씬 빌드 -> {usd}")
        sh = "\n".join(m.scene_cmd(n))
        if subprocess.run(["bash", "-c", sh], cwd=os.environ["W"]).returncode:
            raise SystemExit("  ★ 씬 빌드 실패 — 위 [s] 줄을 보세요")
    else:
        kb = os.path.getsize(usd) / 1024
        print(f"-- 씬 재사용: {usd} ({kb:.1f} KB)   다시 빌드하려면 --rebuild")
    if not os.path.getsize(usd):
        raise SystemExit("  ★ 씬이 0바이트입니다")

    # 이전 Kit 정리 — 안 죽이면 NVST_R_BUSY 가 도배된다
    if subprocess.run(["pgrep", "-f", "kit/kit|isaacsim"],
                      capture_output=True).returncode == 0:
        print("-- 이전 Kit 종료")
        subprocess.run(["bash", "-c", "pkill -f 'kit/kit'; pkill -f isaacsim; sleep 5"])
        if subprocess.run(["pgrep", "-f", "kit/kit|isaacsim"],
                          capture_output=True).returncode == 0:
            subprocess.run(["bash", "-c", "pkill -9 -f 'kit/kit'; sleep 3"])

    env, argv = m.launch(n).argv()
    if args.lite:
        env["LIVE_LITE"] = "1"
    print(f"-- 실행.  WebRTC 클라이언트 Server 에 [ {os.environ['LAN_IP']} ] 를 넣고 Connect")
    print("   (기동 약 12초. 'Streaming server started' 뒤에 붙는다.  종료 Ctrl+C)\n")
    os.execvpe(argv[0], argv, dict(os.environ, **env))


def main():
    ap = argparse.ArgumentParser(description="AMR 시뮬레이션 진입점")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--planner", default="pibt_h", help="amr/path/ 의 계획기 이름")
        p.add_argument("--n", type=int, default=12, help="로봇 대수")
        p.add_argument("--map", default=None, help="맵 폴더 (기본: upstream v5.9)")

    p = sub.add_parser("list", help="계획기 목록")
    p.set_defaults(fn=cmd_list)

    p = sub.add_parser("plan", help="계획 (로컬)")
    common(p)
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--seconds", type=float, default=420)
    p.add_argument("--extra", nargs=argparse.REMAINDER, default=[],
                   help="계획기 CLI 에 그대로 넘길 인자")
    p.set_defaults(fn=cmd_plan)

    p = sub.add_parser("verify", help="산출물 검증")
    common(p)
    p.set_defaults(fn=cmd_verify)

    p = sub.add_parser("cmds", help="서버에서 칠 명령 출력")
    common(p)
    p.set_defaults(fn=cmd_cmds)

    p = sub.add_parser("run", help="서버에서 실행 (paths.sh 를 먼저 source)")
    common(p)
    p.add_argument("--rebuild", action="store_true", help="씬을 강제로 다시 빌드")
    p.add_argument("--lite", action="store_true",
                   help="랙 등 시각 전용 지오메트리를 숨긴다 (배속용. 물리 동일)")
    p.set_defaults(fn=cmd_run)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
