# -*- coding: utf-8 -*-
"""path/battery.py — `lifelong` 에서 쓰는데 import 안 한 이름을 전부 찾아 넣는다.

무엇이 터졌나 (2026-09-11)
---------------------------
    File "path/battery.py", line 351, in _unpark_docks
        if r["state"] == TO_HOME and r["goal"] is not None:
    NameError: name 'TO_HOME' is not defined

`_unpark_docks` 는 저장소(`54a9276`)의 `path/battery.py` 에 없다.
`fms/sim_engine/kernel/battery.py:61` 에만 있고, 원본은

    from .states import IDLE, TO_HOME, FREED, TO_CHARGE, CHARGING, PARKABLE

로 `TO_HOME` 을 가져온다. 우리 쪽으로 옮겨 오면서 함수만 오고 import 가 안 왔다.

    path/battery.py:  from lifelong import (
                          CHARGING, FREED, IDLE, TO_CHARGE, TO_CHARGE_HI,
                          TO_DROP, TO_PICK, Kernel,
                      )        ← TO_HOME 없음

pitch 와 무관한 별개 버그다. `TO_HOME` 은 평소에 한 번도 안 나오는 상태라
(`claude/mir-fleet-배차-적용성-측정.md`: "`TO_HOME` 이 한 번도 발생하지 않는다")
이 분기에 처음 들어가는 실행에서만 터진다.

왜 한 줄로 안 고치나
--------------------
`TO_HOME` 만 넣으면 그 다음 이름에서 또 NameError 가 난다 — NameError 는 처음
하나에서 멈추기 때문에 한 번에 하나씩만 보인다. 그래서 추측하지 않고 **센다**:

    1. `path/lifelong.py` 의 모듈 최상위 이름을 전부 모은다 (AST)
    2. `path/battery.py` 가 **읽는** 이름 중 (AST: Name/Load)
       - lifelong 에 있고
       - battery 안에서 정의·대입·import 되지 않았고
       - 빌트인도 아닌 것
       을 고른다
    3. 그것들을 `from lifelong import (...)` 목록에 알파벳순으로 넣는다

`lifelong` 에 **없는** 이름은 고칠 수 없다 (예: `PARKABLE` 은 FMS 의
`kernel/states.py` 에만 있고 우리 `lifelong.py` 에는 없다). 그런 것은 고치지
않고 **어디에 정의돼 있는지 찾아서 알려 준다.** 조용히 넘기면 다음 실행에서
같은 자리에서 또 죽는다.

미정의 이름 목록은 `pyflakes` 가 있으면 그걸 쓴다 (스코프를 제대로 본다).
없으면 자체 AST 스캔으로 떨어지는데, 그건 파일 전체를 한 스코프로 보기 때문에
**놓칠 수 있다** — 그래서 pyflakes 설치를 권한다.

멱등이다. `.bak.<시각>` 을 남기고 `ast.parse` 와 `compile` 로 검사한다.
"""
import argparse
import ast
import builtins
import os
import shutil
import sys
import time

MARK = "# [patch_battery_names]"


def toplevel_names(path):
    """모듈이 최상위에서 정의/대입/import 하는 이름."""
    tree = ast.parse(open(path, encoding="utf-8").read(), path)
    out = set()
    for n in tree.body:
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            out.add(n.name)
        elif isinstance(n, ast.Assign):
            for t in n.targets:
                out |= {x.id for x in ast.walk(t) if isinstance(x, ast.Name)}
        elif isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name):
            out.add(n.target.id)
        elif isinstance(n, (ast.Import, ast.ImportFrom)):
            for al in n.names:
                out.add((al.asname or al.name).split(".")[0])
    return out


def bound_names(path):
    """파일 어디서든 바인딩되는 이름 (import·대입·def·인자·for·with·except)."""
    tree = ast.parse(open(path, encoding="utf-8").read(), path)
    out = set()
    for n in ast.walk(tree):
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            for al in n.names:
                out.add((al.asname or al.name).split(".")[0])
        elif isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            out.add(n.name)
        elif isinstance(n, ast.Name) and isinstance(n.ctx, (ast.Store, ast.Del)):
            out.add(n.id)
        elif isinstance(n, ast.arg):
            out.add(n.arg)
        elif isinstance(n, ast.ExceptHandler) and n.name:
            out.add(n.name)
        elif isinstance(n, (ast.Global, ast.Nonlocal)):
            out |= set(n.names)
    return out


def loaded_names(path):
    tree = ast.parse(open(path, encoding="utf-8").read(), path)
    return {n.id for n in ast.walk(tree)
            if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}


def undefined_via_pyflakes(path):
    """pyflakes 로 미정의 이름을 뽑는다. 없으면 None."""
    try:
        import subprocess
        r = subprocess.run([sys.executable, "-m", "pyflakes", path],
                           capture_output=True, text=True)
    except Exception:
        return None
    if r.returncode not in (0, 1) or ("No module named" in r.stderr):
        return None
    out = set()
    for ln in r.stdout.splitlines():
        if "undefined name" in ln:
            out.add(ln.rsplit("'", 2)[-2])
    return out


def find_definition(name, roots):
    """이 이름을 최상위에서 정의하는 파일을 찾는다."""
    hits = []
    for root in roots:
        for dp, _, fns in os.walk(root):
            for fn in fns:
                if not fn.endswith(".py"):
                    continue
                p = os.path.join(dp, fn)
                try:
                    if name in toplevel_names(p):
                        hits.append(p)
                except Exception:
                    pass
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--battery", default="path/battery.py")
    ap.add_argument("--lifelong", default="path/lifelong.py")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    for p in (a.battery, a.lifelong):
        if not os.path.isfile(p):
            raise SystemExit(f"★ {p} 가 없습니다. 저장소 루트에서 실행하세요.")

    src = open(a.battery, encoding="utf-8").read()
    avail = toplevel_names(a.lifelong)
    print(f"[in] {a.battery}  ·  lifelong 최상위 이름 {len(avail)}개")

    undef = undefined_via_pyflakes(a.battery)
    if undef is None:
        used, bound = loaded_names(a.battery), bound_names(a.battery)
        undef = used - bound - set(dir(builtins))
        print("     판정: 자체 AST 스캔 (pyflakes 없음 — `pip install pyflakes` 를 권합니다.\n"
              "           자체 스캔은 파일 전체를 한 스코프로 봐서 놓칠 수 있습니다)")
    else:
        print("     판정: pyflakes")

    if not undef:
        print("\n미정의 이름이 없습니다 — 고칠 것이 없습니다.")
        return 0

    def where(m):
        for i, ln in enumerate(src.splitlines(), 1):
            if m in ln and not ln.lstrip().startswith("#"):
                return f":{i}  {ln.strip()[:64]}"
        return ""

    missing = sorted(undef & avail)
    other = sorted(undef - avail)
    print(f"\n★ 미정의 이름 {len(undef)}개")
    if missing:
        print(f"\n  [고칠 수 있음] lifelong 에 있는데 import 안 됨 — {len(missing)}개")
        for m in missing:
            print(f"     {m:<16} {where(m)}")
    if other:
        print(f"\n  [★ 손으로 고쳐야 함] lifelong 에 없는 이름 — {len(other)}개")
        roots = [os.path.dirname(a.battery) or ".", "fms", "amr"]
        roots = [r for r in roots if os.path.isdir(r)]
        for m in other:
            print(f"     {m:<16} {where(m)}")
            hits = find_definition(m, roots)
            if hits:
                for h in hits[:3]:
                    print(f"        └ 정의 있음: {h}")
            else:
                print(f"        └ 저장소에서 정의를 못 찾음 — 새로 정의해야 합니다")
        print("\n     이 이름들은 이 패치가 손대지 않습니다. 그대로 두면 같은 자리에서")
        print("     또 NameError 가 납니다.")
    if not missing:
        print("\nlifelong 에서 가져올 것이 없어 import 문은 그대로 둡니다.")
        return 1

    # from lifelong import (...) 블록을 찾아 이름을 합친다
    tree = ast.parse(src, a.battery)
    node = None
    for n in tree.body:
        if isinstance(n, ast.ImportFrom) and n.module == "lifelong" and n.level == 0:
            node = n
            break
    if node is None:
        raise SystemExit("★ `from lifelong import ...` 문을 못 찾았습니다.")

    have = [(al.asname or al.name) for al in node.names]
    if any(al.name == "*" for al in node.names):
        raise SystemExit("★ `from lifelong import *` 입니다 — 손으로 고치세요.")
    names = sorted(set(have) | set(missing),
                   key=lambda s: (s[0].islower(), s))     # 상수 먼저, 그다음 클래스
    lines = src.splitlines(keepends=True)
    body = ", ".join(names)
    wrapped, cur = [], "    "
    for tok in body.split(", "):
        add = tok + ", "
        if len(cur) + len(add) > 76:
            wrapped.append(cur.rstrip())
            cur = "    "
        cur += add
    wrapped.append(cur.rstrip().rstrip(","))
    new_stmt = (f"{MARK} lifelong 에서 쓰는 이름을 전부 가져온다.\n"
                f"#   _unpark_docks 를 fms/sim_engine/kernel/battery.py 에서 옮겨 올 때\n"
                f"#   함수만 오고 import 가 안 와서 TO_HOME 이 NameError 였다 (2026-09-11).\n"
                f"from lifelong import (\n" + "\n".join(wrapped) + ",\n)\n")
    out = "".join(lines[:node.lineno - 1]) + new_stmt + "".join(lines[node.end_lineno:])

    try:
        compile(out, a.battery, "exec")
    except SyntaxError as e:
        raise SystemExit(f"★ 결과가 문법 오류입니다: {e}")
    print(f"\n  ✓ compile 통과 · import 이름 {len(have)} -> {len(names)}")

    if a.dry_run:
        print("\n--- 새 import 문 ---")
        print(new_stmt)
        print("--dry-run — 저장하지 않았습니다.")
        return 0

    b = a.battery + time.strftime(".bak.%Y%m%d_%H%M%S")
    shutil.copy2(a.battery, b)
    open(a.battery, "w", encoding="utf-8").write(out)
    print(f"\n[bak] {b}\n[out] {a.battery}")

    print(r"""
=============================================================
확인
=============================================================
  cd /home/j-j15a106/khs/wh
  source ~/khs/venv/bin/activate

  # 1) 남은 미정의 이름이 없는지
  pip install pyflakes
  python -m pyflakes path/battery.py | grep -i undefined
    → 아무것도 안 나와야 한다

  # 위 [★ 손으로 고쳐야 함] 목록이 있었다면 그것부터 처리한다.
  #   PARKABLE 처럼 FMS 의 kernel/states.py 에만 있는 이름이면,
  #   lifelong.py 에 같은 정의를 추가하거나 이식한 함수에서 빼야 한다.

  # 2) 계획만 다시
  REPLAN=1 bash run_record.sh 12

=============================================================
이 버그가 왜 지금 나왔나
=============================================================
  `TO_HOME` 은 평소에 한 번도 안 나오는 상태다 —
  claude/mir-fleet-배차-적용성-측정.md 실측:

      "TO_HOME 이 한 번도 발생하지 않는다. Kernel.step 에서 assign(t) 이
       FREED -> TO_HOME 블록보다 먼저 돌고, 큐가 비는 일이 없어서
       FREED 로봇은 집에 갈 틈 없이 다음 태스크를 받는다 (FREED 가 0.4%)."

  그런데 `_unpark_docks` 는 상태와 무관하게 **매 배차마다 전 로봇을 훑는다.**
  그래서 이 함수가 들어온 순간부터는 첫 배차에서 바로 터진다. 즉 pitch 1.0
  때문이 아니라 **이 함수가 추가된 뒤 첫 실행**이라서 나온 것이다.
  pitch 1.2 로 되돌려도 같은 곳에서 죽는다 — 확인해 보면 안다.
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
