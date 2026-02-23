#!/usr/bin/env python3
import argparse
import glob
import math
import os
import sys
import yaml


def is_num(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(float(x))


def lint_file(path: str, max_jump: float) -> int:
    data = yaml.safe_load(open(path, "r"))
    if not isinstance(data, dict) or "waypoints" not in data:
        print(f"⛔ {path}: falta clave 'waypoints'")
        return 2
    wps = data["waypoints"]
    if not isinstance(wps, list) or len(wps) == 0:
        print(f"⛔ {path}: waypoints vacío")
        return 2

    err = 0
    prev = None
    for i, w in enumerate(wps):
        if not isinstance(w, dict):
            print(f"⛔ {path}: waypoint[{i}] no es dict")
            err = 2
            continue
        for k in ("x", "y"):
            if k not in w or not is_num(w[k]):
                print(f"⛔ {path}: waypoint[{i}] {k} inválido -> {w.get(k)}")
                err = 2
        if "yaw" not in w or not is_num(w["yaw"]):
            print(f"⛔ {path}: waypoint[{i}] yaw inválido -> {w.get('yaw')}")
            err = 2

        if prev is not None and err == 0:
            dx = float(w["x"]) - float(prev["x"])
            dy = float(w["y"]) - float(prev["y"])
            d = math.hypot(dx, dy)
            if d > max_jump:
                print(f"⚠️  {path}: salto grande {d:.2f} m entre {i-1}->{i} (¿mapa equivocado / ruta mala?)")
        prev = w

    if err == 0:
        print(f"✅ {path}: OK ({len(wps)} puntos)")
    return err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("routes_dir", nargs="?", default="/routes")
    ap.add_argument("--max-jump", type=float, default=5.0)
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.routes_dir, "*.yaml")))
    if not files:
        print(f"⛔ No hay rutas .yaml en {args.routes_dir}")
        sys.exit(2)

    worst = 0
    for f in files:
        worst = max(worst, lint_file(f, args.max_jump))
    sys.exit(worst)


if __name__ == "__main__":
    main()
