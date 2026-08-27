#!/usr/bin/env python3
"""
calibrar_homografia.py
──────────────────────────────────────────────────────────────────────────────
Calcula e salva a homografia pixel→cm usando os 4 marcadores ArUco do chão.

Layout físico (cm), origem no canto superior esquerdo da arena:
    ID 3: (  0,   0)   ID 0: (120,   0)
    ID 2: (  0, 170)   ID 1: (120, 170)

Uso:
    python3 calibrar_homografia.py
    python3 calibrar_homografia.py --url rtsp://...  --out homografia_plano.pkl
"""

import argparse
import os
import pickle
import sys
import time

import cv2
import numpy as np

# ── Posições mundo dos 4 ArUcos (cm) ─────────────────────────────────────────
# Ajuste aqui se a geometria mudar.
WORLD_POSITIONS = {
    0: (120.0,   0.0),
    1: (120.0, 170.0),
    2: (  0.0, 170.0),
    3: (  0.0,   0.0),
}

# Tamanho do marcador em pixels para desenho (não afeta cálculo)
ARUCO_DICT_NAME = "DICT_4X4_50"

# ── Argumentos ────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Calibração de homografia por ArUco")
    p.add_argument(
        "--url",
        default=os.getenv(
            "RTSP_URL",
            "rtsp://admin:nupedee7@192.168.1.4:554"
            "/cam/realmonitor?channel=1&subtype=1&proto=Onvif",
        ),
        help="URL do stream RTSP",
    )
    p.add_argument(
        "--out",
        default="homografia_plano.pkl",
        help="Arquivo de saída (.pkl)",
    )
    p.add_argument(
        "--frames",
        type=int,
        default=60,
        help="Número de frames para acumular detecções antes de calcular (default: 60)",
    )
    p.add_argument(
        "--min-detections",
        type=int,
        default=10,
        help="Mínimo de detecções por marcador para aceitar (default: 10)",
    )
    return p.parse_args()


# ── Abertura do stream ────────────────────────────────────────────────────────
def abrir_rtsp(url: str) -> cv2.VideoCapture:
    os.environ.setdefault(
        "OPENCV_FFMPEG_CAPTURE_OPTIONS",
        "rtsp_transport;tcp|buffer_size;1048576|stimeout;7000000",
    )
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    if not cap.isOpened():
        print(f"[ERRO] Não foi possível abrir: {url}")
        sys.exit(1)
    return cap


# ── Detector ArUco ────────────────────────────────────────────────────────────
def criar_detector():
    if not hasattr(cv2, "aruco"):
        print("[ERRO] OpenCV sem módulo aruco. Instale opencv-contrib-python.")
        sys.exit(1)
    aruco = cv2.aruco
    d = aruco.getPredefinedDictionary(getattr(aruco, ARUCO_DICT_NAME))
    try:
        params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(d, params)
        return detector, None
    except AttributeError:
        params = aruco.DetectorParameters_create()
        return None, (d, params)


def detectar(frame, detector, legacy):
    if detector is not None:
        corners, ids, _ = detector.detectMarkers(frame)
    else:
        d, params = legacy
        corners, ids, _ = cv2.aruco.detectMarkers(frame, d, parameters=params)
    return corners, ids


# ── Cálculo do centro de um marcador ─────────────────────────────────────────
def centro(corners_marker):
    c = corners_marker.reshape(4, 2)
    return c.mean(axis=0)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    print(f"[INFO] Conectando a: {args.url}")
    cap = abrir_rtsp(args.url)

    # Descarta frames iniciais do buffer
    for _ in range(20):
        cap.grab()

    detector, legacy = criar_detector()

    # Acumula centros detectados por ID
    acumulado: dict[int, list] = {k: [] for k in WORLD_POSITIONS}

    print(f"[INFO] Coletando detecções ({args.frames} frames)...")
    frames_lidos = 0
    t_start = time.monotonic()

    while frames_lidos < args.frames:
        cap.grab()
        ret, frame = cap.retrieve()
        if not ret:
            time.sleep(0.05)
            continue

        corners, ids = detectar(frame, detector, legacy)
        found_this = set()

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            for i, mid in enumerate(ids_flat):
                if mid in acumulado:
                    ctr = centro(corners[i])
                    acumulado[mid].append(ctr)
                    found_this.add(mid)

        frames_lidos += 1
        elapsed = time.monotonic() - t_start
        counts = {k: len(v) for k, v in acumulado.items()}
        print(
            f"\r[INFO] Frame {frames_lidos}/{args.frames} | "
            f"IDs detectados: {counts} | {elapsed:.1f}s",
            end="",
            flush=True,
        )

    cap.release()
    print()  # nova linha após o \r

    # ── Valida detecções ──────────────────────────────────────────────────────
    img_pts = []
    world_pts = []
    faltando = []

    for mid, pts in acumulado.items():
        if len(pts) < args.min_detections:
            faltando.append((mid, len(pts)))
        else:
            media = np.mean(pts, axis=0)
            img_pts.append(media)
            world_pts.append(WORLD_POSITIONS[mid])
            print(
                f"[INFO] ID {mid}: {len(pts)} detecções → "
                f"pixel médio ({media[0]:.1f}, {media[1]:.1f}) → "
                f"mundo ({WORLD_POSITIONS[mid][0]:.0f}, {WORLD_POSITIONS[mid][1]:.0f}) cm"
            )

    if faltando:
        for mid, n in faltando:
            print(
                f"[AVISO] ID {mid}: apenas {n} detecções "
                f"(mínimo={args.min_detections}) — IGNORADO"
            )

    if len(img_pts) < 4:
        print(
            f"[ERRO] Apenas {len(img_pts)} marcadores válidos — "
            "precisa de pelo menos 4. Verifique iluminação e visibilidade."
        )
        sys.exit(1)

    img_pts_arr  = np.array(img_pts,   dtype=np.float32)
    world_pts_arr = np.array(world_pts, dtype=np.float32)

    # ── Calcula homografia ────────────────────────────────────────────────────
    H, mask = cv2.findHomography(img_pts_arr, world_pts_arr, cv2.RANSAC, 3.0)

    if H is None or H.shape != (3, 3):
        print("[ERRO] findHomography falhou.")
        sys.exit(1)

    # ── Valida: reprojeção dos 4 pontos ──────────────────────────────────────
    reproj = cv2.perspectiveTransform(
        img_pts_arr.reshape(-1, 1, 2), H
    ).reshape(-1, 2)

    erros = np.linalg.norm(reproj - world_pts_arr, axis=1)
    print()
    print("[INFO] Erro de reprojeção por marcador (cm):")
    ids_validos = [k for k in WORLD_POSITIONS if len(acumulado[k]) >= args.min_detections]
    for mid, e in zip(ids_validos, erros):
        print(f"  ID {mid}: {e:.2f} cm")
    print(f"  RMSE total: {float(np.sqrt(np.mean(erros**2))):.2f} cm")

    # ── Salva ─────────────────────────────────────────────────────────────────
    with open(args.out, "wb") as f:
        pickle.dump({"H": H, "world_positions": WORLD_POSITIONS}, f)

    print(f"\n[INFO] Homografia salva em: {args.out}")
    print("[INFO] Matriz H:")
    print(H)
    print()
    print("─── Próximos passos ─────────────────────────────────────────────────")
    print(f"1. Copie {args.out} para o robô:")
    print(f"   scp {args.out} ubuntu@192.168.1.10:~/cuscobot_ws/src/cuscobot/projeto-robo/")
    print()
    print("2. No .env do robô, ative a homografia:")
    print("   HOMOGRAPHY_PKL=homografia_plano.pkl")
    print("   HOMOGRAPHY_FORCE=true")
    print()
    print("3. VIS_FLIP_X e VIS_FLIP_Y provavelmente não são mais necessários")
    print("   (a homografia já mapeia para o referencial correto).")
    print("─────────────────────────────────────────────────────────────────────")


if __name__ == "__main__":
    main()
