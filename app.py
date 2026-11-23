import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from flask import Flask, jsonify, render_template, request
from werkzeug.serving import BaseWSGIServer, WSGIRequestHandler

# Peticiones de sondeo del HMI: no escribir cada GET en la consola (ruido).
_QUIET_HTTP_LOG_PREFIXES = (
    "/api/motion/status",
    "/api/plc/status",
    "/api/prefeeder/status",
    "/api/state",
)


class _QuietRequestHandler(WSGIRequestHandler):
    def log_request(self, code="-", size="-"):
        path = (getattr(self, "path", "") or "").split("?", 1)[0]
        if code == 200 and any(path == p or path.startswith(p + "/") for p in _QUIET_HTTP_LOG_PREFIXES):
            return
        super().log_request(code, size)

_CLASS_DIR = Path(__file__).resolve().parent / "Class"
if str(_CLASS_DIR) not in sys.path:
    sys.path.insert(0, str(_CLASS_DIR))

import error_codes  # noqa: E402

app = Flask(__name__)

DATA_DIR = Path(__file__).resolve().parent / "data"
# Catálogo BOM (API): [{item, desc, longitud}, ...]. Fallback legado: modelos.json.
BOM_LONGITUDES_PATH = DATA_DIR / "bom_longitudes.json"
MODELOS_PATH = DATA_DIR / "modelos.json"
MOTION_PATH = DATA_DIR / "Motion.json"
PFEEDER_PATH = DATA_DIR / "PreFeeder.json"
CYCLE_PATH = DATA_DIR / "cycle.json"
PROD_CYCLE_LOG_PATH = DATA_DIR / "cycle_production.log"
LASER_STOP_LOG_PATH = DATA_DIR / "laser_stop.log"
SERIAL_PATH = DATA_DIR / "serial.json"
MANT_PATH = DATA_DIR / "mantenimiento.json"
CHECKLIST_LOG_PATH = DATA_DIR / "checklist.json"

# Vías de operador. No son parámetros del ciclo automático.
PURGE_PARK_MM = 250.0
PURGE_DEPOSIT_MM = 100.0
PURGE_CLEAN_MS = 3000
PURGE_AFTER_CUT_MS = 500
# La longitud del modelo es la pieza. La LM Guide recorre esa longitud menos este offset.
LMGUIDE_PIECE_OFFSET_MM = 55.0
CHECKLIST_MAX_CYCLES = 10000

# ServoLMGuide.h: fábrica 770 PUU / 45 mm; campo 550 mm pedido → 548 mm reales.
# El sentido de trabajo es PUU negativo.
LMGUIDE_STEPS_PER_MM = (770.0 / 45.0) * (550.0 / 548.0)
LMGUIDE_PUU_SIGN = -1
# Captura / guardado / HMI / corte: resolución 0.01 mm (compensa 3.25, 3.5, etc.).
# El PUU del drive es ~0.058 mm; el entero PUU es el límite físico real.
LMGUIDE_MM_RES = 0.01
LMGUIDE_PR_RPM_MIN = 400.0
LMGUIDE_PR_RPM_MAX = 3000.0
# ASDA P5.020/P5.021: ms para 0↔3000 rpm (Parameters.h PARAM_LM_*).
LMGUIDE_ACCEL_MS_DEFAULT = 200
LMGUIDE_DECEL_MS_DEFAULT = 500
LMGUIDE_RAMP_MS_MIN = 1
LMGUIDE_RAMP_MS_MAX = 65500

# Reset de error: condiciones PLC/Motion despejadas de forma estable (ms).
_ERROR_RESET_STABLE_MS = 200

# DSY 6091:01=1, 6091:02=10000 → 0x60FF = RPM * 10000 / 60 (manual PV).
FEED_SHAFT_INC_PER_REV = 10000.0

# -----------------------------------------------------------------------------
# ANDON_SKIP_OPCIONAL — TODO_BORRAR
# Puente de banco: permitir ciclo sin COM del Andon (env HMI_ANDON_SKIP=1 o UI
# "No usar Andon"). Cuando en planta siempre haya cable, borrar todo el bloque
# marcado con este tag en app.py, static/js/main.js y el párrafo en rutinas-operador.md.
# -----------------------------------------------------------------------------
_ANDON_SKIPPED_ENV = os.environ.get("HMI_ANDON_SKIP", "0").strip() == "1"
_ANDON_NONE_TOKEN = "__none__"
# --- fin constantes ANDON_SKIP_OPCIONAL ---

_serial_lock = threading.Lock()
_motion_write_lock = threading.Lock()
_motion_serial = None
_serial_cfg_lock = threading.Lock()
_serial_cfg = {
    "motion": "",
    "plc": "",
    "andon": "",
    "prefeeder": "",
    "andon_disabled": False,  # ANDON_SKIP_OPCIONAL — TODO_BORRAR (quitar clave y usos)
    "checklist_bypass": False,
    # Última prueba Probar por módulo: {port, ok, message}. No se borra al refrescar la lista USB.
    "online": {},
}

# Orquestador del ciclo automático (rules.md §4.1–4.8) — app.py coordina Motion + PLC.
_STATE_LOCK = threading.Lock()
# Candado de arranque: un solo Start/Resume/Charola a la vez (evita workers gemelos).
_CYCLE_SPAWN_LOCK = threading.Lock()
_CYCLE_WORKER_TLS = threading.local()
_CYCLE_CTRL = {
    "thread": None,
    # Generación del worker oficial. Un hilo huérfano ve gen ≠ la suya y se detiene.
    "gen": 0,
    "stop": threading.Event(),
    "pause_after_step": threading.Event(),
    # Prueba desde Parámetros → Ciclo: un paso, pausa, «Siguiente paso».
    "step_mode": False,
}

# Hilo de purga, posición inicial, alimentar un lado y piezas de prueba.
# El ciclo automático no usa este hilo.
_ROUTINE = {
    "thread": None,
    "stop": threading.Event(),
    "from_lote": False,
}

# Interlocks del orquestador (estado lógico tras pasos completados en este ciclo).
_CYCLE_LATCH = {
    "holder_encoder_active": False,
    "feed_validated": False,
    "gripper_active": False,
    "lm_at_cut": False,
    "cut_done": False,
    "discharge_done": False,
}
# Espejo de salidas PLC: lo actualiza la reply/push del esclavo, no un status pedido.
# known=False si se perdió el COM o aún no hay estado.
_PLC_MIRROR = {
    "known": False,
    "holder": False,
    "encoder": False,
    "gripper": False,
    "cutter_r": False,
    "cutter_l": False,
}
_PLC_MIRROR_LOCK = threading.Lock()
# Blower vive en Motion; se refleja aquí para la UI manual (panel PLC).
_MOTION_BLOWER_ON = False
# Arranque fresco: set_data ya viajó embebido en lmguide_move home.
_CYCLE_SET_DATA_ON_HOME = False
# Pinzas cerradas con pieza en proceso (paso 4 → antes de soltar en paso 7).
# Ante error se retiene; no se re-cierra si ya estaban abiertas.
_GRIPPER_HOLDING = False

# Latch operativo Joint (recovery.kind == "joint" es la fuente HMI).
_JOINT = {
    "skip_cut": "",  # "", "R", "L", "RL"
    "finish_piece": False,  # excepción fabricación: terminar pieza del lado bueno
    "piece_counted": False,  # pieza buena ya contada al entrar a operator
    "holder_released": False,
}

# Lote en curso. El batch (montón de la charola) se calcula al pulsar Start.
# qty_total = piezas del pedido; pieces_r/l = buenas por lado; cycle_sides = lados del ciclo actual
# (puede ser un solo lado para empatar tras fallo parcial).
_LOT = {
    "active": False,
    "tray_wait": False,
    "home_before": False,
    "log_id": 0,
    "cycles_total": 0,
    "cycles_done": 0,
    "qty_total": 0,
    "pieces_r": 0,
    "pieces_l": 0,
    "target_r": 0,
    "target_l": 0,
    "stacks": 0,
    "deposits_per_tray": 0,
    "batch_size": 50,
    "cut_mm": 0.0,
    "cut_offset_mm": 0.0,
    "cut_comp_mode": True,
    "cut_comp_r_mm": 0.0,
    "cut_comp_l_mm": 0.0,
    "length_mm": 0.0,
    "gap_mm": 0.0,
    "clearance_mm": 0.0,
    "sides": "R",
    "cycle_sides": "R",
}
_LAST_PIECE_S = 0.0
# Último Gripper OFF confirmado (perf_counter). Cadencia = liberación actual − anterior.
_PREV_PIECE_END_T: float | None = None
# Ancla perf_counter del Start (primera pieza: Start → primer Gripper OFF).
_CADENCE_T0: float | None = None
# Intervalo actual incluye pausa/error/Joint/charola/recuperación.
_CADENCE_INTERRUPTED = False
# Evita duplicar cadencia si el paso 7 se reentra o se reanuda tras liberar.
_CADENCE_RECORDED = False
# Reloj vivo de la pieza / lote (para HMI: estimado y corriendo). wall-clock.
_PIECE_TICK0: float | None = None
_LOT_T0: float | None = None
# Longitud programada de la pieza en curso (para duración del blower).
# Se conserva aunque el paso 3 del ciclo siguiente termine durante el regreso a home.
_PIECE_BLOWER_MM = None
# Lectura de encoder L/R de la pieza en curso (no se pisa con el solape).
# CAPTURA A: láser detuvo el servo (FeedStage). CAPTURA B: LM Guide reached.
_PIECE_FEED = {"R": None, "L": None}
_PIECE_ENC_AT_LM = {"R": None, "L": None}

# Siguiente ciclo arrancado al soltar pinzas (fin del paso 7 → paso 8).
# Los pasos 1 a 3 pueden terminar mientras el eje sigue hacia home.
# El paso 4 no cierra pinzas hasta que el paso 8 anterior acabó:
# eje en home y pinzas abiertas.
_OVERLAP = {
    "gen": 0,
    "on": False,
    "feed_sent": False,
    "feed_done": False,
    "holder_error": "",
    "holder_ready": threading.Event(),
    "feed": None,
    "blower_mm": None,
    "t0": None,
    "feed_error": False,
    "feed_error_name": "",
    "feed_task_started": False,
    "feed_t0": None,
    "sides": "",  # lados del ciclo solapado (catch-up puede diferir del actual)
}
# El siguiente paso 4 solo cierra: home ya está hecho y las pinzas siguen abiertas.
_STEP4_CLOSES = False

# MODO EXPERIMENTAL. En el paso 4 se pone ServoCAN en "Suelto"
# (feeder_servo_off / Duro apagado). El holder se abre; el encoder neumático
# permanece cerrado. Al depositar (paso 7), tras lmguide_move al montón, se
# dispara feeder_servo_on en paralelo: no bloquea Gripper OFF.
# Para probar: True. Para desactivar: False.
EXPERIMENTAL_SERVO_OFF = True

# Material ya alimentado hasta el láser (fuera de tiempo de ciclo).
# Listo para que el siguiente Start / charola arranque en el paso 4.
_FEED_PRIME = {
    "ready": False,
    "feed": None,  # {"R": ..., "L": ...}
    "sides": "",
    "blower_mm": None,
}

# Pasos ejecutables en orden (1 = arranque / set_data al Motion).
# Con otro ciclo pendiente, el paso 7 dispara de nuevo el 1 sin esperar al 8.
CYCLE_STEPS = (
    {"id": "1", "label": "Start — inicio de secuencia", "actor": "coordinator"},
    {"id": "2", "label": "Holder dual + encoder dual", "actor": "plc"},
    {"id": "3", "label": "Alimentación y medición L/R", "actor": "motion"},
    {"id": "4", "label": "Pinzas + antes de jalar", "actor": "plc"},
    {"id": "5", "label": "LM Guide — posición de corte", "actor": "motion"},
    {"id": "6", "label": "Corte R/L", "actor": "plc"},
    {"id": "7", "label": "Descarga de pieza", "actor": "motion+plc"},
    {"id": "8", "label": "ReleasePart — despeje, home y blower", "actor": "motion"},
)

# ── Registro de producción (una línea por operación; no sustituye [Motion]/[PLC]) ──
_LOT_SEQ = 0
_PROD_CADENCE_SAMPLES: list[float] = []
_PROD_LOG_LOCK = threading.Lock()
_LASER_STOP_LOG_LOCK = threading.Lock()
_PROD_CTX = {"piece": 1, "step_id": "1", "t0": 0.0}
_PROD_STEP_DESC = {
    "1": "Configuración inicial",
    "2": "Holder + encoder",
    "3": "Alimentación L/R",
    "4": "Pinzas + antes de jalar",
    "5": "LM Guide — corte",
    "6": "Corte R/L",
    "7": "Descarga de pieza",
    "8": "ReleasePart home + blower",
}


def _lot_cycle_sides_now() -> str:
    """Lados efectivos del ciclo actual (catch-up puede ser R o L solo)."""
    raw = str(_LOT.get("cycle_sides") or _LOT.get("sides") or "R").strip().upper()
    if raw in ("LR", "RL", "BOTH", "AMBOS"):
        return "LR"
    if raw == "L":
        return "L"
    return "R"


def _lot_credit_sides_now() -> tuple[str, ...]:
    """Lados que entregan pieza válida en el ciclo actual (omite skip_cut / Joint)."""
    sides = _lot_cycle_sides_now()
    skip = str(_JOINT.get("skip_cut") or "").strip().upper()
    out: list[str] = []
    if sides in ("R", "LR") and skip not in ("R", "RL"):
        out.append("R")
    if sides in ("L", "LR") and skip not in ("L", "RL"):
        out.append("L")
    return tuple(out)


def _lot_predict_counts_after_credit() -> tuple[int, int]:
    r = int(_LOT.get("pieces_r") or 0)
    l = int(_LOT.get("pieces_l") or 0)
    for side in _lot_credit_sides_now():
        if side == "R":
            r += 1
        elif side == "L":
            l += 1
    return r, l


def _lot_sides_for_counts(pieces_r: int, pieces_l: int) -> str:
    """Lados del siguiente ciclo para empatar y completar metas R/L."""
    base = str(_LOT.get("sides") or "R").strip().upper()
    if base in ("LR", "RL", "BOTH", "AMBOS"):
        base = "LR"
    elif base != "L":
        base = "R"
    if base != "LR":
        return base
    tr = int(_LOT.get("target_r") or 0)
    tl = int(_LOT.get("target_l") or 0)
    need_r = pieces_r < tr
    need_l = pieces_l < tl
    if not need_r and not need_l:
        return "LR"
    if pieces_r < pieces_l and need_r:
        return "R"
    if pieces_l < pieces_r and need_l:
        return "L"
    if need_r and need_l:
        return "LR"
    if need_r:
        return "R"
    if need_l:
        return "L"
    return "LR"


def _lot_next_cycle_sides() -> str:
    return _lot_sides_for_counts(
        int(_LOT.get("pieces_r") or 0),
        int(_LOT.get("pieces_l") or 0),
    )


def _lot_predict_next_cycle_sides() -> str:
    r, l = _lot_predict_counts_after_credit()
    return _lot_sides_for_counts(r, l)


def _lot_display_pieces(*, preview: bool = True) -> tuple[int, int]:
    r = int(_LOT.get("pieces_r") or 0)
    l = int(_LOT.get("pieces_l") or 0)
    if preview and _LOT.get("active"):
        for side in _lot_credit_sides_now():
            if side == "R":
                r += 1
            elif side == "L":
                l += 1
    return r, l


def _lot_credit_completed_piece() -> tuple[str, ...]:
    """Suma piezas buenas del ciclo que acaba de terminar. No toca cycles_done."""
    credited = _lot_credit_sides_now()
    kept: list[str] = []
    for side in credited:
        if side == "R":
            cap = int(_LOT.get("target_r") or 0)
            cur = int(_LOT.get("pieces_r") or 0)
            if cap <= 0 or cur >= cap:
                continue
            _LOT["pieces_r"] = cur + 1
            kept.append("R")
        elif side == "L":
            cap = int(_LOT.get("target_l") or 0)
            cur = int(_LOT.get("pieces_l") or 0)
            if cap <= 0 or cur >= cap:
                continue
            _LOT["pieces_l"] = cur + 1
            kept.append("L")
    for side in kept:
        _mant_add_side_piece(side)
    return tuple(kept)


def _lot_targets_met() -> bool:
    if not _LOT.get("active") and int(_LOT.get("qty_total") or 0) <= 0:
        return False
    return (
        int(_LOT.get("pieces_r") or 0) >= int(_LOT.get("target_r") or 0)
        and int(_LOT.get("pieces_l") or 0) >= int(_LOT.get("target_l") or 0)
        and (
            int(_LOT.get("target_r") or 0) > 0
            or int(_LOT.get("target_l") or 0) > 0
        )
    )


def _lot_would_need_another_after_credit() -> bool:
    r, l = _lot_predict_counts_after_credit()
    tr = int(_LOT.get("target_r") or 0)
    tl = int(_LOT.get("target_l") or 0)
    return r < tr or l < tl


def _lot_pieces_done_total() -> int:
    return int(_LOT.get("pieces_r") or 0) + int(_LOT.get("pieces_l") or 0)


def _lot_resume_pending() -> bool:
    """Pedido aún no terminado (aunque active haya quedado en False tras error/purga)."""
    if _LOT.get("active") or _LOT.get("tray_wait"):
        return True
    qty = int(_LOT.get("qty_total") or 0)
    tr = int(_LOT.get("target_r") or 0)
    tl = int(_LOT.get("target_l") or 0)
    if qty <= 0 or (tr <= 0 and tl <= 0):
        return False
    if _lot_targets_met():
        return False
    return True


def _lot_ensure_active_for_resume() -> bool:
    """Reactiva el lote sin tocar pieces_r/l ni cycles_done. True si el lote queda abierto."""
    if _LOT.get("tray_wait"):
        return True
    if _LOT.get("active"):
        return True
    if not _lot_resume_pending():
        return False
    _LOT["active"] = True
    return True


def _lot_init_piece_targets(qty: int, sides: str) -> None:
    """Metas R/L a partir de la cantidad total del pedido."""
    qty = max(0, int(qty))
    raw = str(sides or "R").strip().upper()
    if raw in ("LR", "RL", "BOTH", "AMBOS"):
        raw = "LR"
    elif raw != "L":
        raw = "R"
    _LOT["qty_total"] = qty
    _LOT["pieces_r"] = 0
    _LOT["pieces_l"] = 0
    if raw == "LR":
        half = qty // 2
        _LOT["target_r"] = half
        _LOT["target_l"] = half
    elif raw == "L":
        _LOT["target_r"] = 0
        _LOT["target_l"] = qty
    else:
        _LOT["target_r"] = qty
        _LOT["target_l"] = 0
    _LOT["sides"] = raw
    _LOT["cycle_sides"] = raw


def _prod_ts() -> str:
    """Fecha/hora corta con segundos y milisegundos."""
    now = datetime.now()
    return now.strftime("%d/%m/%y %H:%M:%S.") + f"{now.microsecond // 1000:03d}"


def _prod_qty_tag() -> str:
    """QTY = cantidad total de piezas del pedido."""
    if _LOT.get("active") or int(_LOT.get("qty_total") or 0) > 0:
        qty = int(_LOT.get("qty_total") or 0)
    else:
        qty = int(_MACHINE_STATE.get("cantidad") or 0)
    return f"QTY {max(qty, 0):03d}"


def _prod_rl_tag(*, preview: bool = True) -> str:
    """Contador R/L de piezas buenas (preview=True suma el ciclo en curso)."""
    r, l = _lot_display_pieces(preview=preview)
    return f"R{r:03d}/L{l:03d}"


def _prod_piece_tag(piece: int | None = None, total: int | None = None) -> str:
    """Compat: QTY + R/L (ya no PIEZA ciclos/total)."""
    return f"{_prod_qty_tag()} | {_prod_rl_tag(preview=True)}"


def _prod_actor(step_id: str) -> str:
    _, meta = _cycle_step_meta(step_id)
    if not meta:
        return "MOTION"
    actor = str(meta.get("actor") or "motion").lower()
    if actor == "coordinator":
        return "MOTION"
    if actor == "plc":
        return "PLC"
    if actor == "motion":
        return "MOTION"
    if "+" in actor or "motion" in actor and "plc" in actor:
        return "PLC+MOTION"
    return actor.upper()


def _prod_write(line: str) -> None:
    stamped = f"{_prod_ts()} | {line}"
    with _PROD_LOG_LOCK:
        PROD_CYCLE_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with PROD_CYCLE_LOG_PATH.open("a", encoding="utf-8") as f:
            f.write(stamped + "\n")
    print(f"[PROD] {stamped}", flush=True)
    _push_event(stamped, "prod")


def _laser_stop_log_write(msg: dict | str) -> None:
    """Traza de parada por láser: terminal + data/laser_stop.log."""
    if isinstance(msg, dict):
        try:
            raw = json.dumps(msg, ensure_ascii=False, separators=(",", ":"))
        except (TypeError, ValueError):
            raw = str(msg)
        side = str(msg.get("side") or "?")
        summary = (
            f"side={side} t_laser={msg.get('t_laser_ms')}ms "
            f"at={msg.get('counts_at_laser')} after_halt={msg.get('counts_after_halt')} "
            f"settled={msg.get('counts_settled')} overrun={msg.get('overrun_counts')} "
            f"settle={msg.get('settle_ms')}ms stop_ok={msg.get('stop_ok')} "
            f"status=0x{int(msg.get('status_word') or 0):04X} "
            f"vel={msg.get('vel_actual')}"
        )
    else:
        raw = str(msg)
        summary = raw
    stamped = f"{_prod_ts()} | {summary}"
    with _LASER_STOP_LOG_LOCK:
        LASER_STOP_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with LASER_STOP_LOG_PATH.open("a", encoding="utf-8") as f:
            f.write(stamped + " | " + raw + "\n")
    print(f"[LASER_STOP] {stamped}", flush=True)
    _push_event(f"[LASER_STOP] {stamped}", "prod")


def _prod_enc_counts_side(feed: dict | None, side: str) -> str:
    """Solo counts enteros del MCU. Sin inventar counts desde mm."""
    block = (feed or {}).get(side)
    if not isinstance(block, dict):
        return "-"
    if block.get("counts") is None:
        return "-"
    try:
        return str(int(block["counts"]))
    except (TypeError, ValueError):
        return "-"


def _prod_enc_tag(feed: dict | None = None) -> str:
    src = feed if feed is not None else _PIECE_FEED
    return (
        f"ENC R={_prod_enc_counts_side(src, 'R')} "
        f"L={_prod_enc_counts_side(src, 'L')}"
    )


def _prod_log_op(
    step_id: str,
    duration_s: float,
    *,
    piece: int | None = None,
    parallel: bool = False,
    status: str = "OK",
    detail: str = "",
    feed: dict | None = None,
) -> None:
    try:
        step_no = f"STEP {int(step_id):02d}"
    except (TypeError, ValueError):
        step_no = f"STEP {step_id or '?'}"
    desc = _PROD_STEP_DESC.get(str(step_id), str(step_id or "?"))
    actor = _prod_actor(step_id)
    flag = status if status not in ("OK",) else ("PARALLEL" if parallel else "OK")
    line = (
        f"{_prod_qty_tag()} | {_prod_rl_tag(preview=True)} | "
        f"{step_no} | {desc} | {actor} | {duration_s:.3f} s | {flag}"
    )
    if detail and flag in ("ERROR", "WARNING"):
        line += f" | {detail}"
    # Dos lecturas por pieza: láser-stop (STEP 03) y LM reached (STEP 05).
    if step_id == "3" and flag in ("OK", "PARALLEL"):
        line += f" | {_prod_enc_tag(feed)}"
    elif step_id == "5" and flag in ("OK", "PARALLEL"):
        line += f" | {_prod_enc_tag(feed if feed is not None else _PIECE_ENC_AT_LM)}"
    _prod_write(line)


def _prod_log_complete(duration_s: float, kind: str) -> None:
    """kind: FIRST | NORMAL | INTERRUPTED (cadencia en Gripper OFF)."""
    line = (
        f"{_prod_qty_tag()} | {_prod_rl_tag(preview=False)} | COMPLETE | "
        f"{duration_s:.3f} s | {kind} | {_prod_enc_tag()}"
    )
    _prod_write(line)


def _cadence_mark_interrupted() -> None:
    """Marca el intervalo de cadencia en curso como interrumpido (fuera del promedio)."""
    global _CADENCE_INTERRUPTED
    _CADENCE_INTERRUPTED = True


def _cadence_on_gripper_off() -> None:
    """Cadencia productiva: Gripper OFF confirmado (paso 7). No usa el paso 8."""
    global _LAST_PIECE_S, _PREV_PIECE_END_T, _PIECE_TICK0
    global _PROD_CADENCE_SAMPLES, _CADENCE_INTERRUPTED, _CADENCE_RECORDED
    if _CADENCE_RECORDED:
        return
    now = time.perf_counter()
    first = _PREV_PIECE_END_T is None
    if not first:
        dt = now - float(_PREV_PIECE_END_T)
    elif _CADENCE_T0 is not None:
        dt = now - float(_CADENCE_T0)
    else:
        dt = 0.0
    if dt < 0.0:
        dt = 0.0
    _LAST_PIECE_S = dt
    _CADENCE_RECORDED = True
    # Pieza descartada por Joint (sin lados válidos): no cuenta como producción/cadencia.
    discarded = bool(_joint_active()) and not _lot_credit_sides_now()
    interrupted = bool(_CADENCE_INTERRUPTED) or discarded
    if interrupted:
        kind = "INTERRUPTED"
    elif first:
        kind = "FIRST"
    else:
        kind = "NORMAL"
    if kind == "NORMAL" and dt > 0.0:
        _PROD_CADENCE_SAMPLES.append(dt)
    _PREV_PIECE_END_T = now
    _CADENCE_INTERRUPTED = False
    # Siguiente intervalo de estimación HMI parte de esta liberación.
    _PIECE_TICK0 = time.time()
    _set_machine_ui(tiempo_ciclo_s=round(_LAST_PIECE_S, 1))
    _prod_log_complete(_LAST_PIECE_S, kind)


def _prod_log_summary(done: int, total: int, avg_cadence: float) -> None:
    line = (
        f"SUMMARY | {_prod_qty_tag()} | {_prod_rl_tag(preview=False)} | "
        f"AVG CADENCE: {avg_cadence:.3f} s"
    )
    _prod_write(line)


def _prod_log_step_context() -> tuple[int, str, float, bool]:
    """Pieza / paso / duración / paralelo para una línea de producción."""
    if (
        _OVERLAP.get("on")
        and _OVERLAP.get("feed_sent")
        and not _OVERLAP.get("feed_done")
    ):
        piece = int(_LOT["cycles_done"]) + 2
        step_id = "3"
        t0 = float(_OVERLAP.get("feed_t0") or time.perf_counter())
        dt = max(0.0, time.perf_counter() - t0)
        return piece, step_id, dt, True
    ctx = _PROD_CTX
    ctx_step = str(ctx.get("step_id") or "").strip()
    if _cycle_thread_alive():
        piece = int(_LOT["cycles_done"]) + 1 if _LOT.get("active") else 1
        if not ctx_step:
            with _STATE_LOCK:
                ctx_step = str(_MACHINE_STATE.get("ciclo_paso_id") or "?")
        t0 = float(ctx.get("t0") or time.perf_counter())
        dt = max(0.0, time.perf_counter() - t0)
        return piece, ctx_step or "?", dt, bool(ctx.get("parallel"))
    step_id = ctx_step or "?"
    t0 = float(ctx.get("t0") or time.perf_counter())
    dt = max(0.0, time.perf_counter() - t0)
    piece = int(ctx.get("piece") or 1)
    return piece, step_id, dt, bool(ctx.get("parallel"))


def _prod_log_error(code: str, mensaje: str) -> None:
    """Registra error de producción; alimentación en paralelo usa pieza/paso del solape."""
    piece, step_id, dt, parallel = _prod_log_step_context()
    _prod_log_op(
        step_id,
        dt,
        piece=piece,
        parallel=parallel,
        status="ERROR",
        detail=mensaje or code,
    )


def _prod_log_joint(side: str, *, origin: str, code: str, mensaje: str) -> None:
    """Registra detección Joint en el log de producción (WARNING, no ERROR)."""
    piece, step_id, dt, parallel = _prod_log_step_context()
    label = _joint_side_label(side)
    detail = mensaje or code or f"Joint {label}"
    if origin:
        detail = f"{detail} ({origin})"
    _prod_log_op(
        step_id,
        dt,
        piece=piece,
        parallel=parallel,
        status="WARNING",
        detail=detail,
    )

_DEFAULT_CYCLE_PARAMS = {
    "ms_holder_enc_stable": 100,
    "ms_after_feed": 100,
    "ms_lm_at_cut": 100,
    "ms_lm_at_discharge": 100,
    "ms_cutter_on": 50,
    "ms_after_cutter": 100,
    "ms_after_grip_release": 100,
    "ms_blower_delay": 200,
    "ms_at_home_end": 50,
    "tolerance_mm": 2.0,
    "cut_offset_mm": 0.0,  # legado; ya no interviene en el plan de corte
    "cut_comp_mode": True,  # dual permanente: compensaciones R/L independientes
    "cut_comp_r_mm": 0.0,
    "cut_comp_l_mm": 0.0,
    "deposit_batch_size": 50,
    "deposit_stack_gap_mm": 100.0,
    "deposit_max_travel_mm": 1450.0,
    "gripper_clearance_mm": 20.0,
    "feed_sides": "R",
    "blower_ms_per_mm": 8.0,
    "blower_ms_min": 200,
}

# Persistidos en cycle.json junto al ciclo (no son tiempos del orquestador).
_HMI_CYCLE_EXTRA_KEYS = ("cantidad", "numero_modelo", "longitud_modelo_mm", "modelo_desc")

# Estado inicial (orquestador + UI)
_MACHINE_STATE = {
    "estado": "idle",
    "estado_label": "En espera",
    "mensaje": "Sistema listo. Sin errores activos.",
    "progreso_pct": 0,
    "ciclo_paso_id": "",
    "ciclo_paso_label": "",
    "ciclo_paso_indice": 0,
    "ciclo_paralelo_id": "",
    "ciclo_en_curso": False,
    "ciclo_error": "",
    "ciclo_feed": {"R": None, "L": None},
    "numero_modelo": "",
    "modelo_desc": "",
    "cantidad": 1,
    "longitud_modelo_mm": None,
    "compensacion_medicion_mm": 0.0,  # legado; ya no se edita ni interviene en el corte
    "tiempo_ciclo_s": None,
    "tiempo_estimado_s": 0,
    "tiempo_corriendo_s": 0,
    "eventos": [],
    "rutina": "",
    "rutina_fase": "",
    "rutina_manos": False,
    # Tras cualquier error: Reset → purga → Reanudar reintenta la pieza desde el paso 1
    # (el lote y cycles_done se conservan; no se cuenta ni se descarta la pieza).
    "post_error_purga": False,
    # Flujo guiado de recuperación (popup): error → reset → piezas → manguera → purga → listo.
    # kind="joint": WARNING Joint (sin Reset/piezas/manguera).
    "recovery": {
        "active": False,
        "step": "",
        "code": "",
        "mensaje": "",
        "kind": "",
        "side": "",
        "origin": "",
        "busy": False,
    },
    "ciclos_mant": 0,
    # Piezas buenas por lado desde el último reset de mantenimiento (no son el lote).
    "ciclos_mant_r": 0,
    "ciclos_mant_l": 0,
    "checklist": {
        "ok": os.environ.get("HMI_CHECKLIST_SKIP", "0").strip() == "1",
        "paso": 1,
        "empleado": "",
        "aire_ok": False,
        "eje_ok": False,
        "recorrido_ok": False,
        "alimentador_ok": False,
        "purga_ok": False,
        "piezas_cortadas": False,
    },
    # Estado de la franja global «Inicializar actuadores».
    "actuators": {
        "state": "offline",  # offline | initializing | online
        "stage": "",
    },
}

_HW_INIT_STAGE_LABELS = {
    "feeder_init": "feeder",
    "lmguide_init": "lmguide",
    "lmguide_home": "home",
}


def _actuators_set(state: str, stage: str = "") -> None:
    """Actualiza Offline / etapa (feeder|lmguide|home) / Online para la franja Motion."""
    st = str(state or "offline").strip().lower()
    if st not in ("offline", "initializing", "online"):
        st = "offline"
    stage_txt = str(stage or "").strip() if st == "initializing" else ""
    with _STATE_LOCK:
        act = _MACHINE_STATE.setdefault(
            "actuators", {"state": "offline", "stage": ""}
        )
        act["state"] = st
        act["stage"] = stage_txt


def _actuators_sync_from_motion_status(status: dict | None) -> bool:
    """Si Motion ya reporta feeders+LM listos, marca actuadores online (sesión nueva)."""
    if not isinstance(status, dict):
        return False
    if status.get("feeder_r_ready") is not True or status.get("feeder_l_ready") is not True:
        return False
    if status.get("lm_homed") is not True:
        return False
    _actuators_set("online")
    return True


def _hw_init_on_line(msg: dict) -> None:
    """Etapas intermedias de motion_hw_init (sin cerrar la espera de la respuesta final)."""
    if not isinstance(msg, dict):
        return
    op = str(msg.get("op") or "")
    if op == "hw_init_stage":
        key = str(msg.get("stage") or "").strip()
        label = _HW_INIT_STAGE_LABELS.get(key) or key.replace("_", " ")
        if label:
            _actuators_set("initializing", label)
        return
    if op == "feeder_init" and msg.get("ok") is True:
        _actuators_set("initializing", "lmguide")

def _dig(src, path, default):
    cur = src
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _as_float(src, path, default):
    try:
        return float(_dig(src, path, default))
    except (TypeError, ValueError):
        return float(default)


def _as_int(src, path, default):
    try:
        return int(round(float(_dig(src, path, default))))
    except (TypeError, ValueError):
        return int(default)


def _clamp_lm_pr_speed_rpm(value) -> float:
    try:
        x = float(value)
    except (TypeError, ValueError):
        x = 1200.0
    return max(LMGUIDE_PR_RPM_MIN, min(LMGUIDE_PR_RPM_MAX, x))


def _clamp_lm_ramp_ms(value, default: int) -> int:
    try:
        x = int(round(float(value)))
    except (TypeError, ValueError):
        x = int(default)
    return max(LMGUIDE_RAMP_MS_MIN, min(LMGUIDE_RAMP_MS_MAX, x))


def _as_bool(src, path, default=False) -> bool:
    value = _dig(src, path, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "on", "yes")
    return bool(value)


def _exp_flag(value, default=True) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "on", "yes")
    return bool(value)


def _exp_rpm(value, default=30) -> int:
    try:
        raw = default if value is None or value == "" else value
        return max(1, min(3000, int(round(float(raw)))))
    except (TypeError, ValueError):
        return max(1, min(3000, int(default)))


def _exp_ms(value, default=200) -> int:
    try:
        raw = default if value is None or value == "" else value
        return max(50, min(30000, int(raw)))
    except (TypeError, ValueError):
        return max(50, min(30000, int(default)))


def _canonical_experimental_side(src, side, legacy_rpm=30, legacy_ms=200):
    """Un lado del bloque experimental (antes de jalar / antes de cortar / corrección)."""
    side_src = src.get(side) if isinstance(src, dict) else None
    if not isinstance(side_src, dict):
        side_src = {}
    corr_src = side_src.get("correction")
    if not isinstance(corr_src, dict):
        corr_src = {}
    pull_src = side_src.get("before_pull")
    if not isinstance(pull_src, dict):
        pull_src = {}
    cut_src = side_src.get("before_cut")
    if not isinstance(cut_src, dict):
        cut_src = {}
    before_cut_holder = _exp_flag(cut_src.get("holder"), True)
    # Corrección solo tiene sentido con holder cerrado antes de cortar.
    correction_enabled = _exp_flag(corr_src.get("enabled"), False) and before_cut_holder
    return {
        "before_pull": {
            "holder": _exp_flag(pull_src.get("holder"), True),
            "encoder": _exp_flag(pull_src.get("encoder"), True),
        },
        "before_cut": {
            "holder": before_cut_holder,
            "encoder": _exp_flag(cut_src.get("encoder"), True),
        },
        "correction": {
            "enabled": correction_enabled,
            "rpm": _exp_rpm(corr_src.get("rpm"), legacy_rpm),
            "ms": _exp_ms(corr_src.get("ms"), legacy_ms),
        },
    }


def _canonical_experimental(src):
    """Normaliza experimental (migra reverse_rpm/ms legacy a corrección por lado)."""
    src = src if isinstance(src, dict) else {}
    legacy_rpm = max(1, min(3000, _as_int(src, ("reverse_rpm",), 30)))
    legacy_ms = max(50, min(30000, _as_int(src, ("reverse_ms",), 200)))
    return {
        "R": _canonical_experimental_side(src, "R", legacy_rpm, legacy_ms),
        "L": _canonical_experimental_side(src, "L", legacy_rpm, legacy_ms),
    }


def _exp_correction(tune, side: str) -> dict:
    """Lee corrección experimental del lado (R/L). Fallback a R si el lado no existe."""
    exp = (tune or {}).get("experimental") or {}
    side_u = str(side or "R").strip().upper()
    if side_u not in ("R", "L"):
        side_u = "R"
    side_cfg = exp.get(side_u) if isinstance(exp.get(side_u), dict) else None
    if side_cfg is None:
        side_cfg = exp.get("R") if isinstance(exp.get("R"), dict) else {}
    corr = side_cfg.get("correction") if isinstance(side_cfg, dict) else None
    if not isinstance(corr, dict):
        corr = {}
    # Compat: JSON antiguo con reverse_rpm / reverse_ms en la raíz.
    return {
        "enabled": _exp_flag(corr.get("enabled"), False),
        "rpm": _exp_rpm(corr.get("rpm"), exp.get("reverse_rpm", 30)),
        "ms": _exp_ms(corr.get("ms"), exp.get("reverse_ms", 200)),
    }


def _exp_active_sides() -> tuple[str, ...]:
    sides = str(_run_sides() or "LR").strip().upper()
    out = []
    if "R" in sides:
        out.append("R")
    if "L" in sides:
        out.append("L")
    return tuple(out) if out else ("R",)


def _exp_stage_valves(tune, stage: str) -> tuple[bool, bool]:
    """Holder/encoder dual de una etapa experimental (OR entre lados activos).

    stage: 'before_pull' | 'before_cut'
    """
    holder = False
    encoder = False
    exp = (tune or {}).get("experimental") or {}
    key = str(stage or "").strip()
    for side in _exp_active_sides():
        side_cfg = exp.get(side) if isinstance(exp.get(side), dict) else {}
        block = side_cfg.get(key) if isinstance(side_cfg, dict) else {}
        if not isinstance(block, dict):
            block = {}
        if _exp_flag(block.get("holder"), True):
            holder = True
        if _exp_flag(block.get("encoder"), True):
            encoder = True
    return holder, encoder


def _exp_before_pull_valves(tune) -> tuple[bool, bool]:
    return _exp_stage_valves(tune, "before_pull")


def _exp_before_cut_valves(tune) -> tuple[bool, bool]:
    return _exp_stage_valves(tune, "before_cut")


def _exp_correction_jobs(tune) -> list[dict]:
    """Lados activos con corrección usable (enabled + holder cerrado antes de cortar)."""
    jobs = []
    exp = (tune or {}).get("experimental") or {}
    for side in _exp_active_sides():
        side_cfg = exp.get(side) if isinstance(exp.get(side), dict) else {}
        cut = side_cfg.get("before_cut") if isinstance(side_cfg, dict) else {}
        if not isinstance(cut, dict):
            cut = {}
        if not _exp_flag(cut.get("holder"), True):
            continue
        corr = _exp_correction(tune, side)
        if not corr["enabled"]:
            continue
        jobs.append({"side": side, "rpm": corr["rpm"], "ms": corr["ms"]})
    return jobs


def _canonical_motion_tune(src):
    """Parámetros persistidos en Motion.json. Defaults alineados con Parameters.h."""
    src = src if isinstance(src, dict) else {}
    return {
        "feeder": {
            "R": {
                "profile_velocity_pps": _as_int(src, ("feeder", "R", "profile_velocity_pps"), 42500),
                "target_position_counts": _as_int(src, ("feeder", "R", "target_position_counts"), 42500),
            },
            "L": {
                "profile_velocity_pps": _as_int(src, ("feeder", "L", "profile_velocity_pps"), 42500),
                "target_position_counts": _as_int(src, ("feeder", "L", "target_position_counts"), 42500),
            },
        },
        "lmguide": {
            "pr_speed_rpm": _clamp_lm_pr_speed_rpm(_as_float(src, ("lmguide", "pr_speed_rpm"), 1200.0)),
            "accel_ms": _clamp_lm_ramp_ms(
                _dig(src, ("lmguide", "accel_ms"), LMGUIDE_ACCEL_MS_DEFAULT),
                LMGUIDE_ACCEL_MS_DEFAULT,
            ),
            "decel_ms": _clamp_lm_ramp_ms(
                _dig(src, ("lmguide", "decel_ms"), LMGUIDE_DECEL_MS_DEFAULT),
                LMGUIDE_DECEL_MS_DEFAULT,
            ),
            "pos_puu": _as_int(src, ("lmguide", "pos_puu"), 0),
        },
        "feed_stage": {
            # Apagado por defecto: sin rangos mm definidos en HMI no forzar E035/E036.
            "validate_encoder": _as_bool(src, ("feed_stage", "validate_encoder"), False),
        },
        # Experimental: holder/encoder por etapa + corrección (paso 2 y 6).
        "experimental": _canonical_experimental(src.get("experimental")),
    }


def _save_motion_tune(tune):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    MOTION_PATH.write_text(
        json.dumps(tune, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _load_motion_tune():
    if not MOTION_PATH.exists():
        tune = _canonical_motion_tune({})
        _save_motion_tune(tune)
        return tune
    # utf-8-sig: tolera BOM de editores Windows (Notepad, etc.)
    with MOTION_PATH.open(encoding="utf-8-sig") as f:
        data = json.load(f)
    return _canonical_motion_tune(data)


def _canonical_prefeeder_side(src: dict, side: str) -> dict:
    src = src if isinstance(src, dict) else {}
    defaults = {
        "L": {
            "rpm": 60.0,
            "puller_rpm": 60.0,
            "settle_ms": 150,
            "on_s": 2.0,
        },
        "R": {
            "rpm": 60.0,
            "puller_rpm": 60.0,
            "settle_ms": 150,
            "on_s": 2.0,
        },
    }[side]
    block = src.get(side) if isinstance(src.get(side), dict) else {}
    return {
        "rpm": _as_float(block, ("rpm",), defaults["rpm"]),
        "puller_rpm": _as_float(block, ("puller_rpm",), defaults["puller_rpm"]),
        "settle_ms": _as_int(block, ("settle_ms",), defaults["settle_ms"]),
        "on_s": _as_float(block, ("on_s",), defaults["on_s"]),
    }


def _canonical_prefeeder_tune(src):
    src = src if isinstance(src, dict) else {}
    return {
        "L": _canonical_prefeeder_side(src, "L"),
        "R": _canonical_prefeeder_side(src, "R"),
    }


def _save_prefeeder_tune(tune: dict) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PFEEDER_PATH.write_text(
        json.dumps(tune, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _load_prefeeder_tune():
    if not PFEEDER_PATH.exists():
        tune = _canonical_prefeeder_tune({})
        _save_prefeeder_tune(tune)
        return tune
    # utf-8-sig: tolera BOM de editores Windows (Notepad, etc.)
    with PFEEDER_PATH.open(encoding="utf-8-sig") as f:
        data = json.load(f)
    return _canonical_prefeeder_tune(data)


def _prefeeder_cfg_signature(side_cfg: dict) -> tuple:
    c = side_cfg
    return (
        round(c["rpm"], 2),
        round(c["puller_rpm"], 2),
        c["settle_ms"],
        round(c["on_s"], 3),
    )


# Datos de ciclo en ESP32 (NVS); la app los empuja con set_data / modelo.
_MOTION_DATA = {
    "target_length_mm": None,
    "tolerance_mm": 2.0,
    "modelo_id": "",
    "measure_comp_mm": 0.0,
}


def _lm_cmd_fields(tune: dict | None = None) -> dict:
    """Campos LM Guide que van en cada orden de movimiento / set_tune."""
    lm = _canonical_motion_tune(tune or {})["lmguide"]
    return {
        "lm_pr_speed_rpm": lm["pr_speed_rpm"],
        "lm_accel_ms": lm["accel_ms"],
        "lm_decel_ms": lm["decel_ms"],
    }


def _motion_tune_flat(tune: dict) -> dict:
    tune = _canonical_motion_tune(tune)
    feed = tune["feeder"]
    lm = tune["lmguide"]
    flat = {
        "op": "set_tune",
        "feed_r_vel_pps": feed["R"]["profile_velocity_pps"],
        "feed_r_target_counts": feed["R"]["target_position_counts"],
        "feed_l_vel_pps": feed["L"]["profile_velocity_pps"],
        "feed_l_target_counts": feed["L"]["target_position_counts"],
        "lm_pos_puu": lm["pos_puu"],
        "feed_validate_encoder": tune["feed_stage"]["validate_encoder"],
    }
    flat.update(_lm_cmd_fields(tune))
    return flat


def _compact(obj) -> str:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False)


def _snap_lm_mm(mm) -> float:
    """Redondea mm a la resolución del LM Guide (0.01 mm)."""
    try:
        x = float(mm)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(x):
        return 0.0
    return round(x / LMGUIDE_MM_RES) * LMGUIDE_MM_RES


def _puu_to_mm(puu) -> float:
    """PUU → mm absolutos a 0.01 mm (misma escala que la HMI)."""
    try:
        counts = abs(int(puu))
    except (TypeError, ValueError):
        return 0.0
    return _snap_lm_mm(counts / LMGUIDE_STEPS_PER_MM)


def _mm_to_puu(mm) -> int:
    snapped = _snap_lm_mm(mm)
    return int(round(LMGUIDE_PUU_SIGN * snapped * LMGUIDE_STEPS_PER_MM))


def _num(value, default=0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _feed_sides_of(params: dict) -> str:
    raw = str(params.get("feed_sides") or "R").strip().upper().replace(" ", "").replace("+", "")
    if raw in ("LR", "RL", "BOTH", "AMBOS"):
        return "LR"
    if raw == "L":
        return "L"
    return "R"


_PFEEDER_CODES_L = frozenset({"E093", "E094", "E095"})
_PFEEDER_CODES_R = frozenset({"E096", "E097", "E098"})


def _active_feed_sides(sides: str | None = None) -> tuple[str, ...]:
    """Lados que cuentan para Motion/PreFeeder según feed_sides."""
    raw = str(sides if sides is not None else _run_sides() or "R").strip().upper()
    if raw == "LR":
        return ("L", "R")
    if raw == "L":
        return ("L",)
    return ("R",)


def _feed_validate_encoder_enabled(tune: dict | None = None) -> bool:
    """True solo si el operador dejó activo «Validar longitud con encoder»."""
    src = tune if isinstance(tune, dict) else _load_motion_tune()
    return bool(_canonical_motion_tune(src)["feed_stage"]["validate_encoder"])


def _feed_stage_empty_error(feed: dict | None, sides: str | None = None) -> str:
    """Rechaza alimentación inválida del FeedStage.

    Láser ya ON al arrancar (counts 0, sin avance) es material listo: Motion no
    alimenta ese lado y responde ok. El host no lo convierte en E035/E036.

    Tolerancia: solo si validate_encoder está ON y Motion marcó enc_ok=false.
    No inventa umbrales mm; eso no está definido en HMI.
    """
    check_tol = _feed_validate_encoder_enabled()
    for side in _active_feed_sides(sides):
        block = (feed or {}).get(side)
        if not isinstance(block, dict):
            return f"encoder_tolerance_{side.lower()}"
        if check_tol and block.get("enc_ok") is False:
            return f"encoder_tolerance_{side.lower()}"
    return ""


def _parse_bool(value, default: bool = False) -> bool:
    """Convierte checkbox/JSON suelto a bool. No confundir con _as_bool(src, path, default)."""
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return value != 0
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on", "si", "sí"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return default


def _cut_base_mm(length: float) -> float:
    """Posición base de corte: |largo| − 55 mm (sin compensación)."""
    return abs(length) - LMGUIDE_PIECE_OFFSET_MM


def _cut_comp_side_mm(params: dict, side: str) -> float:
    if side == "R":
        return _num(params.get("cut_comp_r_mm"), 0.0)
    return _num(params.get("cut_comp_l_mm"), 0.0)


def _cut_comp_params_of(params: dict | None = None) -> dict:
    """Compensaciones de corte: modo dual permanente (R/L independientes)."""
    src = params if isinstance(params, dict) else _load_cycle_params()
    return {
        "cut_comp_mode": True,
        "cut_comp_r_mm": _num(src.get("cut_comp_r_mm"), 0.0),
        "cut_comp_l_mm": _num(src.get("cut_comp_l_mm"), 0.0),
        "cut_offset_mm": 0.0,
    }


def _build_cut_plan(length: float, params: dict | None, sides: str) -> dict:
    """
    Plan de corte LM Guide (compensación dual permanente).
    - Un lado: base + compensación de ese lado.
    - LR: siempre un corte por lado en su compensación (nunca omitir ni
      colapsar a un solo pulso dual). Orden: offset menor primero, mayor al
      final (despeje/depósito sobre last_mm).
    """
    base = _cut_base_mm(length)
    sides = str(sides or "R").strip().upper()
    if sides not in ("R", "L", "LR"):
        sides = "R"
    comps = _cut_comp_params_of(params)
    mode = True
    general = 0.0
    comp_r = float(comps["cut_comp_r_mm"])
    comp_l = float(comps["cut_comp_l_mm"])

    def side_name(side: str) -> str:
        return {"R": "derecho", "L": "izquierdo", "LR": "L y R"}[side]

    def valve_of(side: str) -> str:
        return {"R": "cutter_r", "L": "cutter_l", "LR": "cutters"}[side]

    def single(side: str, mm: float) -> dict:
        return {
            "mode": mode,
            "sequential": False,
            "base_mm": base,
            "general_mm": general,
            "last_mm": mm,
            "first_side": side,
            "first_mm": mm,
            "first_valve": valve_of(side),
            "first_label": side_name(side),
            "second_side": "",
            "second_mm": mm,
            "second_valve": "",
            "second_label": "",
            "comp_r_mm": comp_r,
            "comp_l_mm": comp_l,
        }

    if sides == "R":
        return single("R", base + comp_r)
    if sides == "L":
        return single("L", base + comp_l)

    # Ambos lados: compensación de cada lado siempre, aunque sean iguales.
    first_side, second_side = ("R", "L") if comp_r <= comp_l else ("L", "R")
    first_comp = comp_r if first_side == "R" else comp_l
    second_comp = comp_r if second_side == "R" else comp_l
    first_mm = base + first_comp
    second_mm = base + second_comp
    return {
        "mode": mode,
        "sequential": True,
        "base_mm": base,
        "general_mm": 0.0,
        "last_mm": second_mm,
        "first_side": first_side,
        "first_mm": first_mm,
        "first_valve": valve_of(first_side),
        "first_label": side_name(first_side),
        "second_side": second_side,
        "second_mm": second_mm,
        "second_valve": valve_of(second_side),
        "second_label": side_name(second_side),
        "comp_r_mm": comp_r,
        "comp_l_mm": comp_l,
    }


def _cut_mm_of(length: float, params: dict | None = None) -> float:
    """Última posición de corte (referencia de depósito/despeje)."""
    src = params if isinstance(params, dict) else _load_cycle_params()
    plan = _build_cut_plan(length, src, _feed_sides_of(src))
    return float(plan["last_mm"])


def _active_cut_plan() -> dict | None:
    """Plan de corte del ciclo en curso. None si no hay longitud."""
    if _LOT["active"]:
        length = float(_LOT.get("length_mm") or 0.0)
        if length <= 0:
            return None
        params = {
            "cut_comp_r_mm": _LOT.get("cut_comp_r_mm"),
            "cut_comp_l_mm": _LOT.get("cut_comp_l_mm"),
        }
        return _build_cut_plan(length, params, _lot_cycle_sides_now())
    params = _load_cycle_params()
    length = _target_length_mm()
    if length <= 0:
        return None
    return _build_cut_plan(length, params, _feed_sides_of(params))


def _clear_of(deposit: float, clearance: float) -> float:
    clearance = abs(clearance)
    if deposit < 0:
        return deposit - clearance
    return deposit + clearance


def _stack_count(cut: float, length: float, gap: float, clearance: float, travel: float) -> int:
    """Montones que caben. 0 = sin tope de carrera."""
    if travel <= 0:
        return 0
    step = abs(length) + gap
    deposit1 = cut + gap
    if gap >= 0 and deposit1 >= 0 and step > 0:
        room = travel - abs(clearance) - deposit1
        if room < 0:
            return 0
        return 1 + int(room // step)
    if step == 0:
        return 1
    n = 0
    while n < 100000:
        deposit = cut + gap + n * step
        if abs(_clear_of(deposit, clearance)) > travel:
            break
        n += 1
    return n


def _tray_plan(length: float, params: dict):
    """Plan de charola para esta longitud. None si el corte o el primer montón no caben."""
    batch = int(_num(params.get("deposit_batch_size"), 50))
    if batch < 1:
        return None, "El tamaño del batch tiene que ser al menos 1."
    gap = _num(params.get("deposit_stack_gap_mm"), 100.0)
    clearance = abs(_num(params.get("gripper_clearance_mm"), 20.0))
    travel = _num(params.get("deposit_max_travel_mm"), 0.0)
    cut = _cut_mm_of(length, params)
    deposit1 = cut + gap
    clear1 = _clear_of(deposit1, clearance)
    if travel > 0:
        passed = []
        if abs(cut) > travel:
            passed.append(f"corte en {cut:.1f} mm")
        if abs(clear1) > travel:
            passed.append(f"primer montón con despeje en {clear1:.1f} mm")
        if passed:
            return None, (
                f"No cabe en la carrera de {travel:.1f} mm: " + " y ".join(passed) + "."
            )
    stacks = _stack_count(cut, length, gap, clearance, travel)
    if travel > 0 and stacks < 1:
        return None, (
            f"No cabe en la carrera de {travel:.1f} mm: "
            f"primer montón con despeje en {clear1:.1f} mm."
        )
    return {
        "batch_size": batch,
        "gap_mm": gap,
        "clearance_mm": clearance,
        "cut_mm": cut,
        "length_mm": abs(length),
        "stacks": stacks,
        "deposits": (stacks * batch) if stacks else 0,
    }, ""


def _order_cycles(qty, sides: str):
    try:
        n = int(qty)
    except (TypeError, ValueError):
        n = 0
    if sides == "LR":
        if n < 2 or n % 2:
            return 0, "Con ambos lados la cantidad tiene que ser par y de al menos 2 piezas."
        return n // 2, ""
    if n < 1:
        return 0, "La cantidad es el número de ciclos y tiene que ser al menos 1."
    return n, ""


def _deposit_at(cut: float, length: float, gap: float, batch: int, stacks: int, cycle: int):
    idx0 = (max(cycle, 1) - 1) // max(batch, 1)
    if stacks:
        idx0 %= stacks
    deposit = cut + gap + idx0 * (abs(length) + gap)
    return deposit, idx0 + 1


def _active_cut_deposit():
    """Corte y depósito del ciclo que va a cortarse. None si no hay longitud."""
    if _LOT["active"]:
        cycle = _LOT["cycles_done"] + 1
        deposit, stack = _deposit_at(
            _LOT["cut_mm"],
            _LOT["length_mm"],
            _LOT["gap_mm"],
            _LOT["batch_size"],
            _LOT["stacks"],
            cycle,
        )
        return _LOT["cut_mm"], deposit, stack
    params = _load_cycle_params()
    length = _target_length_mm()
    if length <= 0:
        return None
    cut = _cut_mm_of(length, params)
    gap = _num(params.get("deposit_stack_gap_mm"), 0.0)
    batch = int(_num(params.get("deposit_batch_size"), 50))
    deposit, stack = _deposit_at(cut, length, gap, max(batch, 1), 0, 1)
    return cut, deposit, stack


def _lm_cut_and_discharge(params: dict) -> tuple[float, float] | None:
    """Corte = |L| − 55 mm + offset. Descarga = depósito del primer montón."""
    length = _target_length_mm()
    if length <= 0:
        return None
    cut = _cut_mm_of(length, params)
    gap = _num(params.get("deposit_stack_gap_mm"), 0.0)
    return cut, cut + gap


def _run_sides() -> str:
    if _MACHINE_STATE.get("rutina") == "checklist":
        return "LR"
    if _LOT["active"]:
        return _lot_cycle_sides_now()
    return _feed_sides_of(_load_cycle_params())


def _cycle_piece_prefix() -> str:
    if not _LOT["active"]:
        return ""
    r, l = _lot_display_pieces(preview=True)
    qty = int(_LOT.get("qty_total") or 0)
    sides = _lot_cycle_sides_now()
    return f"QTY {qty:03d} | R{r:03d}/L{l:03d} ({sides}). "


def _announce_trays(cycles: int, plan: dict) -> None:
    deposits = int(plan["deposits"] or 0)
    if not deposits or cycles <= deposits:
        return
    trays = math.ceil(cycles / deposits)
    _push_event(
        f"Caben {plan['stacks']} montones, {deposits} depósitos por charola. "
        f"El lote pide {trays} charolas y pausa cada {deposits} ciclos."
    )


def _motion_scale() -> dict:
    return {
        "steps_per_mm": LMGUIDE_STEPS_PER_MM,
        "puu_sign": LMGUIDE_PUU_SIGN,
        "mm_res": LMGUIDE_MM_RES,
        "feed_shaft_inc_per_rev": FEED_SHAFT_INC_PER_REV,
    }


def _build_motion_lines(op: str, tune: dict, body: dict):
    """Líneas Serial compactas. El firmware busca \"op\":\"...\" sin espacios."""
    tune = _canonical_motion_tune(tune)
    tune_line = _compact(_motion_tune_flat(tune))

    if op == "set_tune":
        return tune, [tune_line]
    if op == "encoder_reset":
        side = str(body.get("side") or "").strip().upper()
        if side in ("R", "L"):
            return tune, [_compact({"op": "encoder_reset", "side": side})]
        return tune, [_compact({"op": "encoder_reset"})]
    if op == "feeder_init":
        return tune, [tune_line, '{"op":"feeder_init","side":"both"}']
    if op == "motion_hw_init":
        return tune, [tune_line, '{"op":"motion_hw_init"}']
    if op == "lmguide_init":
        return tune, ['{"op":"lmguide_servo_on"}']
    if op == "lmguide_servo":
        if body.get("on"):
            return tune, ['{"op":"lmguide_servo_on"}']
        return tune, ['{"op":"lmguide_servo_off"}']
    if op == "feeder_servo":
        side = str(body.get("side") or "").strip().upper()
        payload = {"op": "feeder_servo_on" if body.get("on") else "feeder_servo_off"}
        if side in ("R", "L"):
            payload["side"] = side
        return tune, [_compact(payload)]
    if op == "lmguide_home":
        return tune, ['{"op":"lmguide_home"}']
    if op == "lmguide_move_home":
        payload = {"op": "lmguide_move", "target": "home"}
        payload.update(_lm_cmd_fields(tune))
        return tune, [_compact(payload)]
    if op == "lmguide_move":
        if body.get("mm") is None or body.get("mm") == "":
            raise ValueError("Falta la posición en mm.")
        puu = _mm_to_puu(body.get("mm"))
        tune["lmguide"]["pos_puu"] = puu
        payload = {"op": "lmguide_move", "puu": puu}
        payload.update(_lm_cmd_fields(tune))
        return tune, [_compact(payload)]
    if op == "lmguide_jog":
        if body.get("jog_mm") is None or body.get("jog_mm") == "":
            raise ValueError("Falta el jog en mm.")
        jog_mm = float(body.get("jog_mm"))
        if jog_mm == 0:
            raise ValueError("El jog en mm tiene que ser distinto de cero.")
        direction = 1 if int(body.get("dir") or 1) >= 0 else -1
        delta = _mm_to_puu(direction * abs(jog_mm))
        payload = {"op": "lmguide_jog", "delta_puu": delta}
        payload.update(_lm_cmd_fields(tune))
        return tune, [_compact(payload)]
    if op == "lmguide_stop":
        return tune, ['{"op":"lmguide_stop"}']
    if op == "blower_on":
        raw = body.get("ms")
        ms = 2000 if raw is None or raw == "" else int(raw)
        if ms < 200:
            ms = 200
        if ms > 300000:
            ms = 300000
        return tune, [_compact({"op": "blower_on", "ms": ms})]
    if op == "blower_off":
        return tune, ['{"op":"blower_off"}']
    if op == "feeder_reverse":
        side = str(body.get("side") or "both").strip().upper()
        if side not in ("R", "L", "BOTH"):
            raise ValueError("feeder_reverse pide side R, L o both.")
        corr_side = "R" if side == "BOTH" else side
        corr = _exp_correction(tune, corr_side)
        rpm_raw = body.get("rpm")
        if rpm_raw is None or rpm_raw == "":
            rpm_raw = corr["rpm"]
        ms_raw = body.get("ms")
        if ms_raw is None or ms_raw == "":
            ms_raw = corr["ms"]
        rpm = _exp_rpm(rpm_raw, corr["rpm"])
        ms = _exp_ms(ms_raw, corr["ms"])
        exp = _canonical_experimental(tune.get("experimental"))
        if side in ("R", "L"):
            exp[side]["correction"]["rpm"] = rpm
            exp[side]["correction"]["ms"] = ms
        else:
            exp["R"]["correction"]["rpm"] = rpm
            exp["R"]["correction"]["ms"] = ms
            exp["L"]["correction"]["rpm"] = rpm
            exp["L"]["correction"]["ms"] = ms
        tune["experimental"] = exp
        payload = {
            "op": "feeder_reverse",
            "side": "both" if side == "BOTH" else side,
            "rpm": rpm,
            "ms": ms,
        }
        return tune, [_compact(payload)]
    if op == "release_part":
        payload = {"op": "release_part"}
        payload.update(_lm_cmd_fields(tune))
        if body.get("clear_puu") is not None:
            payload["clear_puu"] = int(body["clear_puu"])
        for key in ("blower_delay_ms", "blower_ms", "home_settle_ms"):
            if body.get(key) is not None:
                payload[key] = int(body[key])
        return tune, [_compact(payload)]
    raise ValueError("Orden Motion no reconocida.")


# Firmware: PARAM_LM_MOVE_DONE_TIMEOUT_MS (45s) × PARAM_LM_MOVE_MAX_ATTEMPTS (1).
_MOTION_LM_MOVE_TIMEOUT_S = 50.0
_MOTION_STATUS_TIMEOUT_S = 3.0


def _reply_wait_s(line: str) -> float:
    if "release_part" in line:
        return _MOTION_LM_MOVE_TIMEOUT_S * 2.0
    if "lmguide_home" in line or "lmguide_move" in line:
        return _MOTION_LM_MOVE_TIMEOUT_S
    if "motion_hw_init" in line:
        return 55.0
    if "feeder_init" in line or "lmguide_servo_on" in line:
        return 8.0
    if '"op":"feeder_reverse"' in line:
        ms = 200
        try:
            ms = int(json.loads(line).get("ms") or 200)
        except (TypeError, ValueError, json.JSONDecodeError):
            pass
        return max(2.0, (ms / 1000.0) + 2.0)
    if '"op":"status"' in line:
        return _MOTION_STATUS_TIMEOUT_S
    return 2.0


def _normalize_online_map(raw, motion: str, plc: str, andon: str, prefeeder: str) -> dict:
    """Conserva solo entradas online cuyo puerto sigue coincidiendo con la asignación."""
    src = raw if isinstance(raw, dict) else {}
    assigned = {
        "motion": motion,
        "plc": plc,
        "andon": andon,
        "prefeeder": prefeeder,
    }
    out = {}
    for which, port in assigned.items():
        item = src.get(which)
        if not isinstance(item, dict) or not port:
            continue
        saved_port = str(item.get("port") or "").strip()
        if not saved_port or _port_compare_key(saved_port) != _port_compare_key(port):
            continue
        out[which] = {
            "port": port,
            "ok": bool(item.get("ok")),
            "message": str(item.get("message") or "").strip(),
        }
    return out


def _load_serial_cfg() -> None:
    motion = os.environ.get("MOTION_SERIAL", "").strip()
    plc = os.environ.get("PLC_SERIAL", "").strip()
    andon = os.environ.get("ANDON_SERIAL", "").strip()
    prefeeder = os.environ.get("PREFEEDER_SERIAL", "").strip()
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR ---
    andon_disabled = False
    checklist_bypass = False
    online_raw = {}
    if SERIAL_PATH.exists():
        try:
            saved = json.loads(SERIAL_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            saved = None
        if isinstance(saved, dict):
            motion = str(saved.get("motion") or "").strip()
            plc = str(saved.get("plc") or "").strip()
            andon = str(saved.get("andon") or "").strip()
            prefeeder = str(saved.get("prefeeder") or "").strip()
            andon_disabled = bool(saved.get("andon_disabled"))
            checklist_bypass = bool(saved.get("checklist_bypass"))
            # No restaurar "online" de disco: un ok viejo no implica enlace vivo.
    if andon_disabled:
        andon = ""
    with _serial_cfg_lock:
        _serial_cfg["motion"] = motion
        _serial_cfg["plc"] = plc
        _serial_cfg["andon"] = andon
        _serial_cfg["prefeeder"] = prefeeder
        _serial_cfg["andon_disabled"] = andon_disabled
        _serial_cfg["checklist_bypass"] = checklist_bypass
        _serial_cfg["online"] = {}
    # --- fin ANDON_SKIP_OPCIONAL (_load_serial_cfg) ---


def _andon_skipped_effective() -> bool:
    """ANDON_SKIP_OPCIONAL — TODO_BORRAR: True si no se exige/enlaza el MCU Andon."""
    with _serial_cfg_lock:
        disabled = bool(_serial_cfg.get("andon_disabled"))
    return _ANDON_SKIPPED_ENV or disabled


def _checklist_bypassed() -> bool:
    with _serial_cfg_lock:
        return bool(_serial_cfg.get("checklist_bypass"))


def _serial_port(which: str) -> str:
    with _serial_cfg_lock:
        return _serial_cfg.get(which, "").strip()


def _close_motion_serial() -> None:
    global _motion_serial
    with _serial_lock:
        if _motion_serial is not None:
            try:
                _motion_serial.close()
            except Exception:
                pass
            _motion_serial = None


def _plc_mirror_invalidate() -> None:
    with _PLC_MIRROR_LOCK:
        _PLC_MIRROR["known"] = False


def _plc_mirror_apply(msg: dict | None) -> None:
    """Actualiza banderas desde status/valve reply del PLC."""
    if not isinstance(msg, dict):
        return
    keys = ("holder", "encoder", "gripper", "cutter_r", "cutter_l")
    if not any(k in msg for k in keys):
        return
    with _PLC_MIRROR_LOCK:
        for key in keys:
            if key in msg:
                _PLC_MIRROR[key] = bool(msg[key])
        _PLC_MIRROR["known"] = True


def _plc_mirror_check(expected: dict) -> tuple[bool, str | None]:
    """Valida salidas contra el espejo local (sin GET status)."""
    if not _plc_serial_configured():
        return False, "E017"
    with _PLC_MIRROR_LOCK:
        if not _PLC_MIRROR["known"]:
            return False, "E010"
        snap = {key: _PLC_MIRROR.get(key) for key in expected}
    return _plc_verify_logical(snap, expected)


def _plc_rx_seed_mirror(ser) -> None:
    """Tras abrir COM: consume boot/status push y alimenta el espejo."""
    t_end = time.time() + 0.2
    while time.time() < t_end:
        try:
            n = getattr(ser, "in_waiting", 0) or 0
        except Exception:
            break
        if not n:
            time.sleep(0.02)
            continue
        line = _read_serial_line(ser, 0.08)
        if not line:
            break
        try:
            msg = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") in ("status", "valve"):
            _plc_mirror_apply(msg)


def _close_plc_serial() -> None:
    global _plc_serial
    with _plc_serial_lock:
        if _plc_serial is not None:
            try:
                _plc_serial.close()
            except Exception:
                pass
            _plc_serial = None
    _plc_mirror_invalidate()


def _close_andon_serial() -> None:
    global _andon_serial, _andon_state_sent, _andon_attempted
    with _andon_serial_lock:
        if _andon_serial is not None:
            try:
                _andon_serial.close()
            except Exception:
                pass
            _andon_serial = None
        _andon_state_sent = ""
        _andon_attempted = ""


def _close_prefeeder_serial() -> None:
    global _prefeeder_serial, _PF_NEED_REOPEN
    with _prefeeder_serial_lock:
        if _prefeeder_serial is not None:
            try:
                _prefeeder_serial.close()
            except Exception:
                pass
            _prefeeder_serial = None
        _PF_NEED_REOPEN = True


def _invalidate_serial_handle(ser) -> None:
    """Cierra un handle que Windows dejó zombie tras Monitor Serial / desconexión USB."""
    if ser is None:
        return
    try:
        ser.close()
    except Exception:
        pass


def _port_compare_key(device: str) -> str:
    """Clave de comparación case-insensitive para COMx."""
    return str(device or "").strip().upper()


def _is_windows_com_name(device: str) -> bool:
    text = str(device or "").strip().upper()
    return text.startswith("COM") and text[3:].isdigit()


def _list_com_ports() -> list[dict]:
    """Lista puertos COM disponibles (Windows)."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    found = []
    seen = set()
    for port in list_ports.comports():
        device = str(port.device or "").strip()
        if not device or not _is_windows_com_name(device):
            continue
        key = _port_compare_key(device)
        if key in seen:
            continue
        seen.add(key)
        found.append({"device": device, "description": str(port.description or "").strip()})
    found.sort(key=lambda item: item["device"])
    return found


def _match_listed_port(value: str, ports: list[dict]) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    want = _port_compare_key(text)
    for port in ports:
        if _port_compare_key(port["device"]) == want:
            return port["device"]
    return ""


def _online_snapshot(listed: list[dict] | None = None) -> dict:
    """Estado online en memoria. Si se pasa la lista COM, quita ok de puertos ausentes."""
    present = None
    if listed is not None:
        present = {_port_compare_key(p["device"]) for p in listed if p.get("device")}
    with _serial_cfg_lock:
        raw = dict(_serial_cfg.get("online") or {})
        if present is not None:
            changed = False
            for which, item in list(raw.items()):
                if not isinstance(item, dict):
                    raw.pop(which, None)
                    changed = True
                    continue
                port_key = _port_compare_key(str(item.get("port") or ""))
                if not port_key or port_key not in present:
                    raw.pop(which, None)
                    changed = True
            if changed:
                _serial_cfg["online"] = raw
        return {k: dict(v) for k, v in raw.items() if isinstance(v, dict)}


def _set_module_online(which: str, *, ok: bool, message: str) -> None:
    port = _serial_port(which)
    with _serial_cfg_lock:
        online = dict(_serial_cfg.get("online") or {})
        if not port:
            online.pop(which, None)
        else:
            online[which] = {
                "port": port,
                "ok": bool(ok),
                "message": str(message or "").strip(),
            }
        _serial_cfg["online"] = online
        motion = str(_serial_cfg.get("motion") or "")
        plc = str(_serial_cfg.get("plc") or "")
        andon = str(_serial_cfg.get("andon") or "")
        prefeeder = str(_serial_cfg.get("prefeeder") or "")
        andon_disabled = bool(_serial_cfg.get("andon_disabled"))
        checklist_bypass = bool(_serial_cfg.get("checklist_bypass"))
        online_save = _normalize_online_map(online, motion, plc, andon, prefeeder)
        _serial_cfg["online"] = online_save
    SERIAL_PATH.parent.mkdir(parents=True, exist_ok=True)
    SERIAL_PATH.write_text(
        json.dumps(
            {
                "motion": motion,
                "plc": plc,
                "andon": andon,
                "prefeeder": prefeeder,
                "andon_disabled": andon_disabled,
                "checklist_bypass": checklist_bypass,
                "online": online_save,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def _ports_for_ui() -> list[dict]:
    """Solo COM detectados ahora. Si no aparece, el módulo queda en Sin puerto."""
    return _list_com_ports()


def _accept_assigned_port(raw: str, listed: list[dict], previous: str = "") -> str:
    """Acepta únicamente un COM presente en la lista actual."""
    del previous  # ya no se conservan asignaciones fantasma
    text = str(raw or "").strip()
    if not text:
        return ""
    return _match_listed_port(text, listed)


def _prune_missing_serial_assignments(listed: list[dict]) -> bool:
    """Si un COM guardado ya no está detectado, lo limpia a Sin puerto."""
    motion = _serial_port("motion")
    plc = _serial_port("plc")
    andon = _serial_port("andon")
    prefeeder = _serial_port("prefeeder")
    next_motion = _accept_assigned_port(motion, listed)
    next_plc = _accept_assigned_port(plc, listed)
    next_andon = _accept_assigned_port(andon, listed)
    next_prefeeder = _accept_assigned_port(prefeeder, listed)
    if (
        next_motion == motion
        and next_plc == plc
        and next_andon == andon
        and next_prefeeder == prefeeder
    ):
        return False
    with _serial_cfg_lock:
        andon_disabled = bool(_serial_cfg.get("andon_disabled"))
        checklist_bypass = bool(_serial_cfg.get("checklist_bypass"))
    _save_serial_cfg(
        next_motion,
        next_plc,
        next_andon,
        andon_disabled,
        checklist_bypass,
        next_prefeeder,
    )
    return True


def _save_serial_cfg(
    motion: str,
    plc: str,
    andon: str,
    andon_disabled: bool,
    checklist_bypass: bool,
    prefeeder: str | None = None,
) -> None:
    """Guarda serial.json. Parámetro andon_disabled: ANDON_SKIP_OPCIONAL — TODO_BORRAR."""
    motion = str(motion or "").strip()
    plc = str(plc or "").strip()
    andon = str(andon or "").strip()
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR ---
    if andon_disabled:
        andon = ""
    # ---
    with _serial_cfg_lock:
        if prefeeder is None:
            prefeeder = str(_serial_cfg.get("prefeeder") or "").strip()
        else:
            prefeeder = str(prefeeder or "").strip()
        prev_motion = _serial_cfg["motion"]
        prev_plc = _serial_cfg["plc"]
        prev_andon = _serial_cfg["andon"]
        prev_pf = str(_serial_cfg.get("prefeeder") or "").strip()
        prev_ad = bool(_serial_cfg.get("andon_disabled"))
        prev_online = dict(_serial_cfg.get("online") or {})
        _serial_cfg["motion"] = motion
        _serial_cfg["plc"] = plc
        _serial_cfg["andon"] = andon
        _serial_cfg["prefeeder"] = prefeeder
        _serial_cfg["andon_disabled"] = bool(andon_disabled)
        _serial_cfg["checklist_bypass"] = bool(checklist_bypass)
        online = _normalize_online_map(prev_online, motion, plc, andon, prefeeder)
        _serial_cfg["online"] = online
    SERIAL_PATH.parent.mkdir(parents=True, exist_ok=True)
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR: quitar clave andon_disabled del JSON ---
    SERIAL_PATH.write_text(
        json.dumps(
            {
                "motion": motion,
                "plc": plc,
                "andon": andon,
                "prefeeder": prefeeder,
                "andon_disabled": bool(andon_disabled),
                "checklist_bypass": bool(checklist_bypass),
                "online": online,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    if motion != prev_motion:
        _close_motion_serial()
    if plc != prev_plc:
        _close_plc_serial()
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR: la condición incluye prev_ad ---
    if andon != prev_andon or bool(andon_disabled) != prev_ad:
        _close_andon_serial()
    if prefeeder != prev_pf:
        _close_prefeeder_serial()
        _pf_pushed_signature["L"] = None
        _pf_pushed_signature["R"] = None


def _serial_rx_flush(ser) -> None:
    """Descarta datos ya en cola RX (boot/status al conectar) para no mezclar con la respuesta al comando."""
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    try:
        t_end = time.time() + 0.05
        while time.time() < t_end:
            n = getattr(ser, "in_waiting", 0) or 0
            if n:
                ser.read(n)
            else:
                time.sleep(0.01)
    except Exception:
        pass


def _read_serial_line(
    ser, timeout_s: float, carry: bytearray | None = None
) -> str:
    """Lee una línea terminada en \\n.

    Si se pasa carry, los bytes sin \\n se conservan entre llamadas (no se
    reportan como línea). Sin carry, al timeout se devuelve lo parcial — solo
    para lecturas one-shot con espera larga.
    """
    buf = carry if carry is not None else bytearray()
    deadline = time.time() + max(0.0, float(timeout_s))
    while time.time() < deadline:
        chunk = ser.read(1)
        if chunk:
            if chunk == b"\n":
                line = bytes(buf).decode("utf-8", "replace").strip()
                del buf[:]
                return line
            if chunk != b"\r":
                buf.extend(chunk)
        else:
            time.sleep(0.02)
    if carry is None and buf:
        return bytes(buf).decode("utf-8", "replace").strip()
    return ""


def _read_lines_until_op(
    ser,
    want_op: str,
    budget_s: float,
    max_lines: int = 800,
    trace_prefix: str | None = None,
    inject=None,
    on_line=None,
) -> list[str]:
    """Lee líneas completas hasta la respuesta pedida, conservando las intermedias.

    inject() corre con el candado de escritura: None sigue esperando, [] no
    escribe, una lista se manda una sola vez. Con inject activo se usan
    rebanadas cortas de lectura; carry evita partir un JSON a mitad de línea
    (p. ej. `{` + resto) y colgar el paso 8 del solape.
    """
    out: list[str] = []
    seen: set[str] = set()
    injected = False
    carry = bytearray()
    deadline = time.time() + max(0.05, budget_s)
    abort_drain_until = 0.0

    def _consume(line: str) -> bool:
        """True si ya tenemos la respuesta pedida (y no falta feed_stage del solape)."""
        if not line:
            return False
        out.append(line)
        if trace_prefix:
            print(f"{trace_prefix} {line}", flush=True)
        try:
            msg = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            return False
        op = msg.get("op")
        if op:
            seen.add(op)
            if op == "laser_stop_trace":
                _laser_stop_log_write(msg)
            if on_line is not None:
                on_line(msg)
        if want_op not in seen:
            return False
        # feed_error: la respuesta ya llegó (fallida) en un paso anterior; no
        # re-esperar feed_stage o el paso 8 cuelga ~100 s con "Ciclo en marcha".
        overlap_feed_pending = (
            inject is not None
            and _OVERLAP.get("on")
            and _OVERLAP.get("feed_sent")
            and not _OVERLAP.get("feed_done")
            and not _OVERLAP.get("feed_error")
            and "feed_stage" not in seen
        )
        return not overlap_feed_pending

    while time.time() < deadline and len(out) < max_lines:
        if _cycle_should_abort():
            if abort_drain_until <= 0.0:
                abort_drain_until = time.time() + 0.4
            if time.time() >= abort_drain_until:
                break
        if inject is not None and not injected and not _cycle_should_abort():
            with _motion_write_lock:
                extra = inject()
                if extra is not None:
                    injected = True
                    if extra:
                        for line in extra:
                            print(f"[Motion TX] {line}", flush=True)
                            ser.write((line + "\n").encode("ascii", "replace"))
                        ser.flush()
            if injected and extra:
                for line in extra:
                    _push_event(f"[Motion] {line}")
        rem = deadline - time.time()
        if rem <= 0:
            break
        if abort_drain_until > 0.0:
            rem = min(rem, max(0.02, abort_drain_until - time.time()))
        if inject is not None and not injected and not carry:
            slice_s = 0.15
        else:
            slice_s = min(2.0, rem)
        line = _read_serial_line(ser, min(slice_s, rem), carry)
        if _consume(line):
            break
        if not line:
            time.sleep(0.02)
    return out


_plc_serial = None
_plc_serial_lock = threading.Lock()
_andon_serial = None
_andon_serial_lock = threading.Lock()
_andon_state_sent = ""
_andon_attempted = ""
_prefeeder_serial = None
_prefeeder_serial_lock = threading.Lock()
_pf_pushed_signature: dict[str, tuple | None] = {"L": None, "R": None}
_pf_last_fault_sig: tuple[str, ...] = ()
_PF_CACHED_STATUS: dict | None = None
_PF_ERROR_LATCH = False
_PF_NEED_REOPEN = False
_PF_STABILIZE_TIMEOUT_S = 45.0
_PF_SERIAL_REPLY_S = 6.0
_PF_SOFT_ERRORS = frozenset({"E090", "E091", "E092", "E099"})
_RECOVERY_LINK_SOFT_RESET = frozenset({
    "E017", "E018", "E019",
    "E041", "E042", "E043",
    "E070", "E071", "E072",
    "E090", "E091", "E092",
})
_PF_MANUAL_HOLD = {"L": False, "R": False}
_PF_MANUAL_HOLD_LOCK = threading.Lock()
_PF_UI_REFRESH_AT = 0.0
_PLC_VALVES = ("holder", "encoder", "cutter_r", "cutter_l", "gripper")
_load_serial_cfg()


def _send_plc_serial(lines):
    port = _serial_port("plc")
    if not port:
        return {
            "sent": False,
            "reason": "Sin puerto del PLC. Elígelo en Parámetros, pestaña Conexión.",
            "replies": [],
        }
    try:
        import serial
    except ImportError:
        return {
            "sent": False,
            "reason": "Falta pyserial para abrir el puerto.",
            "replies": [],
        }

    replies = []
    global _plc_serial
    try:
        with _plc_serial_lock:
            if (
                _plc_serial is None
                or not _plc_serial.is_open
                or getattr(_plc_serial, "port", None) != port
            ):
                if _plc_serial is not None:
                    try:
                        _plc_serial.close()
                    except Exception:
                        pass
                ser = serial.Serial()
                ser.port = port
                ser.baudrate = int(os.environ.get("PLC_SERIAL_BAUD", "115200"))
                ser.timeout = 0.05
                ser.write_timeout = 2
                ser.dtr = False
                ser.rts = False
                ser.open()
                time.sleep(0.05)
                # Boot push del PLC (status); no descartar — alimenta el espejo.
                _plc_rx_seed_mirror(ser)
                _plc_serial = ser
            ser = _plc_serial
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
                try:
                    request = json.loads(line)
                    expected_op = str(request.get("op") or "")
                except (TypeError, json.JSONDecodeError):
                    request = {}
                    expected_op = ""
                if expected_op:
                    wait_s = 2.0
                    if expected_op == "valve":
                        for key in ("ms", "settle_ms"):
                            if request.get(key):
                                wait_s = max(
                                    wait_s, float(request[key]) / 1000.0 + 0.5
                                )
                    replies.extend(_read_lines_until_op(ser, expected_op, wait_s))
                else:
                    reply = _read_serial_line(ser, 2.0)
                    if reply:
                        replies.append(reply)
    except Exception as exc:
        with _plc_serial_lock:
            _invalidate_serial_handle(_plc_serial)
            _plc_serial = None
        _plc_mirror_invalidate()
        return {"sent": False, "reason": str(exc), "replies": replies}
    for raw in replies:
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") in ("status", "valve"):
            _plc_mirror_apply(msg)
    return {"sent": True, "reason": "", "replies": replies}


def _send_andon_serial(lines):
    port = _serial_port("andon")
    if not port:
        return {
            "sent": False,
            "reason": "Sin puerto del Andon. Elígelo en Parámetros, pestaña Conexión.",
            "replies": [],
        }
    try:
        import serial
    except ImportError:
        return {
            "sent": False,
            "reason": "Falta pyserial para abrir el puerto.",
            "replies": [],
        }

    replies = []
    global _andon_serial
    try:
        with _andon_serial_lock:
            if (
                _andon_serial is None
                or not _andon_serial.is_open
                or getattr(_andon_serial, "port", None) != port
            ):
                if _andon_serial is not None:
                    try:
                        _andon_serial.close()
                    except Exception:
                        pass
                ser = serial.Serial()
                ser.port = port
                ser.baudrate = 115200
                ser.timeout = 0.05
                ser.write_timeout = 2
                ser.dtr = False
                ser.rts = False
                ser.open()
                time.sleep(0.05)
                _serial_rx_flush(ser)
                _andon_serial = ser
            ser = _andon_serial
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
                try:
                    expected_op = str(json.loads(line).get("op") or "")
                except (TypeError, json.JSONDecodeError):
                    expected_op = ""
                if expected_op:
                    replies.extend(_read_lines_until_op(ser, expected_op, 2.0))
                else:
                    reply = _read_serial_line(ser, 2.0)
                    if reply:
                        replies.append(reply)
    except Exception as exc:
        with _andon_serial_lock:
            _invalidate_serial_handle(_andon_serial)
            _andon_serial = None
        return {"sent": False, "reason": str(exc), "replies": replies}
    return {"sent": True, "reason": "", "replies": replies}


def _open_prefeeder_serial_unlocked(port: str):
    """Abre el COM del PreFeeder. Caller debe tener _prefeeder_serial_lock.

    Tras Monitor Serial / otro proceso, Windows puede tardar en liberar el COM:
    se cierra el handle previo, se espera y se reintenta el open.
    """
    import serial

    global _prefeeder_serial
    last_exc: Exception | None = None
    for attempt in range(5):
        _invalidate_serial_handle(_prefeeder_serial)
        _prefeeder_serial = None
        # Liberación del handle en Windows (Access denied si se reabre al instante).
        time.sleep(0.2 + 0.25 * attempt)
        ser = None
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = 115200
            ser.timeout = 0.05
            ser.write_timeout = 2
            ser.dtr = False
            ser.rts = False
            try:
                ser.exclusive = True
            except Exception:
                pass
            ser.open()
            # Si DTR pulsó, el Master reinicia (~0.3 s en setup).
            time.sleep(0.55 if attempt == 0 else 0.85)
            _serial_rx_flush(ser)
            _prefeeder_serial = ser
            return ser
        except Exception as exc:
            last_exc = exc
            _invalidate_serial_handle(ser)
            _prefeeder_serial = None
    if last_exc is not None:
        raise last_exc
    raise OSError(f"No se pudo abrir {port}")


def _send_prefeeder_serial(lines, *, force_reopen: bool = False):
    port = _serial_port("prefeeder")
    if not port:
        return {
            "sent": False,
            "reason": "Sin puerto del PreAlimentador. Elígelo en Parámetros, pestaña Conexión.",
            "replies": [],
        }
    try:
        import serial  # noqa: F401
    except ImportError:
        return {
            "sent": False,
            "reason": "Falta pyserial para abrir el puerto.",
            "replies": [],
        }

    replies = []
    global _prefeeder_serial, _PF_NEED_REOPEN
    try:
        with _prefeeder_serial_lock:
            need_open = (
                force_reopen
                or _PF_NEED_REOPEN
                or _prefeeder_serial is None
                or not _prefeeder_serial.is_open
                or getattr(_prefeeder_serial, "port", None) != port
            )
            if need_open:
                ser = _open_prefeeder_serial_unlocked(port)
                _PF_NEED_REOPEN = False
            else:
                ser = _prefeeder_serial
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
                try:
                    expected_op = str(json.loads(line).get("op") or "")
                except (TypeError, json.JSONDecodeError):
                    expected_op = ""
                if expected_op:
                    got = _read_lines_until_op(ser, expected_op, _PF_SERIAL_REPLY_S)
                    replies.extend(got)
                    # Write “pasa” pero el handle está muerto tras Monitor Serial.
                    if not got:
                        _PF_NEED_REOPEN = True
                        _invalidate_serial_handle(_prefeeder_serial)
                        _prefeeder_serial = None
                else:
                    reply = _read_serial_line(ser, _PF_SERIAL_REPLY_S)
                    if reply:
                        replies.append(reply)
                    else:
                        _PF_NEED_REOPEN = True
                        _invalidate_serial_handle(_prefeeder_serial)
                        _prefeeder_serial = None
    except Exception as exc:
        with _prefeeder_serial_lock:
            _invalidate_serial_handle(_prefeeder_serial)
            _prefeeder_serial = None
            _PF_NEED_REOPEN = True
        return {"sent": False, "reason": str(exc), "replies": replies}
    return {"sent": True, "reason": "", "replies": replies}


def _prefeeder_status_from_replies(replies):
    for raw in reversed(replies or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == "status" and msg.get("mcu") == "prefeeder":
            return msg
    return None


def _prefeeder_status_get() -> dict | None:
    """Pide status; si el COM quedó mudo (p. ej. tras Monitor Serial), reabre el puerto."""
    global _PF_NEED_REOPEN
    for attempt in range(4):
        sent = _send_prefeeder_serial(
            ['{"op":"status"}'],
            force_reopen=(attempt > 0 or _PF_NEED_REOPEN),
        )
        if not sent.get("sent"):
            _PF_NEED_REOPEN = True
            if attempt < 3:
                time.sleep(0.45 + 0.2 * attempt)
            continue
        st = _prefeeder_status_from_replies(sent.get("replies"))
        if st:
            _PF_NEED_REOPEN = False
            return st
        _PF_NEED_REOPEN = True
        if attempt < 3:
            _close_prefeeder_serial()
            time.sleep(0.45 + 0.2 * attempt)
    return None


def _prefeeder_fault_codes_active(status: dict | None) -> list[str]:
    """Fallas vigentes del PreAlimentador.

    Lado fuera de feed_sides: se ignora por completo (no espera ni bloquea).
    """
    codes = error_codes.prefeeder_fault_codes(status)
    if not codes:
        return []
    watched = set(_active_feed_sides())
    out: list[str] = []
    for code in codes:
        if code in _PFEEDER_CODES_L and "L" not in watched:
            continue
        if code in _PFEEDER_CODES_R and "R" not in watched:
            continue
        out.append(code)
    return out


def _prefeeder_active_errors(status: dict | None) -> list[dict]:
    if not status:
        return []
    items = []
    for code in _prefeeder_fault_codes_active(status):
        items.append({"code": code, "message": error_codes.format_message(code)})
    return items


def _prefeeder_cfg_from_status_side(st: dict | None, side: str) -> dict | None:
    if not isinstance(st, dict):
        return None
    block = st.get(side)
    if not isinstance(block, dict):
        return None
    cfg = block.get("cfg")
    if not isinstance(cfg, dict):
        return None
    return _canonical_prefeeder_side({side: cfg}, side)


def _prefeeder_reconcile_pushed_from_status(st: dict | None) -> None:
    """Alinea la caché de envío con el cfg real del maestro."""
    if not isinstance(st, dict):
        return
    for side in ("L", "R"):
        live = _prefeeder_cfg_from_status_side(st, side)
        if live is not None:
            _pf_pushed_signature[side] = _prefeeder_cfg_signature(live)


def _prefeeder_set_config_side(side: str, cfg: dict) -> tuple[bool, bool]:
    """Envía set_config al maestro solo si la firma cambió. Retorna (ok, changed)."""
    side = side.upper()
    if side not in ("L", "R"):
        return False, False
    canonical = _canonical_prefeeder_side({side: cfg}, side)
    sig = _prefeeder_cfg_signature(canonical)
    if _pf_pushed_signature.get(side) == sig:
        return True, False
    payload = {"op": "set_config", "side": side, **canonical}
    sent = _send_prefeeder_serial([_compact(payload)])
    if not sent.get("sent"):
        return False, False
    changed = False
    ok = False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == "set_config" and str(msg.get("side") or "").upper() == side:
            ok = bool(msg.get("ok", False))
            changed = bool(msg.get("changed", False))
            break
    if ok and (changed or _pf_pushed_signature.get(side) is None):
        _pf_pushed_signature[side] = sig
    return ok, changed


def _prefeeder_adopt_live_config() -> None:
    """Alinea archivo/firma con el cfg del maestro (NVS). No sobrescribe el ESP32."""
    if not _serial_port("prefeeder"):
        return
    st = _prefeeder_refresh_cache()
    if not st:
        return
    try:
        tune = _load_prefeeder_tune()
    except (OSError, json.JSONDecodeError):
        tune = _canonical_prefeeder_tune({})
    changed = False
    for side in ("L", "R"):
        live = _prefeeder_cfg_from_status_side(st, side)
        if live is None:
            continue
        if _prefeeder_cfg_signature(tune[side]) != _prefeeder_cfg_signature(live):
            tune[side] = live
            changed = True
    if changed:
        try:
            _save_prefeeder_tune(tune)
        except OSError:
            pass


def _prefeeder_required() -> bool:
    return bool(_serial_port("prefeeder"))


def _prefeeder_cmd(payload: dict) -> dict:
    return _send_prefeeder_serial([_compact(payload)])


def _prefeeder_reply_ok(sent: dict, op: str) -> bool:
    if not sent.get("sent"):
        return False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            return bool(msg.get("ok", True))
    return False


def _prefeeder_set_feed_lock(locked: bool) -> bool:
    return _prefeeder_reply_ok(
        _prefeeder_cmd({"op": "set_feed_lock", "locked": bool(locked)}),
        "set_feed_lock",
    )


def _prefeeder_set_force(side: str, on: bool) -> bool:
    side = str(side or "").upper()
    if side not in ("L", "R"):
        return False
    return _prefeeder_reply_ok(
        _prefeeder_cmd({"op": "set_force_feed", "side": side, "on": bool(on)}),
        "set_force_feed",
    )


def _prefeeder_side_idle_ok(block: dict | None) -> bool:
    if not isinstance(block, dict) or not block.get("online"):
        return False
    try:
        fault = int(block.get("fault") or 0)
    except (TypeError, ValueError):
        fault = 0
    if fault:
        return False
    state = str(block.get("state") or "").strip().lower()
    return state == "idle"


def _prefeeder_status_ready(status: dict | None) -> bool:
    """Listo según lados de feed_sides (el maestro viejo exige L+R siempre)."""
    if not status:
        return False
    if not status.get("feed_lock"):
        return False
    if _prefeeder_fault_codes_active(status):
        return False
    for side in _active_feed_sides():
        if not _prefeeder_side_idle_ok(status.get(side)):
            return False
    return True


def _prefeeder_offline_sides(status: dict | None) -> tuple[str, ...]:
    """Lados activos (feed_sides) que el maestro no ve online."""
    if not isinstance(status, dict):
        return tuple(_active_feed_sides())
    offline: list[str] = []
    for side in _active_feed_sides():
        block = status.get(side)
        if not isinstance(block, dict) or not block.get("online"):
            offline.append(side)
    return tuple(offline)


def _prefeeder_not_ready_message(status: dict | None) -> str:
    """Motivo concreto de E099 (evita ‘no listo’ genérico cuando un esclavo cayó)."""
    if not status:
        return error_codes.format_message("E092")
    faults = _prefeeder_fault_codes_active(status)
    if faults:
        _, msg = _machine_fault_message(tuple(faults))
        return msg
    offline = _prefeeder_offline_sides(status)
    if offline:
        sides = "/".join(offline)
        return (
            f"E099 PreAlimentador: lado {sides} sin enlace WiFi al maestro "
            "(revisa alimentación/red del esclavo)."
        )
    if not status.get("feed_lock"):
        return (
            "E099 PreAlimentador: desbloqueado — pulsa Iniciar para estabilizar y bloquear."
        )
    return error_codes.format_message("E099")


def _prefeeder_stop_now() -> None:
    """Frena alimentación manual y bloquea de inmediato."""
    if not _prefeeder_required():
        return
    _prefeeder_set_force("L", False)
    _prefeeder_set_force("R", False)
    _prefeeder_set_feed_lock(True)
    _prefeeder_refresh_cache()


def _prefeeder_refresh_cache() -> dict | None:
    global _PF_CACHED_STATUS
    st = _prefeeder_status_get()
    if st:
        _PF_CACHED_STATUS = st
        _prefeeder_reconcile_pushed_from_status(st)
    return st


def _prefeeder_comm_active() -> bool:
    """Poll al maestro si hay COM (evita banner ‘sin enlace’ por no consultar en idle)."""
    if not _serial_port("prefeeder"):
        return False
    return True


def _prefeeder_fault_sig(st: dict | None) -> tuple[str, ...]:
    return tuple(sorted(_prefeeder_fault_codes_active(st)))


def _machine_fault_message(codes: tuple[str, ...]) -> tuple[str, str]:
    primary = codes[0]
    if len(codes) == 1:
        return primary, error_codes.format_message(primary)
    msg = " · ".join(error_codes.format_message(c) for c in codes)
    return primary, msg


def _recovery_blank() -> dict:
    return {
        "active": False,
        "step": "",
        "code": "",
        "mensaje": "",
        "guide": "",  # instrucciones Joint (cambian); mensaje = error fijo
        "kind": "",
        "side": "",
        "origin": "",
        "busy": False,
        "lm_at_park": False,
    }


def _recovery_get() -> dict:
    with _STATE_LOCK:
        rec = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
    for key, default in _recovery_blank().items():
        rec.setdefault(key, default)
    return rec


def _recovery_set(**fields) -> None:
    with _STATE_LOCK:
        rec = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
        rec.update(fields)
        _MACHINE_STATE["recovery"] = rec


def _recovery_begin(code: str, msg: str) -> None:
    """Inicia recuperación: mensaje = error fijo; guide = instrucciones del paso."""
    code_s = str(code or "").strip()
    msg_s = str(msg or "").strip()
    _recovery_set(
        active=True,
        step="error",
        kind="",
        side="",
        origin="",
        busy=False,
        code=code_s,
        mensaje=msg_s,
        guide=(
            "Pulsa Reset para soltar la pieza. Luego sigue los pasos."
            if code_s not in ("E094", "E097")
            else (
                "Revisa el PreAlimentador y deja holgura. "
                "Cuando el límite se libere, pulsa Reset."
            )
        ),
    )


def _recovery_set_guide(msg: str) -> None:
    """Instrucciones del paso (popup). No pisa el error fijo (recovery.mensaje)."""
    _recovery_set(guide=str(msg or "").strip())


def _recovery_clear() -> None:
    with _STATE_LOCK:
        _MACHINE_STATE["recovery"] = _recovery_blank()
    _joint_latch_clear()


def _joint_latch_clear() -> None:
    _JOINT["skip_cut"] = ""
    _JOINT["finish_piece"] = False
    _JOINT["piece_counted"] = False
    _JOINT["holder_released"] = False


def _joint_active() -> bool:
    rec = _recovery_get()
    return bool(rec.get("active") and rec.get("kind") == "joint")


def _joint_code_for_side(side: str) -> str:
    if side == "RL":
        return "E052"
    if side == "L":
        return "E051"
    return "E050"


def _joint_side_label(side: str) -> str:
    return {"R": "Right", "L": "Left", "RL": "Right y Left"}.get(side, side or "?")


def _joint_side_from_feed_error(error_name: str) -> str:
    err = (error_name or "").strip().lower()
    if err in ("joint_rl", "joint_r_l"):
        return "RL"
    if err == "joint_l":
        return "L"
    if err == "joint_r":
        return "R"
    return ""


def _joint_merge_side(a: str, b: str) -> str:
    sides = set()
    for s in (a, b):
        if s == "RL":
            sides.update(("R", "L"))
        elif s in ("R", "L"):
            sides.add(s)
    if "R" in sides and "L" in sides:
        return "RL"
    if "R" in sides:
        return "R"
    if "L" in sides:
        return "L"
    return ""


def _joint_poll_sides_from_status(mst: dict | None) -> str:
    if not isinstance(mst, dict):
        return ""
    r_on = False
    l_on = False
    br = mst.get("enc_R")
    bl = mst.get("enc_L")
    if isinstance(br, dict) and br.get("joint") is True:
        r_on = True
    if isinstance(bl, dict) and bl.get("joint") is True:
        l_on = True
    if r_on and l_on:
        return "RL"
    if r_on:
        return "R"
    if l_on:
        return "L"
    return ""


def _joint_blocks_next_piece() -> bool:
    return bool(_JOINT.get("finish_piece") or _joint_active())


def _joint_valve_allowed(valve: str) -> str | None:
    """None = omitir corte. Puede reducir cutters a un solo lado."""
    skip = str(_JOINT.get("skip_cut") or "")
    valve = str(valve or "")
    if not skip:
        return valve
    if skip == "RL":
        return None
    if valve == "cutters":
        if skip == "R":
            return "cutter_l"
        if skip == "L":
            return "cutter_r"
        return valve
    if valve == "cutter_r" and skip == "R":
        return None
    if valve == "cutter_l" and skip == "L":
        return None
    return valve


def _joint_ui_warning(side: str, msg: str) -> None:
    """Banner Joint: solo warning fijo (sin paso a paso)."""
    keep = bool(
        _MACHINE_STATE.get("ciclo_en_curso")
        or _LOT.get("active")
        or _LOT.get("tray_wait")
    )
    side_lbl = _joint_side_label(side)
    _set_machine_ui(
        estado="paused",
        estado_label="Warning Joint",
        mensaje=f"Joint {side_lbl}. Abre el aviso para continuar.",
        en_curso=keep,
        ciclo_error="",
    )


def _joint_set_guide(msg: str, *, lead: str | None = None) -> None:
    """Guía Joint solo en popup (recovery.guide). No pisa el error fijo ni el banner."""
    text = str(msg or "").strip()
    _recovery_set(guide=text)
    keep = bool(
        _MACHINE_STATE.get("ciclo_en_curso")
        or _LOT.get("active")
        or _LOT.get("tray_wait")
        or _joint_active()
    )
    side = str(_recovery_get().get("side") or "")
    side_lbl = _joint_side_label(side) if side else ""
    banner = (
        f"Joint {side_lbl}. Abre el aviso para continuar."
        if side_lbl
        else "Joint. Abre el aviso para continuar."
    )
    _set_machine_ui(
        estado="paused",
        estado_label=lead or "Warning Joint",
        mensaje=banner,
        en_curso=keep,
        ciclo_error="",
    )


def _joint_begin_common(side: str, *, origin: str, step: str) -> None:
    side = _joint_merge_side(side, side) or "R"
    code = _joint_code_for_side(side)
    msg = error_codes.format_message(code)
    _JOINT["skip_cut"] = side
    _JOINT["holder_released"] = False
    if origin == "fabricate" and side != "RL":
        _JOINT["finish_piece"] = True
        _JOINT["piece_counted"] = False
    else:
        _JOINT["finish_piece"] = False
        _JOINT["piece_counted"] = False
    _cadence_mark_interrupted()
    _push_event(
        f"WARNING Joint {_joint_side_label(side)} ({origin}).",
        "warn",
    )
    try:
        _prod_log_joint(side, origin=origin, code=code, mensaje=msg)
    except Exception:
        pass
    _recovery_set(
        active=True,
        kind="joint",
        step=step,
        side=side,
        origin=origin,
        code=code,
        mensaje=msg,  # error fijo (E050/E051/E052)
        guide="",
        busy=False,
    )
    _joint_ui_warning(side, msg)


def _joint_park_run() -> None:
    """Avisa Park, mueve LM a Park; luego step=operator."""
    _recovery_set(step="park", busy=True, lm_at_park=False)
    _joint_set_guide(
        f"Voy a Park ({PURGE_PARK_MM:.0f} mm). Espera sin meter las manos.",
        lead="Warning Joint · Park",
    )
    params = _load_cycle_params()
    ok_hw, code = _plc_accept_valve(
        _cycle_plc_send(
            {
                "op": "valve",
                "name": "holder_encoder",
                "on": True,
                "settle_ms": int(params["ms_holder_enc_stable"]),
            }
        ),
        {"holder": True, "encoder": True},
    )
    if _ROUTINE["stop"].is_set():
        _recovery_set(step="operator", busy=False)
        return
    if not ok_hw:
        if code:
            _push_event(error_codes.format_message(code), "error")
        _recovery_set(step="operator", busy=False)
        _joint_set_guide(
            "Holder/encoder sin confirmación. Usa Alimentar, Liberar o Continuar."
        )
        return
    _joint_set_guide(
        f"Moviendo LM Guide a Park ({PURGE_PARK_MM:.0f} mm). Espera.",
        lead="Warning Joint · Park",
    )
    if _ROUTINE["stop"].is_set() or not _routine_move_lm_mm(PURGE_PARK_MM):
        _recovery_set(step="operator", busy=False)
        _joint_set_guide("No se alcanzó la posición segura. Usa Funciones Extras o Continuar.")
        return
    _recovery_set(step="operator", busy=False, lm_at_park=True)
    _joint_set_guide(
        "Libera el Joint hasta Apagado. Luego pulsa Continuar para iniciar la limpieza."
    )


def _joint_begin_from_feed(side: str) -> None:
    """Escenario A: Joint durante FeedStage."""
    mst = _motion_status_get()
    side = _joint_merge_side(side, _joint_poll_sides_from_status(mst)) or side
    if _serial_port("motion"):
        _send_motion_urgent(['{"op":"feeder_stop"}'])
    _joint_begin_common(side, origin="feed", step="park")
    _joint_set_guide(
        f"Joint {_joint_side_label(side)} detectado en alimentación. Voy a Park.",
        lead="Warning Joint · Park",
    )
    ok, err = _start_routine(_joint_park_run)
    if not ok:
        _recovery_set(step="operator", busy=False)
        _joint_set_guide(
            (err or "No se pudo iniciar Park.")
            + " Usa Alimentar, Liberar o Continuar."
        )


def _joint_begin_immediate(side: str, *, origin: str) -> None:
    """Ambos Joint o salida forzada: Park + operator sin terminar pieza."""
    if _serial_port("motion"):
        _send_motion_urgent(['{"op":"feeder_stop"}', '{"op":"lmguide_stop"}'])
    _joint_begin_common(side, origin=origin, step="park")
    _JOINT["finish_piece"] = False
    _joint_set_guide(
        f"Joint {_joint_side_label(side)}: paro inmediato. Voy a Park.",
        lead="Warning Joint · Park",
    )
    ok, err = _start_routine(_joint_park_run)
    if not ok:
        _recovery_set(step="operator", busy=False)
        _joint_set_guide(
            (err or "No se pudo iniciar Park.")
            + " Usa Alimentar, Liberar o Continuar."
        )


def _joint_latch_fabricate(side: str) -> None:
    """Escenario B: Joint en pasos 4–8; omitir corte y terminar pieza buena."""
    if _cycle_error_active():
        return
    mst_side = side
    if _joint_active():
        prev = str(_recovery_get().get("side") or "")
        merged = _joint_merge_side(prev, mst_side)
        if merged == prev:
            return
        side = merged
        _JOINT["skip_cut"] = side
        _recovery_set(side=side, code=_joint_code_for_side(side), mensaje=error_codes.format_message(_joint_code_for_side(side)))
        if side == "RL":
            _JOINT["finish_piece"] = False
            _joint_begin_immediate("RL", origin="fabricate")
        return
    if side == "RL":
        _joint_begin_immediate("RL", origin="fabricate")
        return
    _joint_begin_common(side, origin="fabricate", step="await_home")
    _joint_set_guide(
        f"Joint {_joint_side_label(side)}: se omite su corte. "
        "El lado correcto termina la pieza y regresa a Home. Luego iré a Park."
    )


def _joint_check_during_cycle(step_id: str) -> bool:
    """
    True = continuar el ciclo.
    False = detener el worker (Joint RL inmediato u operator pendiente).
    """
    if _cycle_error_active():
        return True
    if _joint_active():
        if _JOINT.get("finish_piece") and str(_recovery_get().get("step") or "") == "await_home":
            return True
        return False
    if step_id in ("1", "2", "3"):
        return True
    mst = _motion_status_get()
    side = _joint_poll_sides_from_status(mst)
    if not side:
        return True
    if side == "RL":
        _joint_begin_immediate("RL", origin="fabricate")
        return False
    _joint_latch_fabricate(side)
    return True


def _joint_enter_operator_after_piece() -> None:
    """Pieza buena terminada → aviso Park → Park → operator."""
    _JOINT["finish_piece"] = False
    _JOINT["piece_counted"] = True
    side = str(_recovery_get().get("side") or "")
    label = _joint_side_label(side)
    msg = f"Joint {label}: pieza del lado correcto terminada. Voy a Park."
    _push_event(msg, "warn")
    _recovery_set(step="park", busy=True)
    _joint_set_guide(msg, lead="Warning Joint · Park")
    ok, err = _start_routine(_joint_park_run)
    if not ok:
        _recovery_set(step="operator", busy=False)
        _joint_set_guide(
            (err or "No se pudo iniciar Park.")
            + " Usa Alimentar, Liberar o Continuar."
        )


def _feed_clean_side(side: str | None):
    """FeedStage clean 3 s; side R/L o ambos si None/RL."""
    payload = _feed_stage_payload(side if side in ("R", "L") else None)
    payload["mode"] = "clean"
    payload["timeout_ms"] = PURGE_CLEAN_MS
    sent = _cycle_motion_send(payload, timeout_s=8.0)
    found, ok, error_name, feed = _read_feed_stage(sent)
    _set_machine_ui(feed=feed)
    if not sent.get("sent"):
        return False, sent.get("code") or "E043", feed
    if not found or not ok:
        err = error_name or "unknown"
        return False, error_codes.feed_stage_code(err), feed
    return True, "", feed


def _joint_release_holder_encoder() -> tuple[bool, str]:
    ok_hw, code = _plc_accept_valve(
        _cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": False}),
        {"holder": False, "encoder": False},
    )
    if not ok_hw:
        return False, code or "E014"
    _JOINT["holder_released"] = True
    return True, ""


def _joint_liberar_actuators() -> tuple[bool, str]:
    """Libera gripper + holder + encoder (feeder queda libre al soltar holder/encoder)."""
    ok_g, code_g = _plc_accept_valve(
        _cycle_plc_send({"op": "valve", "name": "gripper", "on": False}),
        {"gripper": False},
    )
    if not ok_g:
        return False, code_g or "E019"
    _gripper_holding_set(False)
    return _joint_release_holder_encoder()


def _joint_clean_feed_run() -> None:
    rec = _recovery_get()
    side = str(rec.get("side") or "")
    feed_side = None if side == "RL" else (side if side in ("R", "L") else None)
    label = _joint_side_label(side)
    _joint_set_guide(f"Alimentando lado {label} (~3 s). Espera.")
    if not _purga_close_holder():
        _recovery_set(busy=False, step="operator")
        _joint_set_guide("No se cerró holder/encoder. Reintenta Alimentar o usa Liberar.")
        return
    _JOINT["holder_released"] = False
    ok, code, _feed = _feed_clean_side(feed_side)
    if _ROUTINE["stop"].is_set():
        _recovery_set(busy=False, step="operator")
        return
    if not ok:
        _push_event(error_codes.format_message(code), "error")
        _recovery_set(busy=False, step="operator")
        _joint_set_guide("Falló la alimentación. Reintenta Alimentar o usa Liberar.")
        return
    # Apagar al terminar para poder volver a pulsar Alimentar.
    released, rel_code = _joint_release_holder_encoder()
    _recovery_set(busy=False, step="operator")
    if not released:
        _joint_set_guide(
            "Alimentación hecha, pero no se apagó holder/encoder. "
            "Usa Liberar y luego Continuar."
            + (f" ({error_codes.format_message(rel_code)})" if rel_code else "")
        )
        return
    _joint_set_guide(
        f"Alimentación {label} terminó. Libera el Joint hasta Apagado y pulsa Continuar."
    )


def _joint_finish_purga_run() -> None:
    """Purga post-Joint: Park + corte de limpieza (reusa _purge_cut)."""
    _ROUTINE["from_lote"] = False
    with _STATE_LOCK:
        _MACHINE_STATE["post_error_purga"] = True
    _recovery_set(step="purga", busy=True)
    _set_routine_ui(
        rutina="purga",
        rutina_fase="segura",
        rutina_manos=True,
        estado_label="Purga",
        mensaje="NO INTRODUCIR MANOS. Purga Joint en curso, espera.",
    )
    _joint_set_guide("NO INTRODUCIR MANOS. Purga Joint en curso, espera.")
    if not _routine_sleep_ms(500):
        _purga_finish(True)
        _recovery_set(step="operator", busy=False)
        return
    if not _routine_try_reset_fault() or _ROUTINE["stop"].is_set():
        _purga_finish(True)
        _recovery_set(step="operator", busy=False)
        return
    if not _cycle_plc_send({"op": "all_off"}).get("sent"):
        _purga_finish(True)
        _recovery_set(step="operator", busy=False)
        return
    _gripper_holding_set(False)
    _JOINT["holder_released"] = True
    guide_park = f"NO METAS LAS MANOS. Llevando el eje a parque ({PURGE_PARK_MM:.0f} mm)."
    _set_routine_ui(mensaje=guide_park)
    _joint_set_guide(guide_park)
    if _ROUTINE["stop"].is_set() or not _routine_move_lm_mm(PURGE_PARK_MM):
        _purga_finish(True)
        _recovery_set(step="operator", busy=False)
        return
    ok = False
    try:
        ok = _purge_cut()
    finally:
        if not ok or _ROUTINE["stop"].is_set():
            _cycle_plc_send({"op": "all_off"})
            _purga_finish(True)
            _recovery_set(step="operator", busy=False)
        else:
            _purga_finish(False)


def _joint_claim_operator(action: str) -> tuple[bool, str]:
    rec = _recovery_get()
    if not _joint_active():
        return False, "No hay recuperación Joint activa."
    if rec.get("step") != "operator":
        return False, "Joint aún no está en intervención de operador."
    if rec.get("busy"):
        return False, "Joint ocupado. Espera a que termine la acción."
    if _MACHINE_STATE.get("estado") == "error" or _MACHINE_STATE.get("ciclo_error"):
        return False, "Hay un error crítico activo. Atiéndelo primero."
    if _MACHINE_STATE.get("rutina") == "purga" and action not in ("finish", "continuar"):
        return False, "Hay una purga en curso."
    return True, ""


def _joint_alimentar_action(_data=None) -> tuple[bool, str]:
    ok, err = _joint_claim_operator("alimentar")
    if not ok:
        return False, err
    ready, link_err = _require_links()
    if not ready:
        return False, link_err
    _recovery_set(busy=True)
    _joint_set_guide("Cerrando holder/encoder e iniciando alimentación…")
    started, start_err = _start_routine(_joint_clean_feed_run)
    if not started:
        _recovery_set(busy=False)
        return False, start_err
    return True, "Alimentación Joint 3 s."


def _joint_liberar_action(_data=None) -> tuple[bool, str]:
    """Libera gripper + holder + encoder."""
    ok, err = _joint_claim_operator("liberar")
    if not ok:
        return False, err
    _recovery_set(busy=True)
    _joint_set_guide("Liberando gripper, holder y encoder…")
    released, code = _joint_liberar_actuators()
    _recovery_set(busy=False)
    if not released:
        return False, error_codes.format_message(code) if code else "No se liberó gripper/holder/encoder."
    _joint_set_guide(
        "Sujeciones liberadas. Libera el Joint hasta Apagado y pulsa Continuar."
    )
    return True, "Liberados gripper, holder y encoder."


def _joint_sensors_clear_now(side: str | None = None) -> bool:
    """True si el Joint del lado afectado está Apagado (sin detección)."""
    mst = _motion_status_get()
    if mst is None:
        return False
    active = str(side or _recovery_get().get("side") or "").upper()
    check = ("R", "L")
    if active == "R":
        check = ("R",)
    elif active == "L":
        check = ("L",)
    for s in check:
        block = mst.get("enc_" + s)
        if not isinstance(block, dict):
            return False
        if block.get("joint") is True:
            return False
    return True


def _joint_continuar_action(_data=None) -> tuple[bool, str]:
    """Continuar = purga Joint (antes Finish)."""
    ok, err = _joint_claim_operator("continuar")
    if not ok:
        return False, err
    if not _joint_sensors_clear_now():
        return False, "El Joint aún detecta. Libera la unión hasta Apagado."
    ready, link_err = _require_links()
    if not ready:
        return False, link_err
    _recovery_set(busy=True, step="purga")
    _joint_set_guide("NO INTRODUCIR MANOS. Purga Joint en curso, espera.")
    started, start_err = _start_routine(_joint_finish_purga_run)
    if not started:
        _recovery_set(busy=False, step="operator")
        return False, start_err
    return True, "Purga Joint iniciada."


# Compat: nombres antiguos → mismos handlers.
def _joint_retry_action(_data=None) -> tuple[bool, str]:
    return _joint_alimentar_action(_data)


def _joint_confirm_action(_data=None) -> tuple[bool, str]:
    return _joint_liberar_action(_data)


def _joint_continue_action(_data=None) -> tuple[bool, str]:
    return _joint_liberar_action(_data)


def _joint_finish_action(_data=None) -> tuple[bool, str]:
    return _joint_continuar_action(_data)


def _recovery_cutters_clear_now() -> bool:
    """True si ambos cortadores estan apagados (válvula y sensor de falla)."""
    st = _plc_status_get()
    if st is None:
        return False
    if st.get("cutter_r") is True or st.get("cutter_l") is True:
        return False
    if st.get("fault_cutter") is True or st.get("sens_cutter") is True:
        return False
    return True


def _recovery_lasers_clear_now() -> bool:
    """True si los láseres de los lados activos están apagados (sin detección)."""
    mst = _motion_status_get()
    if mst is None:
        return False
    sides = _run_sides()
    check = ("R", "L") if sides not in ("R", "L") else (sides,)
    for side in check:
        block = mst.get("enc_" + side)
        if not isinstance(block, dict):
            return False
        if block.get("laser") is True:
            return False
    return True


def _enter_machine_error(primary: str, msg: str, *, en_curso: bool | None = None) -> None:
    """Un solo evento al entrar en error; el primer código queda congelado hasta Reset."""
    with _STATE_LOCK:
        in_error = _MACHINE_STATE.get("estado") == "error"
        had_code = bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
        existing_code = str(_MACHINE_STATE.get("ciclo_error") or "").strip()
        existing_msg = str(_MACHINE_STATE.get("mensaje") or "")
        if en_curso is None:
            en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
    if in_error or had_code:
        # Mismo stop: no pisar código/mensaje del popup con fallos posteriores.
        if not _recovery_get().get("active"):
            _recovery_begin(existing_code or primary, existing_msg or msg)
        if not in_error:
            _set_machine_ui(estado="error", estado_label="Error")
        return
    _push_event(msg, "error")
    with _STATE_LOCK:
        _MACHINE_STATE["post_error_purga"] = True
    _recovery_begin(primary, msg)
    _set_machine_ui(
        estado="error",
        estado_label="Error",
        mensaje=msg,
        ciclo_error=primary,
        en_curso=en_curso,
    )


def _prefeeder_maybe_clear_idle_error() -> None:
    """Ya no limpia sola: tras cualquier error el operador debe Reset y luego purga."""
    return


def _prefeeder_raise_fault(sig: tuple[str, ...]) -> None:
    if not sig:
        return
    primary, msg = _machine_fault_message(sig)
    with _STATE_LOCK:
        active = (
            bool(_MACHINE_STATE.get("ciclo_en_curso"))
            or _MACHINE_STATE.get("estado") == "running"
            or bool(_MACHINE_STATE.get("rutina"))
        )
    if active:
        _cycle_fail(primary, mensaje=msg)
        return
    _enter_machine_error(primary, msg, en_curso=False)


def _prefeeder_drain_pf_fault_lines() -> None:
    global _PF_ERROR_LATCH
    if not _serial_port("prefeeder"):
        return
    watched = set(_active_feed_sides())
    try:
        with _prefeeder_serial_lock:
            ser = _prefeeder_serial
            if ser is None or not ser.is_open:
                return
            for _ in range(12):
                line = _read_serial_line(ser, 0.04)
                if not line:
                    break
                try:
                    msg = json.loads(line)
                except (TypeError, json.JSONDecodeError):
                    continue
                if msg.get("op") != "pf_fault":
                    continue
                side = str(msg.get("side") or "").strip().upper()
                if side and side not in watched:
                    continue
                try:
                    fault = int(msg.get("fault") or 0)
                except (TypeError, ValueError):
                    fault = 0
                if fault == 0:
                    continue
                _PF_ERROR_LATCH = True
    except Exception:
        pass


def _prefeeder_watch_once() -> None:
    global _pf_last_fault_sig, _PF_ERROR_LATCH
    if not _serial_port("prefeeder"):
        _pf_last_fault_sig = ()
        _PF_ERROR_LATCH = False
        return
    st = _prefeeder_refresh_cache()
    _prefeeder_drain_pf_fault_lines()
    if not st:
        return
    sig = _prefeeder_fault_sig(st)
    with _STATE_LOCK:
        in_error = _MACHINE_STATE.get("estado") == "error"
    if sig != _pf_last_fault_sig:
        if sig:
            _PF_ERROR_LATCH = True
            _prefeeder_raise_fault(sig)
        else:
            _PF_ERROR_LATCH = False
            if in_error:
                _prefeeder_maybe_clear_idle_error()
        _pf_last_fault_sig = sig
    elif sig and not in_error:
        _PF_ERROR_LATCH = True
        _prefeeder_raise_fault(sig)


def _prefeeder_watch_loop() -> None:
    while True:
        try:
            if _prefeeder_required():
                _prefeeder_watch_once()
        except Exception:
            pass
        time.sleep(0.75)


def _prefeeder_sides_online(status: dict | None) -> bool:
    """Lados activos (feed_sides) reportados online por el maestro."""
    if not isinstance(status, dict):
        return False
    for side in _active_feed_sides():
        block = status.get(side)
        if not isinstance(block, dict) or not block.get("online"):
            return False
    return True


def _prefeeder_stabilize_and_lock() -> tuple[bool, str]:
    """Desbloquea, espera condición segura y vuelve a bloquear listo.

    No hay que esperar solo stable=true del maestro (exige L+R): basta
    online + sin fallas activas de los lados en feed_sides, bloquear y
    confirmar ready. El lado contrario no bloquea.
    """
    if not _prefeeder_required():
        return True, ""
    _prefeeder_set_feed_lock(False)
    deadline = time.time() + _PF_STABILIZE_TIMEOUT_S
    can_lock = False
    while time.time() < deadline:
        st = _prefeeder_refresh_cache()
        if not st:
            time.sleep(0.3)
            continue
        sig = _prefeeder_fault_sig(st)
        if sig:
            _prefeeder_set_feed_lock(True)
            _, msg = _machine_fault_message(sig)
            return False, msg
        # No usar solo stable del maestro: con un solo feed_side exige L+R.
        if _prefeeder_sides_online(st) and not _prefeeder_fault_codes_active(st):
            can_lock = True
            break
        time.sleep(0.3)
    _prefeeder_set_feed_lock(True)
    # Tras bloquear, confirmar idle/ready en los lados activos.
    ready_deadline = time.time() + 8.0
    st = None
    while time.time() < ready_deadline:
        st = _prefeeder_refresh_cache()
        if st and _prefeeder_status_ready(st):
            return True, ""
        time.sleep(0.25)
    if not st:
        return False, error_codes.format_message("E092")
    if not can_lock or not _prefeeder_status_ready(st):
        return False, _prefeeder_not_ready_message(st)
    return True, ""


def _prefeeder_unlock_for_cycle() -> None:
    global _PF_ERROR_LATCH
    if _prefeeder_required():
        _prefeeder_set_feed_lock(False)
        st = _PF_CACHED_STATUS
        if st and not _prefeeder_fault_sig(st):
            _PF_ERROR_LATCH = False


def _prefeeder_lock_after_cycle() -> None:
    if not _prefeeder_required():
        return
    ok, msg = _prefeeder_stabilize_and_lock()
    if not ok:
        _push_event(msg, "error")
        _set_machine_ui(mensaje=msg)


def _prefeeder_lock_after_cycle_async() -> None:
    _prefeeder_process_halt_async()


def _prefeeder_process_halt(*, stabilize: bool = True) -> None:
    """Bloquea alimentación; opcionalmente corre estabilización (misma lógica que Iniciar)."""
    if not _prefeeder_required():
        return
    _prefeeder_stop_now()
    if not stabilize:
        return
    st = _PF_CACHED_STATUS
    if st and _prefeeder_fault_sig(st):
        return
    ok, msg = _prefeeder_stabilize_and_lock()
    _prefeeder_refresh_cache()
    if not ok:
        _push_event(msg, "error")


def _prefeeder_process_halt_async(*, stabilize: bool = True) -> None:
    threading.Thread(
        target=_prefeeder_process_halt, kwargs={"stabilize": stabilize}, daemon=True
    ).start()


def _prefeeder_fault_clear_now() -> bool:
    """Fallas de campo (cilindro/holgura). Sin status ≠ falla activa."""
    if not _prefeeder_required():
        return True
    st = _prefeeder_refresh_cache() or _PF_CACHED_STATUS
    if not st:
        return True
    return not _prefeeder_fault_codes_active(st)


def _error_reset_blocked_message() -> str:
    """Motivo concreto si el reset no puede validar condiciones físicas."""
    if _serial_port("plc"):
        st = _plc_status_get()
        if st is None:
            return error_codes.format_message("E019")
        faults = _plc_fault_codes_ops(st)
        if faults:
            return error_codes.format_message(faults[0])
    if _serial_port("motion"):
        mst = _motion_status_get()
        if mst is None:
            return error_codes.format_message("E043")
        if mst.get("safety") is True:
            return error_codes.format_message("E039")
    if _prefeeder_required():
        st = _prefeeder_refresh_cache() or _PF_CACHED_STATUS
        if st:
            faults = _prefeeder_fault_codes_active(st)
            if faults:
                return error_codes.format_message(faults[0])
    return error_codes.format_message("E046")


def _prefeeder_public_summary() -> dict:
    """Resumen para el HMI (sin consultar COM si no hay caché)."""
    if not _prefeeder_required():
        return {
            "com": False,
            "ready": None,
            "stable": None,
            "feed_lock": None,
            "label": "Sin puerto",
            "tone": "muted",
        }
    st = _PF_CACHED_STATUS
    if not st:
        return {
            "com": True,
            "ready": False,
            "stable": False,
            "feed_lock": True,
            "label": "Sin enlace al maestro",
            "tone": "warn",
        }
    faults = _prefeeder_fault_codes_active(st)
    feed_lock = bool(st.get("feed_lock"))
    ready = _prefeeder_status_ready(st)
    offline = _prefeeder_offline_sides(st)
    stable = _prefeeder_sides_online(st) and not faults
    if faults:
        tone = "warn"
        label = "Bloqueado"
    elif offline:
        tone = "warn"
        label = f"Sin enlace lado {'/'.join(offline)}"
    elif ready:
        tone = "ok"
        label = "Listo (bloqueado)"
    elif not feed_lock:
        tone = "active"
        label = "Desbloqueado — alimentación activa"
    elif not stable:
        tone = "warn"
        label = "Estabilizando…"
    else:
        tone = "warn"
        label = "Bloqueado — pulsa Iniciar"
    return {
        "com": True,
        "ready": ready,
        "stable": stable,
        "feed_lock": feed_lock,
        "label": label,
        "tone": tone,
    }


def _prefeeder_maybe_refresh_ui() -> None:
    """Refuerzo en /api/state si el hilo de vigilancia va retrasado."""
    global _PF_UI_REFRESH_AT
    if not _prefeeder_required():
        return
    now = time.time()
    if now - _PF_UI_REFRESH_AT < 0.6:
        return
    _PF_UI_REFRESH_AT = now
    _prefeeder_watch_once()


def _prefeeder_boot_lock() -> None:
    time.sleep(0.5)
    if _prefeeder_required():
        _prefeeder_set_feed_lock(True)
        _prefeeder_adopt_live_config()


def _build_plc_line(op: str, body: dict) -> dict:
    if op == "valve":
        name = str(body.get("name") or "")
        allowed = _PLC_VALVES + ("holder_encoder", "cutters")
        if name not in allowed:
            raise ValueError("Salida manual no reconocida.")
        if not isinstance(body.get("on"), bool):
            raise ValueError("Falta el estado on de la salida.")
        command = {"op": "valve", "name": name, "on": body["on"]}
        if body.get("ms") is not None:
            if not body["on"] or name not in ("cutter_r", "cutter_l", "cutters"):
                raise ValueError("La duración solo aplica al encendido de cortadores.")
            ms = int(body["ms"])
            if ms < 1 or ms > 5000:
                raise ValueError("La duración del cortador debe estar entre 1 y 5000 ms.")
            command["ms"] = ms
        if body.get("settle_ms") is not None:
            if "ms" in command:
                raise ValueError("No combines ms de cortador con settle_ms.")
            settle = int(body["settle_ms"])
            if settle < 0 or settle > 5000:
                raise ValueError("settle_ms debe estar entre 0 y 5000.")
            if settle > 0:
                command["settle_ms"] = settle
        return command
    if op == "blower_on" or op == "blower_off":
        raise ValueError("El blower está en MCU Motion; usa /api/motion/cmd.")
    if op == "all_off":
        return {"op": "all_off"}
    if op == "reset":
        return {"op": "reset"}
    raise ValueError("Orden PLC no reconocida.")


def _plc_status_from_replies(replies):
    for raw in reversed(replies or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == "status":
            return msg
        # valve ack ya trae GPIO/fallas: sirve como status para la UI.
        if msg.get("op") == "valve" and "holder" in msg:
            return msg
    return None


def _plc_valve_reply(sent: dict) -> dict | None:
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == "valve":
            return msg
    return None


def _plc_accept_valve(
    sent: dict, expected: dict | None = None
) -> tuple[bool, str | None]:
    """Acepta valve ok (+ estado/fallas en la misma reply). Sin status extra."""
    if not sent.get("sent"):
        return False, sent.get("code") or (
            "E019" if _plc_serial_configured() else "E017"
        )
    if _cycle_error_active():
        return False, ""
    reply = _plc_valve_reply(sent)
    if reply is None:
        if _cycle_should_abort():
            return False, ""
        return False, "E010"
    _plc_mirror_apply(reply)
    if not bool(reply.get("ok", True)):
        return False, "E019"
    faults = _plc_fault_codes_ops(reply)
    if faults:
        return False, faults[0]
    if expected:
        return _plc_verify_logical(reply, expected)
    return True, None


def _status_module(replies) -> str:
    """Qué MCU contestó, según los campos de su status."""
    for raw in reversed(replies or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") != "status":
            continue
        if msg.get("mcu") == "motion":
            return "motion"
        if msg.get("mcu") == "prefeeder":
            return "prefeeder"
        if msg.get("mcu") == "andon" or (
            "state" in msg and "cutter_r" not in msg and "feeder_r_ready" not in msg
        ):
            return "andon"
        if "cutter_r" in msg:
            return "plc"
        if "feeder_r_ready" in msg or "lm_servo_on" in msg or "enc_r" in msg:
            return "motion"
    return ""


def _check_module_link(which: str, *, status_timeout_s: float | None = None) -> tuple[bool, str]:
    """True si el COM elegido responde y es el módulo pedido."""
    if which == "motion":
        missing_port, wrong, silent = "E041", "E042", "E043"
        if not _serial_port("motion"):
            return False, missing_port
        # Ciclo/arranque: hasta 90 s (init CAN). Probar en UI: timeout corto.
        motion_timeout = 90.0 if status_timeout_s is None else float(status_timeout_s)
        sent = _send_motion_serial(
            ['{"op":"status"}'],
            line_timeout_s=motion_timeout,
            reply_until_op="status",
        )
    elif which == "plc":
        missing_port, wrong, silent = "E017", "E018", "E019"
        if not _serial_port("plc"):
            return False, missing_port
        sent = _send_plc_serial(['{"op":"status"}'])
    elif which == "andon":
        missing_port, wrong, silent = "E070", "E071", "E072"
        if not _serial_port("andon"):
            return False, missing_port
        sent = _send_andon_serial(['{"op":"status"}'])
    elif which == "prefeeder":
        missing_port, wrong, silent = "E090", "E091", "E092"
        if not _serial_port("prefeeder"):
            return False, missing_port
        # Misma recuperación que _prefeeder_status_get: reopen si el Monitor
        # dejó el COM en un estado donde write “pasa” pero no hay respuesta.
        st = _prefeeder_status_get()
        if st and st.get("mcu") == "prefeeder":
            return True, ""
        if st:
            return False, wrong
        return False, silent
    else:
        return False, "E070"
    if not sent.get("sent"):
        return False, silent
    found = _status_module(sent.get("replies"))
    if found == which:
        return True, ""
    if found:
        return False, wrong
    return False, silent


def _links_ready_for_cycle() -> tuple[bool, str]:
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR: dejar solo ("motion", "plc", "andon") ---
    modules: tuple[str, ...] = (
        ("motion", "plc")
        if _andon_skipped_effective()
        else ("motion", "plc", "andon")
    )
    codes: list[str] = []
    for which in modules:
        ok, code = _check_module_link(which)
        if not ok:
            codes.append(code)
    if codes:
        _primary, msg = _machine_fault_message(tuple(codes))
        return False, msg
    return True, ""


def _plc_serial_configured() -> bool:
    return bool(_serial_port("plc"))


def _plc_status_get() -> dict | None:
    """GET status al MCU PLC (una consulta, sin polling en la UI)."""
    sent = _send_plc_serial(['{"op":"status"}'])
    if not sent.get("sent"):
        return None
    status = _plc_status_from_replies(sent.get("replies"))
    _plc_mirror_apply(status)
    return status


def _plc_status_after_command(sent: dict) -> dict | None:
    """Tras POST al PLC: status en la reply o GET adicional."""
    status = _plc_status_from_replies(sent.get("replies"))
    if status is not None:
        return status
    if not sent.get("sent"):
        return None
    return _plc_status_get()


def _plc_wait_ready(timeout_s: float = 3.0) -> dict | None:
    """Obtiene el estado actual; las salidas por nivel no tienen cola pendiente."""
    return _plc_status_get()


def _motion_blower_send(payload: dict, *, push_log: bool = False) -> dict:
    """Blower Dual: MCU Motion (GPIO)."""
    global _MOTION_BLOWER_ON
    op = str(payload.get("op") or "")
    try:
        tune = _load_motion_tune()
        _tune, lines = _build_motion_lines(op, tune, payload)
        line = lines[0]
    except (TypeError, ValueError, OSError, json.JSONDecodeError):
        line = _compact(payload)
    sent = _send_motion_serial([line], line_timeout_s=2.0)
    sent["last_payload"] = payload
    if not sent.get("sent"):
        sent["code"] = "E043" if _serial_port("motion") else "E041"
        if push_log:
            _push_event(error_codes.format_message(sent["code"]), "error")
    else:
        if push_log:
            _push_event(f"[Motion] {line}")
        for raw in sent.get("replies") or []:
            try:
                msg = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                continue
            if msg.get("op") in ("blower_on", "blower_off", "status", "release_part"):
                if "blower" in msg:
                    _MOTION_BLOWER_ON = bool(msg["blower"])
                elif msg.get("op") == "blower_on":
                    _MOTION_BLOWER_ON = True
                elif msg.get("op") == "blower_off":
                    _MOTION_BLOWER_ON = False
    return sent


def _plc_send_payload(payload: dict, *, push_log: bool = False) -> dict:
    """Envía una orden al MCU PLC."""
    op = str(payload.get("op") or "")
    if op in ("blower_on", "blower_off"):
        return _motion_blower_send(payload, push_log=push_log)
    try:
        if op in ("valve", "all_off", "reset"):
            line = _compact(_build_plc_line(op, payload))
        else:
            line = _compact(payload)
    except (TypeError, ValueError):
        line = _compact(payload)
    sent = _send_plc_serial([line])
    sent["last_payload"] = payload
    if not sent.get("sent"):
        sent["code"] = "E019" if _serial_port("plc") else "E017"
        if push_log:
            _push_event(error_codes.format_message(sent["code"]), "error")
    elif push_log:
        _push_event(f"[PLC] {line}")
    return sent


def _plc_verify_logical(st: dict, expected: dict) -> tuple[bool, str | None]:
    for key, want in expected.items():
        if st.get(key) is not want:
            if key == "holder":
                return False, "E011" if want else "E014"
            if key == "encoder":
                return False, "E012" if want else "E015"
            if key == "gripper":
                return False, "E013"
            return False, "E010"
    return True, None


def _plc_fault_codes_ops(status: dict | None) -> list[str]:
    """Fallos PLC que bloquean operación."""
    return error_codes.plc_fault_codes(status)


def _plc_confirm(expected: dict, timeout_s: float = 3.0) -> tuple[bool, str | None]:
    """Confirma salidas lógicas. Sin COM no se da por bueno."""
    if not _plc_serial_configured():
        return False, "E017"
    st = _plc_wait_ready(timeout_s)
    if st is None:
        if _cycle_should_stop() and _cycle_thread_alive():
            return False, ""
        return False, "E010"
    faults = _plc_fault_codes_ops(st)
    if faults:
        return False, faults[0]
    return _plc_verify_logical(st, expected)


def _plc_active_errors(status: dict | None) -> list[dict]:
    if not status:
        return []
    items = []
    for code in _plc_fault_codes_ops(status):
        items.append({"code": code, "message": error_codes.format_message(code)})
    return items


def _send_motion_serial(
    lines,
    line_timeout_s=None,
    reply_until_op: str | None = None,
    nonblocking: bool = False,
    inject=None,
    on_line=None,
):
    port = _serial_port("motion")
    if not port:
        return {
            "sent": False,
            "reason": "Sin puerto de Motion. Elígelo en Parámetros, pestaña Conexión.",
            "replies": [],
        }
    try:
        import serial
    except ImportError:
        return {
            "sent": False,
            "reason": "Falta pyserial para abrir el puerto.",
            "replies": [],
        }

    replies = []
    global _motion_serial
    acquired = _serial_lock.acquire(blocking=not nonblocking)
    if not acquired:
        return {"sent": False, "reason": "Motion ocupado.", "replies": [], "busy": True}
    try:
        if (
            _motion_serial is None
            or not _motion_serial.is_open
            or getattr(_motion_serial, "port", None) != port
        ):
            if _motion_serial is not None:
                try:
                    _motion_serial.close()
                except Exception:
                    pass
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = int(os.environ.get("MOTION_SERIAL_BAUD", "115200"))
            ser.timeout = 0.05
            ser.write_timeout = 2
            ser.dtr = False
            ser.rts = False
            ser.open()
            time.sleep(0.05)
            _serial_rx_flush(ser)
            _motion_serial = ser
        ser = _motion_serial
        for index, line in enumerate(lines):
            trace_motion = '"op":"status"' not in line
            if trace_motion:
                print(f"[Motion TX] {line}", flush=True)
            with _motion_write_lock:
                ser.write((line + "\n").encode("ascii", "replace"))
                ser.flush()
            try:
                expected_op = str(json.loads(line).get("op") or "")
            except (TypeError, json.JSONDecodeError):
                expected_op = ""
            if reply_until_op and index == len(lines) - 1:
                expected_op = reply_until_op
            wait_s = line_timeout_s if line_timeout_s is not None else _reply_wait_s(line)
            if expected_op:
                replies.extend(
                    _read_lines_until_op(
                        ser,
                        expected_op,
                        wait_s,
                        trace_prefix="[Motion RX]" if trace_motion else None,
                        inject=inject if index == len(lines) - 1 else None,
                        on_line=on_line if index == len(lines) - 1 else None,
                    )
                )
            else:
                reply = _read_serial_line(ser, wait_s)
                if reply:
                    replies.append(reply)
                    if trace_motion:
                        print(f"[Motion RX] {reply}", flush=True)
    except Exception as exc:
        # Tras Monitor Serial / USB, el handle puede quedar abierto pero muerto.
        try:
            _invalidate_serial_handle(_motion_serial)
        except Exception:
            pass
        _motion_serial = None
        return {"sent": False, "reason": str(exc), "replies": replies}
    finally:
        if _serial_lock.locked():
            _serial_lock.release()
    return {"sent": True, "reason": "", "replies": replies}


def _send_motion_urgent(lines):
    """Escribe paros sin esperar el lock ocupado por una orden larga."""
    global _motion_serial
    if not _serial_lock.locked():
        return _send_motion_serial(lines, line_timeout_s=2.0)
    ser = _motion_serial
    if ser is None or not ser.is_open:
        return {"sent": False, "reason": "Motion ocupado sin puerto abierto.", "replies": []}
    try:
        with _motion_write_lock:
            for line in lines:
                ser.write((line + "\n").encode("ascii", "replace"))
            ser.flush()
    except Exception as exc:
        return {"sent": False, "reason": str(exc), "replies": []}
    return {"sent": True, "reason": "", "replies": [], "urgent": True}


def _motion_data_flat(data: dict) -> dict:
    return {
        "op": "set_data",
        "target_length_mm": data.get("target_length_mm"),
        "tolerance_mm": data.get("tolerance_mm", 2.0),
        "modelo_id": data.get("modelo_id") or "",
        "measure_comp_mm": data.get("measure_comp_mm", 0.0),
    }


def _deep_merge_motion_tune(base: dict, patch: dict) -> None:
    for key, val in patch.items():
        if isinstance(val, dict) and isinstance(base.get(key), dict):
            _deep_merge_motion_tune(base[key], val)
        else:
            base[key] = val


def _normalize_modelo_entry(raw) -> dict | None:
    """Unifica BOM {item, desc, longitud} y legado {modelo, longitud_mm}."""
    if not isinstance(raw, dict):
        return None
    modelo = str(raw.get("item") or raw.get("modelo") or "").strip()
    if not modelo:
        return None
    desc = str(raw.get("desc") or "").strip()
    longitud = raw.get("longitud")
    if longitud is None:
        longitud = raw.get("longitud_mm")
    try:
        longitud_mm = float(longitud) if longitud is not None else None
    except (TypeError, ValueError):
        longitud_mm = None
    return {"modelo": modelo, "desc": desc, "longitud_mm": longitud_mm}


def _load_modelos():
    """Lee catálogo BOM si existe; si no, modelos.json legado."""
    path = BOM_LONGITUDES_PATH if BOM_LONGITUDES_PATH.exists() else MODELOS_PATH
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, list):
        raw_list = data
    elif isinstance(data, dict):
        raw_list = data.get("modelos") or data.get("items") or []
    else:
        raw_list = []
    out = []
    for raw in raw_list:
        entry = _normalize_modelo_entry(raw)
        if entry:
            out.append(entry)
    return out


def _find_modelo(modelo_id: str):
    mid = str(modelo_id or "").strip()
    if not mid:
        return None
    for item in _load_modelos():
        if item.get("modelo") == mid:
            return item
    return None


def _apply_modelo_selection(modelo_id: str, longitud_mm=None, desc: str | None = None) -> tuple[bool, str]:
    """Aplica modelo/longitud al estado. longitud_mm permite entrada manual o override."""
    mid = str(modelo_id or "").strip()
    entry = _find_modelo(mid) if mid else None

    length = None
    if longitud_mm is not None and str(longitud_mm).strip() != "":
        try:
            length = float(longitud_mm)
        except (TypeError, ValueError):
            return False, "La longitud tiene que ser un número."
        if length <= 0:
            return False, "La longitud tiene que ser mayor que 0."

    if entry:
        if length is None:
            length = entry.get("longitud_mm")
        use_desc = desc if desc is not None else (entry.get("desc") or "")
        _MACHINE_STATE["numero_modelo"] = entry["modelo"]
        _MACHINE_STATE["modelo_desc"] = str(use_desc or "").strip()
        _MACHINE_STATE["longitud_modelo_mm"] = length
        _MOTION_DATA["modelo_id"] = entry["modelo"]
        _MOTION_DATA["target_length_mm"] = length
        return True, ""

    if not mid and length is None:
        _MACHINE_STATE["numero_modelo"] = ""
        _MACHINE_STATE["modelo_desc"] = ""
        _MACHINE_STATE["longitud_modelo_mm"] = None
        _MOTION_DATA["modelo_id"] = ""
        _MOTION_DATA["target_length_mm"] = None
        return True, ""

    if length is None:
        return False, "Modelo no encontrado. Escribe también la longitud o elige del catálogo."

    _MACHINE_STATE["numero_modelo"] = mid
    _MACHINE_STATE["modelo_desc"] = str(desc or "").strip()
    _MACHINE_STATE["longitud_modelo_mm"] = length
    _MOTION_DATA["modelo_id"] = mid or f"L{length:g}"
    _MOTION_DATA["target_length_mm"] = length
    return True, ""


def _load_cycle_params():
    params = dict(_DEFAULT_CYCLE_PARAMS)
    if not CYCLE_PATH.exists():
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        CYCLE_PATH.write_text(
            json.dumps(params, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return params
    try:
        with CYCLE_PATH.open(encoding="utf-8") as f:
            raw = json.load(f)
    except (OSError, json.JSONDecodeError):
        return params
    if isinstance(raw, dict):
        for key in params:
            if key in raw:
                params[key] = raw[key]
        for key in _HMI_CYCLE_EXTRA_KEYS:
            if key in raw:
                params[key] = raw[key]
    return params


def _save_hmi_cycle_extras() -> None:
    """Cantidad, modelo y compensaciones R/L en cycle.json."""
    try:
        params = _load_cycle_params()
        comps = _cut_comp_params_of(params)
        params["cut_comp_mode"] = True
        params["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
        params["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
        params["cut_offset_mm"] = 0.0
        try:
            params["cantidad"] = max(1, int(_MACHINE_STATE.get("cantidad") or 1))
        except (TypeError, ValueError):
            params["cantidad"] = 1
        params["numero_modelo"] = str(_MACHINE_STATE.get("numero_modelo") or "").strip()
        params["modelo_desc"] = str(_MACHINE_STATE.get("modelo_desc") or "").strip()
        length = _MACHINE_STATE.get("longitud_modelo_mm")
        try:
            params["longitud_modelo_mm"] = float(length) if length is not None else None
        except (TypeError, ValueError):
            params["longitud_modelo_mm"] = None
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        CYCLE_PATH.write_text(
            json.dumps(params, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    except OSError:
        pass


def _hydrate_hmi_from_cycle() -> None:
    """Restaura pantalla principal desde cycle.json al arrancar app.py."""
    params = _load_cycle_params()
    try:
        qty = int(params.get("cantidad", 1))
    except (TypeError, ValueError):
        qty = 1
    _MACHINE_STATE["cantidad"] = max(1, qty)
    comps = _cut_comp_params_of(params)
    params["cut_comp_mode"] = True
    params["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
    params["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
    params["cut_offset_mm"] = 0.0
    _MACHINE_STATE["compensacion_medicion_mm"] = 0.0
    _MOTION_DATA["measure_comp_mm"] = 0.0
    _MOTION_DATA["tolerance_mm"] = _num(params.get("tolerance_mm"), 2.0)
    modelo_id = str(params.get("numero_modelo") or "").strip()
    saved_len = params.get("longitud_modelo_mm")
    saved_desc = str(params.get("modelo_desc") or "").strip()
    if not modelo_id and saved_len is None:
        return
    try:
        ok, _err = _apply_modelo_selection(modelo_id, saved_len, saved_desc or None)
    except (OSError, json.JSONDecodeError):
        ok = False
    if not ok and modelo_id:
        # Catálogo no disponible: conserva lo persistido si hay longitud.
        try:
            length = float(saved_len) if saved_len is not None else None
        except (TypeError, ValueError):
            length = None
        if length is None or length <= 0:
            return
        _MACHINE_STATE["numero_modelo"] = modelo_id
        _MACHINE_STATE["modelo_desc"] = saved_desc
        _MACHINE_STATE["longitud_modelo_mm"] = length
        _MOTION_DATA["modelo_id"] = modelo_id
        _MOTION_DATA["target_length_mm"] = length


def _cycle_step_meta(step_id: str):
    for i, step in enumerate(CYCLE_STEPS):
        if step["id"] == step_id:
            return i, step
    return -1, None


def _cycle_progress_pct(step_index: int) -> int:
    n = len(CYCLE_STEPS)
    if n <= 0:
        return 0
    return int(round(min(100, max(0, (step_index / n) * 100))))


def _lot_progress_pct(step_index: int | None = None) -> int:
    """Avance del lote por piezas R+L / qty_total. Fracción del paso actual si hay pieza en curso."""
    total = int(_LOT.get("qty_total") or 0)
    if total <= 0:
        total = int(_LOT.get("cycles_total") or 0) * _lot_piece_mult()
    if total <= 0:
        if step_index is not None:
            return _cycle_progress_pct(int(step_index))
        return 0
    done = _lot_pieces_done_total()
    if done >= total or _lot_targets_met():
        return 100
    frac = 0.0
    if (_LOT.get("active") or _LOT.get("tray_wait")) and step_index is not None:
        n = len(CYCLE_STEPS)
        if n > 0:
            # Aporta la fracción del ciclo actual según lados en curso.
            credit_n = len(_lot_credit_sides_now())
            frac = min(1.0, max(0.0, float(step_index) / float(n))) * max(credit_n, 1)
    return int(round(min(100, max(0, ((done + frac) / total) * 100))))


def _now_hms() -> str:
    return time.strftime("%H:%M:%S")


_EVENTS_REV = 0


def _push_event(texto: str, tipo: str = "info") -> None:
    global _EVENTS_REV
    entry = {"texto": texto, "tipo": tipo, "hora": _now_hms()}
    with _STATE_LOCK:
        ev = _MACHINE_STATE.setdefault("eventos", [])
        if ev and tipo == "error" and ev[0].get("tipo") == "error" and ev[0].get("texto") == texto:
            return
        ev.insert(0, entry)
        del ev[300:]
        _EVENTS_REV += 1


def _clear_event_log() -> int:
    global _EVENTS_REV
    with _STATE_LOCK:
        _MACHINE_STATE["eventos"] = []
        _EVENTS_REV += 1
        return _EVENTS_REV


def _andon_desired(machine: dict) -> str:
    code = str(machine.get("ciclo_error") or "")
    if code in ("E033", "E034"):
        return "falta_material"
    if code or machine.get("estado") == "error":
        return "error"
    if machine.get("estado") == "running" or (
        machine.get("ciclo_en_curso") and machine.get("estado") == "paused"
    ):
        return "trabajando"
    if (
        not machine.get("ciclo_en_curso")
        and int(machine.get("ciclo_paso_indice") or 0) >= len(CYCLE_STEPS)
    ):
        return "terminado"
    return "simple"


def _andon_publish_from_state() -> None:
    global _andon_state_sent, _andon_attempted
    if not _serial_port("andon"):
        return
    with _STATE_LOCK:
        desired = _andon_desired(_MACHINE_STATE)
    with _andon_serial_lock:
        if desired == _andon_state_sent or desired == _andon_attempted:
            return
        _andon_attempted = desired
    sent = _send_andon_serial([_compact({"op": "set_state", "state": desired})])
    accepted = False
    if sent.get("sent"):
        for raw in sent.get("replies") or []:
            try:
                msg = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                continue
            if (
                msg.get("op") == "set_state"
                and msg.get("ok") is True
                and msg.get("state") == desired
            ):
                accepted = True
                break
    if accepted:
        _andon_state_sent = desired
        return
    _push_event(error_codes.format_message("E072"), "error")


def _public_machine_state() -> dict:
    estimado_s, corriendo_s = _lot_times_now()
    with _STATE_LOCK:
        state = dict(_MACHINE_STATE)
        state["checklist"] = dict(_MACHINE_STATE.get("checklist") or {})
        state["checklist_bypass"] = _checklist_bypassed()
        state["eventos"] = list(_MACHINE_STATE.get("eventos") or [])
        state["eventos_rev"] = _EVENTS_REV
        state["ciclo_feed"] = dict(_MACHINE_STATE.get("ciclo_feed") or {})
        state["charola_llena"] = bool(_LOT["tray_wait"])
        state["charola_ciclos"] = int(_LOT["deposits_per_tray"] or 0) if _LOT["tray_wait"] else 0
        state["lote_activo"] = bool(_LOT["active"] or _LOT["tray_wait"])
        done_c = int(_LOT["cycles_done"] or 0)
        total_c = int(_LOT["cycles_total"] or 0)
        state["lote_ciclos_hechos"] = done_c
        state["lote_ciclos_total"] = total_c
        state["lote_lados"] = _LOT["sides"]
        state["lote_cycle_sides"] = _lot_cycle_sides_now() if _LOT.get("active") else _LOT.get("sides")
        state["piezas_r"] = int(_LOT.get("pieces_r") or 0)
        state["piezas_l"] = int(_LOT.get("pieces_l") or 0)
        state["ciclos_mant_r"] = int(_MACHINE_STATE.get("ciclos_mant_r") or 0)
        state["ciclos_mant_l"] = int(_MACHINE_STATE.get("ciclos_mant_l") or 0)
        state["piezas_target_r"] = int(_LOT.get("target_r") or 0)
        state["piezas_target_l"] = int(_LOT.get("target_l") or 0)
        qty_total = int(_LOT.get("qty_total") or 0)
        lote_vivo = bool(_LOT["active"] or _LOT["tray_wait"])
        pieces_done = _lot_pieces_done_total()
        lote_terminado = (
            (not lote_vivo)
            and qty_total > 0
            and pieces_done >= qty_total
            and int(_LOT.get("pieces_r") or 0) >= int(_LOT.get("target_r") or 0)
            and int(_LOT.get("pieces_l") or 0) >= int(_LOT.get("target_l") or 0)
        )
        if lote_vivo or lote_terminado:
            state["piezas_producidas"] = pieces_done
            state["piezas_total"] = qty_total or int(state.get("cantidad") or 0)
        else:
            state["piezas_producidas"] = 0
            state["piezas_total"] = int(state.get("cantidad") or 0)
        if qty_total > 0 or total_c > 0:
            state["progreso_pct"] = _lot_progress_pct(state.get("ciclo_paso_indice"))
        if estimado_s is not None:
            state["tiempo_estimado_s"] = estimado_s
        elif lote_terminado:
            state["tiempo_estimado_s"] = 0
        elif not (lote_vivo or state.get("ciclo_en_curso")):
            state["tiempo_estimado_s"] = None
        if lote_vivo or state.get("ciclo_en_curso"):
            state["tiempo_corriendo_s"] = corriendo_s
        params = _load_cycle_params()
        params = dict(params)
        comps = _cut_comp_params_of(params)
        state["feed_sides"] = _feed_sides_of(params)
        state["cut_comp_mode"] = True
        state["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
        state["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
        length = _num(state.get("longitud_modelo_mm"), 0.0)
        if length > 0:
            plan = _build_cut_plan(length, params, state["feed_sides"])
            state["cut_base_mm"] = plan["base_mm"]
            state["cut_mm_r"] = plan["base_mm"] + comps["cut_comp_r_mm"]
            state["cut_mm_l"] = plan["base_mm"] + comps["cut_comp_l_mm"]
            state["cut_mm_last"] = plan["last_mm"]
            state["cut_comp_sequential"] = bool(plan["sequential"])
        else:
            state["cut_base_mm"] = None
            state["cut_mm_r"] = None
            state["cut_mm_l"] = None
            state["cut_mm_last"] = None
            state["cut_comp_sequential"] = False
        state["ciclo_paso_a_paso"] = bool(_CYCLE_CTRL.get("step_mode"))
        state["recovery"] = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
        act = _MACHINE_STATE.get("actuators") or {}
        state["actuators"] = {
            "state": str(act.get("state") or "offline"),
            "stage": str(act.get("stage") or ""),
        }
    state["checklist_log"] = _checklist_log_read()
    state["prefeeder"] = _prefeeder_public_summary()
    return state


def _set_machine_ui(
    *,
    estado=None,
    estado_label=None,
    mensaje=None,
    step_id=None,
    step_index=None,
    paralelo_id=None,
    en_curso=None,
    ciclo_error=None,
    progreso_pct=None,
    tiempo_corriendo_s=None,
    tiempo_ciclo_s=None,
    feed=None,
) -> None:
    with _STATE_LOCK:
        if estado is not None:
            _MACHINE_STATE["estado"] = estado
        if estado_label is not None:
            _MACHINE_STATE["estado_label"] = estado_label
        if mensaje is not None:
            _MACHINE_STATE["mensaje"] = mensaje
        if step_id is not None:
            _MACHINE_STATE["ciclo_paso_id"] = step_id
            _, meta = _cycle_step_meta(step_id)
            _MACHINE_STATE["ciclo_paso_label"] = meta["label"] if meta else step_id
        if step_index is not None:
            _MACHINE_STATE["ciclo_paso_indice"] = step_index
            if progreso_pct is None:
                if int(_LOT.get("cycles_total") or 0) > 0:
                    _MACHINE_STATE["progreso_pct"] = _lot_progress_pct(step_index)
                else:
                    _MACHINE_STATE["progreso_pct"] = _cycle_progress_pct(step_index)
        if paralelo_id is not None:
            _MACHINE_STATE["ciclo_paralelo_id"] = paralelo_id
        if progreso_pct is not None:
            _MACHINE_STATE["progreso_pct"] = progreso_pct
        if en_curso is not None:
            _MACHINE_STATE["ciclo_en_curso"] = en_curso
        if ciclo_error is not None:
            _MACHINE_STATE["ciclo_error"] = ciclo_error
        if tiempo_corriendo_s is not None:
            _MACHINE_STATE["tiempo_corriendo_s"] = tiempo_corriendo_s
        if tiempo_ciclo_s is not None:
            _MACHINE_STATE["tiempo_ciclo_s"] = tiempo_ciclo_s
        if feed is not None:
            _MACHINE_STATE["ciclo_feed"] = feed
    _andon_publish_from_state()


def _cycle_should_stop() -> bool:
    return _CYCLE_CTRL["stop"].is_set()


def _cycle_error_active() -> bool:
    """Error de ciclo latched. Sigue activo aunque el hilo ya haya muerto tras el fallo."""
    with _STATE_LOCK:
        has_err = bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
        en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
    if not has_err:
        return False
    # Idle con fallo de módulo (sin lote) no debe abortar envíos ajenos al ciclo.
    return en_curso or bool(_LOT.get("active") or _LOT.get("tray_wait")) or _cycle_thread_alive()


def _lot_piece_mult(sides: str | None = None) -> int:
    s = (sides if sides is not None else _LOT.get("sides") or "R").upper()
    return 2 if s in ("LR", "RL", "BOTH", "AMBOS") else 1


def _lot_avg_cycle_s() -> float | None:
    if _PROD_CADENCE_SAMPLES:
        return sum(_PROD_CADENCE_SAMPLES) / len(_PROD_CADENCE_SAMPLES)
    if _LAST_PIECE_S and _LAST_PIECE_S > 0:
        return float(_LAST_PIECE_S)
    return None


def _lot_times_now() -> tuple[float | None, float]:
    """(tiempo_estimado_s restante del lote, tiempo_corriendo_s del lote o de la pieza)."""
    now = time.time()
    with _STATE_LOCK:
        estado = _MACHINE_STATE.get("estado")
        en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
        frozen = float(_MACHINE_STATE.get("tiempo_corriendo_s") or 0)
    lote_vivo = bool(_LOT.get("active") or _LOT.get("tray_wait"))
    corriendo = 0.0
    if lote_vivo and _LOT_T0 is not None:
        # Reloj del lote completo (no se reinicia por pieza).
        corriendo = max(0.0, now - _LOT_T0)
    elif en_curso:
        if estado in ("paused", "error"):
            # Congelado en pausa/error; no sigue sumando.
            if _PIECE_TICK0 is not None and frozen <= 0:
                corriendo = max(0.0, now - _PIECE_TICK0)
            else:
                corriendo = max(0.0, frozen)
        elif _PIECE_TICK0 is not None:
            corriendo = max(0.0, now - _PIECE_TICK0)
    estimado = None
    if lote_vivo:
        done = _lot_pieces_done_total()
        qty = int(_LOT.get("qty_total") or 0)
        need_r = max(0, int(_LOT.get("target_r") or 0) - int(_LOT.get("pieces_r") or 0))
        need_l = max(0, int(_LOT.get("target_l") or 0) - int(_LOT.get("pieces_l") or 0))
        # Ciclos restantes ≈ max(need_r, need_l); cada ciclo suma +1/+2 piezas según lados.
        remaining_cycles = max(need_r, need_l)
        remaining_pcs = max(0, qty - done) if qty > 0 else remaining_cycles
        if remaining_cycles <= 0 or remaining_pcs <= 0:
            estimado = 0.0
        else:
            avg = _lot_avg_cycle_s()
            cadence_est = None
            if avg is not None and avg > 0:
                piece_elapsed = 0.0
                if _PIECE_TICK0 is not None and en_curso:
                    piece_elapsed = max(0.0, now - _PIECE_TICK0)
                if en_curso:
                    rest_current = max(0.0, avg - piece_elapsed)
                    cadence_est = rest_current + max(0, remaining_cycles - 1) * avg
                else:
                    cadence_est = remaining_cycles * avg
            # Throughput real del lote (piezas / tiempo transcurrido): evita
            # subestimar cuando la cadencia media es más rápida que el ritmo real.
            throughput_est = None
            if done >= 2 and corriendo > 1.0:
                rate = done / corriendo
                if rate > 0:
                    throughput_est = remaining_pcs / rate
            if cadence_est is not None and throughput_est is not None:
                estimado = max(cadence_est, throughput_est)
            elif throughput_est is not None:
                estimado = throughput_est
            else:
                estimado = cadence_est
    return (
        round(estimado, 1) if estimado is not None else None,
        round(corriendo, 1),
    )


def _cycle_worker_is_current(gen: int) -> bool:
    """False si este hilo ya no es el worker oficial (Stop / nuevo Start / huérfano)."""
    return int(_CYCLE_CTRL.get("gen") or 0) == int(gen)


def _cycle_should_abort() -> bool:
    if _cycle_should_stop() or _cycle_error_active():
        return True
    # Hilo de ciclo (o auxiliar) marcado con gen: si ya no es el oficial, abortar.
    my_gen = getattr(_CYCLE_WORKER_TLS, "gen", None)
    if my_gen is not None and not _cycle_worker_is_current(my_gen):
        return True
    return False


def _cycle_motion_halt_for_error() -> None:
    """Para Motion y blower. No toca pinzas: si hay pieza sujeta, se retiene."""
    if _serial_port("motion"):
        _send_motion_urgent(
            ['{"op":"feeder_stop"}', '{"op":"lmguide_stop"}', '{"op":"blower_off"}']
        )


def _gripper_holding_set(holding: bool) -> None:
    global _GRIPPER_HOLDING
    _GRIPPER_HOLDING = bool(holding)


def _gripper_holding() -> bool:
    return bool(_GRIPPER_HOLDING)


def _cycle_thread_alive() -> bool:
    thread = _CYCLE_CTRL["thread"]
    return thread is not None and thread.is_alive()


def _cycle_spawn_worker(start_index: int, *, locked: bool = False) -> tuple[bool, str]:
    """Arranca exactamente un worker. Rechaza si ya hay uno vivo."""

    def _do() -> tuple[bool, str]:
        if _cycle_thread_alive():
            return False, "Hay un ciclo en ejecución."
        _CYCLE_CTRL["gen"] = int(_CYCLE_CTRL.get("gen") or 0) + 1
        gen = int(_CYCLE_CTRL["gen"])
        th = threading.Thread(
            target=_cycle_worker,
            args=(start_index, gen),
            daemon=True,
            name=f"cycle-worker-{gen}",
        )
        _CYCLE_CTRL["thread"] = th
        th.start()
        return True, ""

    if locked:
        return _do()
    with _CYCLE_SPAWN_LOCK:
        return _do()


def _motion_owner_busy() -> str:
    cycle = _CYCLE_CTRL["thread"]
    if cycle is not None and cycle.is_alive():
        return "Hay un ciclo en ejecución."
    routine = _ROUTINE["thread"]
    if routine is not None and routine.is_alive():
        return "Hay una rutina en curso."
    return ""


def _cycle_wait_if_paused() -> bool:
    """True si hay Stop brusco o error de ciclo (abortar de inmediato)."""
    return _cycle_should_abort()


def _cycle_sleep_ms(ms: int) -> bool:
    """True si se completó la espera; False si hubo Stop o error."""
    deadline = time.time() + (max(0, int(ms)) / 1000.0)
    while time.time() < deadline:
        if _cycle_wait_if_paused():
            return False
        time.sleep(0.02)
    return not _cycle_should_abort()


def _cycle_plc_send(payload: dict):
    return _plc_send_payload(payload, push_log=True)


def _cycle_motion_send(payload: dict, timeout_s: float = 2.0, inject=None, on_line=None):
    line = _compact(payload)
    sent = _send_motion_serial(
        [line], line_timeout_s=timeout_s, inject=inject, on_line=on_line
    )
    sent["last_payload"] = payload
    if sent.get("sent"):
        _push_event(f"[Motion] {line}")
    else:
        sent["code"] = "E043" if _serial_port("motion") else "E041"
        _push_event(error_codes.format_message(sent["code"]), "error")
    return sent


def _serial_reply_ok(sent: dict, op: str, fallback: str) -> bool:
    """True si hubo envío y la última respuesta JSON con ese op tiene ok (por defecto True)."""
    # Si el ciclo ya fue pausado por otro hilo, no reportar falso E043 de COM.
    if _cycle_error_active():
        return False
    if not sent.get("sent"):
        if sent.get("busy"):
            return False
        _cycle_fail(sent.get("code") or fallback)
        return False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            if not bool(msg.get("ok", True)):
                _cycle_fail(
                    "E044"
                    if op
                    in (
                        "lmguide_move",
                        "lmguide_home",
                        "set_tune",
                        "set_data",
                        "release_part",
                    )
                    else fallback
                )
                return False
            return True
    # Espera cortada por Stop/error: no confundir con COM mudo.
    if _cycle_should_abort():
        return False
    _cycle_fail(fallback)
    return False


def _motion_status_get() -> dict | None:
    sent = _send_motion_serial(
        ['{"op":"status"}'],
        line_timeout_s=_MOTION_STATUS_TIMEOUT_S,
        reply_until_op="status",
    )
    if not sent.get("sent"):
        return None
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == "status":
            return msg
    return None


def _motion_ready_for_cycle() -> tuple[bool, str]:
    status = _motion_status_get()
    if status is None:
        return False, error_codes.format_message("E043")
    if status.get("safety") is True:
        return False, error_codes.format_message("E039")
    if status.get("feeder_r_ready") is not True or status.get("feeder_l_ready") is not True:
        return False, error_codes.format_message("E032")
    _actuators_sync_from_motion_status(status)
    # Si el LM Guide no está en home, _run_one_piece lo manda a PUU 0 al arranque
    # fresco y continúa; no se bloquea Start por posición.
    return True, ""


def _plc_cmd_ok(sent: dict, fallback: str) -> bool:
    op = str((sent.get("last_payload") or {}).get("op") or "")
    if not op:
        for raw in reversed(sent.get("replies") or []):
            try:
                msg = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                continue
            if isinstance(msg, dict) and msg.get("op"):
                op = str(msg.get("op"))
                break
    if not op:
        if sent.get("sent"):
            return True
        _cycle_fail(sent.get("code") or fallback)
        return False
    return _serial_reply_ok(sent, op, fallback)


def _motion_cmd_serial_ok(sent: dict, fallback: str) -> bool:
    op = str((sent.get("last_payload") or {}).get("op") or "")
    if not op:
        return _sent_ok(sent, fallback)
    return _serial_reply_ok(sent, op, fallback)


def _sent_ok(sent: dict, fallback: str) -> bool:
    if sent.get("sent"):
        return True
    _cycle_fail(sent.get("code") or fallback)
    return False


def _cycle_blower_length_mm() -> float:
    # Reglas §4.8: duración del blower = longitud programada de la pieza.
    # El encoder OM solo reporta counts; no se deriva mm para el soplo.
    prog = _num(_MOTION_DATA.get("target_length_mm"), 0.0)
    if prog > 0:
        return prog
    return _num(_PIECE_BLOWER_MM, 0.0)


def _cycle_blower_duration_ms(params: dict) -> int:
    length_f = _cycle_blower_length_mm()
    per_mm = float(params.get("blower_ms_per_mm") or 8.0)
    minimum = int(params.get("blower_ms_min") or 200)
    ms = int(round(length_f * per_mm))
    return max(minimum, min(ms, 300000))


def _cycle_enter_step(step_id: str, step_index: int) -> None:
    labels = {
        "1": ("running", "Ciclo en marcha", "Secuencia automática iniciada."),
        "2": ("running", "Ciclo en marcha", "Activando holder y encoder."),
        "3": ("running", "Ciclo en marcha", "Alimentación L/R — láser y encoder."),
        "4": ("running", "Ciclo en marcha", "Pinzas y antes de jalar."),
        "5": ("running", "Ciclo en marcha", "LM Guide hacia corte."),
        "6": ("running", "Ciclo en marcha", "Corte neumático R/L."),
        "7": ("running", "Ciclo en marcha", "Descarga de pieza."),
        "8": ("running", "Ciclo en marcha", "Regreso a home y blower."),
    }
    estado, elabel, msg = labels.get(step_id, ("running", "Ciclo en marcha", step_id))
    msg = _cycle_piece_prefix() + msg
    global _STEP4_CLOSES
    if step_id == "4" and _STEP4_CLOSES:
        _STEP4_CLOSES = False
        msg = _cycle_piece_prefix() + "Home con pinzas abiertas. Cierra pinzas y sigue."
    elif step_id == "8" and _OVERLAP["feed_done"] and _LOT["active"]:
        msg = _cycle_piece_prefix() + "Pasos 1 a 3 listos. Esperando home con pinzas abiertas para cerrar."
    if step_id == "8" and _OVERLAP["on"] and not _OVERLAP["feed_done"] and _LOT["active"]:
        nxt = int(_LOT["cycles_done"]) + 2
        total = int(_LOT["cycles_total"])
        msg = msg + f" Ciclo {nxt} de {total} alimenta en paralelo."
    with _STATE_LOCK:
        keep_error = bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
        err_estado = _MACHINE_STATE.get("estado") == "error"
    if keep_error:
        # No pisar el estado de error: la secuencia queda pausada hasta Reset.
        _set_machine_ui(
            step_id=step_id,
            step_index=step_index,
            en_curso=True,
            estado="error" if err_estado or keep_error else estado,
            estado_label="Error",
        )
        return
    _set_machine_ui(
        estado=estado,
        estado_label=elabel,
        mensaje=msg,
        step_id=step_id,
        step_index=step_index,
        en_curso=True,
        ciclo_error="",
    )


def _cycle_fail(code: str, *, mensaje: str | None = None, **fmt) -> None:
    msg = mensaje if mensaje is not None else error_codes.format_message(code, **fmt)
    with _STATE_LOCK:
        in_error = _MACHINE_STATE.get("estado") == "error"
        had_code = bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
        was_en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
    # Error ya latched: no re-disparar eventos ni pisar el código (aunque el UI
    # se haya movido a running por un hilo huérfano).
    if had_code:
        if not in_error:
            _set_machine_ui(
                estado="error",
                estado_label="Error",
                en_curso=True,
            )
        # Si el error quedó sin popup de recuperación, abrirlo.
        if not _recovery_get().get("active"):
            existing = str(_MACHINE_STATE.get("ciclo_error") or code or "").strip()
            fixed = msg
            try:
                if existing:
                    fixed = error_codes.format_message(existing)
            except Exception:
                fixed = msg
            _recovery_begin(existing, fixed)
        return
    if not in_error:
        _push_event(msg, "error")
    _cadence_mark_interrupted()
    # Lote / ciclo en curso no se borran por error: solo Stop (o fin normal) lo hace.
    keep_lot = bool(_LOT.get("active") or _LOT.get("tray_wait") or was_en_curso)
    freeze_s = None
    if keep_lot and _PIECE_TICK0 is not None:
        freeze_s = round(max(0.0, time.time() - _PIECE_TICK0), 1)
    with _STATE_LOCK:
        _MACHINE_STATE["post_error_purga"] = True
    _feed_prime_clear()
    _recovery_begin(code, msg)
    # Banner: solo aviso corto. El detalle del error vive en recovery.mensaje (popup).
    _set_machine_ui(
        estado="error",
        estado_label="Error",
        mensaje=f"{code}. Abre el aviso para continuar." if code else "Abre el aviso para continuar.",
        en_curso=True if keep_lot else was_en_curso,
        ciclo_error=code,
        tiempo_corriendo_s=freeze_s,
    )
    if _MACHINE_STATE.get("ciclo_en_curso") or _cycle_thread_alive():
        try:
            _prod_log_error(code, msg)
        except Exception:
            pass
    _cycle_motion_halt_for_error()
    if _gripper_holding():
        _push_event(
            "Pinzas retenidas — pieza sujeta hasta Reset o Posición inicial.",
            "warn",
        )
    if _OVERLAP.get("on"):
        _OVERLAP["feed_error"] = True
    if _prefeeder_required() and (
        _MACHINE_STATE.get("ciclo_en_curso")
        or _MACHINE_STATE.get("estado") in ("running", "error")
    ):
        _prefeeder_process_halt_async()


def _cycle_latch_reset() -> None:
    _gripper_holding_set(False)
    with _STATE_LOCK:
        for key in _CYCLE_LATCH:
            _CYCLE_LATCH[key] = False


def _cycle_latch_after_completed_index(completed_idx: int) -> None:
    """Reconstruye interlocks según pasos ya terminados (índice en CYCLE_STEPS, -1 = ninguno)."""
    _cycle_latch_reset()
    if completed_idx >= 1:
        _CYCLE_LATCH["holder_encoder_active"] = True
    if completed_idx >= 2:
        _CYCLE_LATCH["feed_validated"] = True
    if completed_idx >= 3:
        # Paso 4: pinzas ON; holder/encoder según experimental «antes de jalar».
        _CYCLE_LATCH["holder_encoder_active"] = False
        _CYCLE_LATCH["gripper_active"] = True
    if completed_idx >= 4:
        _CYCLE_LATCH["lm_at_cut"] = True
    if completed_idx >= 5:
        # Antes del corte: experimental «antes de cortar».
        _CYCLE_LATCH["holder_encoder_active"] = True
        _CYCLE_LATCH["cut_done"] = True
    if completed_idx >= 6:
        _CYCLE_LATCH["discharge_done"] = True


def _cycle_assert_preconditions(step_index: int) -> bool:
    """False → pausa con error; no avanzar sin la secuencia lógica previa."""
    _cycle_latch_after_completed_index(step_index - 1)
    step_id = CYCLE_STEPS[step_index]["id"]

    if step_id == "3":
        if not _CYCLE_LATCH["holder_encoder_active"]:
            _cycle_fail("E001")
            return False
        # Alimentar siempre con holder+encoder cerrados.
        ok_hw, code = _plc_mirror_check({"holder": True, "encoder": True})
        if not ok_hw:
            if code:
                _cycle_fail(code)
            return False

    if step_id == "4":
        if not _CYCLE_LATCH["feed_validated"]:
            _cycle_fail("E002")
            return False

    if step_id == "5":
        # Paso 4: pinzas ON + holder/encoder según «antes de jalar».
        if not _CYCLE_LATCH["gripper_active"]:
            _cycle_fail("E003")
            return False
        pull_h, pull_e = _exp_before_pull_valves(_load_motion_tune())
        ok_hw, code = _plc_mirror_check(
            {"gripper": True, "holder": pull_h, "encoder": pull_e}
        )
        if not ok_hw:
            if code:
                _cycle_fail(code)
            return False

    if step_id == "6":
        if not _CYCLE_LATCH["lm_at_cut"]:
            _cycle_fail("E004")
            return False

    if step_id == "7":
        if not _CYCLE_LATCH["cut_done"]:
            _cycle_fail("E005")
            return False

    if step_id == "8":
        if not _CYCLE_LATCH["discharge_done"]:
            _cycle_fail("E006")
            return False

    return True


def _cycle_latch_mark_step_done(step_index: int) -> None:
    _cycle_latch_after_completed_index(step_index)


def _read_feed_stage(sent: dict):
    """Respuesta de feed_stage: encontrada, ok, nombre de error, lectura L/R."""
    feed = {"R": None, "L": None}
    found = False
    ok = False
    error_name = ""
    for raw in sent.get("replies") or []:
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") != "feed_stage":
            continue
        found = True
        ok = bool(msg.get("ok"))
        error_name = str(msg.get("error") or "")
        for side in ("R", "L"):
            block = msg.get(side)
            if isinstance(block, dict):
                entry = {
                    "laser": block.get("laser"),
                    "enc_ok": block.get("enc_ok"),
                }
                if block.get("counts") is not None:
                    try:
                        entry["counts"] = int(block["counts"])
                    except (TypeError, ValueError):
                        pass
                feed[side] = entry
    return found, ok, error_name, feed


def _fail_feed_stage(error_name: str) -> None:
    if _OVERLAP.get("feed_done"):
        return
    if _cycle_should_stop():
        return
    err = (error_name or "").strip() or "feed_no_reply"
    if err == "ok":
        err = "unknown"
    joint_side = _joint_side_from_feed_error(err)
    if joint_side:
        _joint_begin_from_feed(joint_side)
        return
    code = error_codes.feed_stage_code(err)
    if code == "E038":
        _cycle_fail("E038", detail=err)
    else:
        _cycle_fail(code)


def _motion_reply_op(sent: dict, op: str) -> dict | None:
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            return msg if isinstance(msg, dict) else None
    return None


def _encoder_counts_from_msg(msg: dict | None) -> dict:
    """Extrae counts R/L de un ack Motion (lmguide_move / status)."""
    out = {"R": None, "L": None}
    if not isinstance(msg, dict):
        return out
    for side in ("R", "L"):
        block = msg.get("enc_" + side)
        if not isinstance(block, dict):
            continue
        entry = {}
        if block.get("counts") is not None:
            try:
                entry["counts"] = int(block["counts"])
            except (TypeError, ValueError):
                pass
        elif block.get("counts_live") is not None:
            try:
                entry["counts"] = abs(int(block["counts_live"]))
            except (TypeError, ValueError):
                pass
        if entry:
            out[side] = entry
    return out


def _cycle_move_lm_send(mm: float, inject=None, on_line=None) -> dict:
    tune = _load_motion_tune()
    puu = _mm_to_puu(mm)
    payload = {"op": "lmguide_move", "puu": puu}
    payload.update(_lm_cmd_fields(tune))
    return _cycle_motion_send(
        payload,
        timeout_s=_MOTION_LM_MOVE_TIMEOUT_S,
        inject=inject,
        on_line=on_line,
    )


def _cycle_move_lm_mm(mm: float, inject=None, on_line=None) -> bool:
    return _motion_cmd_serial_ok(_cycle_move_lm_send(mm, inject=inject, on_line=on_line), "E043")


def _cycle_lm_home_absolute(*, with_set_data: bool = False) -> bool:
    """Mueve el LM Guide a PUU 0 absoluto (no homing mecánico).

    with_set_data: embebe campos de ciclo en la misma orden (arranque fresco).
    """
    global _CYCLE_SET_DATA_ON_HOME
    tune = _load_motion_tune()
    payload = {"op": "lmguide_move", "target": "home"}
    payload.update(_lm_cmd_fields(tune))
    if with_set_data:
        flat = _motion_data_flat(_MOTION_DATA)
        payload["target_length_mm"] = flat.get("target_length_mm")
        payload["tolerance_mm"] = flat.get("tolerance_mm", 2.0)
        payload["modelo_id"] = flat.get("modelo_id") or ""
        payload["measure_comp_mm"] = flat.get("measure_comp_mm", 0.0)
    sent = _cycle_motion_send(payload, timeout_s=_MOTION_LM_MOVE_TIMEOUT_S)
    if not _motion_cmd_serial_ok(sent, "E043"):
        return False
    if with_set_data:
        msg = _motion_reply_op(sent, "lmguide_move")
        _CYCLE_SET_DATA_ON_HOME = bool(msg and msg.get("set_data") is True)
        if not _CYCLE_SET_DATA_ON_HOME:
            # Firmware antiguo sin set_data embebido: el paso 1 lo reenvía.
            _CYCLE_SET_DATA_ON_HOME = False
    return True


def _cycle_encoder_reset() -> bool:
    """Reset hardware PCNT (solo recuperación / uso manual)."""
    return _motion_cmd_serial_ok(
        _cycle_motion_send({"op": "encoder_reset"}, timeout_s=2.0),
        "E043",
    )


def _cycle_encoder_set0() -> bool:
    """Marca referencia relativa R+L; PCNT sigue contando.

    En ciclo normal no hace falta antes de feed_stage: FeedStage ya hace set0
    al iniciar. Se conserva para UI / recuperación / pruebas.
    """
    return _motion_cmd_serial_ok(
        _cycle_motion_send({"op": "encoder_set0", "side": "both"}, timeout_s=2.0),
        "E043",
    )


def _cycle_feeder_servo(on: bool) -> bool:
    """Enable (on=True) / Disable (on=False) — mismo op que el botón feeder_servo de la UI."""
    return _motion_cmd_serial_ok(
        _cycle_motion_send(
            {"op": "feeder_servo_on" if on else "feeder_servo_off"},
            timeout_s=3.0,
        ),
        "E043",
    )


def _cycle_feeder_reverse(side: str, *, rpm: int, ms: int) -> bool:
    """PV inverso en ciclo (sin chequear owner busy)."""
    side_s = str(side or "both").strip().upper()
    if side_s not in ("R", "L", "BOTH"):
        side_s = "BOTH"
    rpm_i = _exp_rpm(rpm)
    ms_i = _exp_ms(ms)
    payload = {
        "op": "feeder_reverse",
        "side": "both" if side_s == "BOTH" else side_s,
        "rpm": rpm_i,
        "ms": ms_i,
    }
    timeout_s = max(2.0, (ms_i / 1000.0) + 2.0)
    sent = _cycle_motion_send(payload, timeout_s=timeout_s)
    if not sent.get("sent"):
        _cycle_fail(sent.get("code") or "E043")
        return False
    if not _motion_cmd_ok(sent, "feeder_reverse"):
        _cycle_fail("E044")
        return False
    return True


def _cycle_set_holder_encoder(
    holder: bool,
    encoder: bool,
    settle_ms: int,
    *,
    expected_extra: dict | None = None,
    cycle_fail: bool = True,
) -> tuple[bool, str | None]:
    """Aplica holder/encoder dual (juntos o por separado) y confirma espejo."""
    expected = {"holder": bool(holder), "encoder": bool(encoder)}
    if expected_extra:
        expected.update(expected_extra)
    settle = max(0, int(settle_ms))

    if holder == encoder:
        ok_hw, code = _plc_accept_valve(
            _cycle_plc_send(
                {
                    "op": "valve",
                    "name": "holder_encoder",
                    "on": bool(holder),
                    "settle_ms": settle,
                }
            ),
            expected,
        )
    else:
        ok_hw, code = _plc_accept_valve(
            _cycle_plc_send({"op": "valve", "name": "holder", "on": bool(holder)}),
            None,
        )
        if ok_hw:
            ok_hw, code = _plc_accept_valve(
                _cycle_plc_send(
                    {
                        "op": "valve",
                        "name": "encoder",
                        "on": bool(encoder),
                        "settle_ms": settle,
                    }
                ),
                expected,
            )
    if not ok_hw:
        if cycle_fail and code:
            _cycle_fail(code)
        return False, code
    return True, None


def _cycle_close_holder_encoder_for_feed(
    params: dict, *, cycle_fail: bool = True, mark_latch: bool = False
) -> bool:
    """Paso 2 / priming / solape: cerrar holder+encoder para alimentar."""
    _push_event("Holder y encoder cerrados para alimentar.")
    ok, _code = _cycle_set_holder_encoder(
        True,
        True,
        int(params["ms_holder_enc_stable"]),
        cycle_fail=cycle_fail,
    )
    if not ok:
        return False
    if mark_latch:
        _CYCLE_LATCH["holder_encoder_active"] = True
    return True


def _cycle_grip_apply_before_pull(params: dict) -> bool:
    """Paso 4: pinzas ON + holder/encoder «antes de jalar» en una sola orden PLC."""
    pull_h, pull_e = _exp_before_pull_valves(_load_motion_tune())
    _push_event(
        f"Pinzas ON; antes de jalar: holder={'ON' if pull_h else 'OFF'} "
        f"encoder={'ON' if pull_e else 'OFF'}."
    )
    ok_hw, code = _plc_accept_valve(
        _cycle_plc_send(
            {
                "op": "valve",
                "gripper": True,
                "holder": bool(pull_h),
                "encoder": bool(pull_e),
                "settle_ms": int(params["ms_after_grip_release"]),
            }
        ),
        {"gripper": True, "holder": bool(pull_h), "encoder": bool(pull_e)},
    )
    if not ok_hw:
        if code:
            _cycle_fail(code)
        return False
    return True


def _cycle_apply_exp_before_cut(params: dict) -> bool:
    """Antes de cortar: holder/encoder experimentales → enable + corrección → suelto."""
    tune = _load_motion_tune()
    holder, encoder = _exp_before_cut_valves(tune)
    _push_event(
        f"Antes de cortar: holder={'ON' if holder else 'OFF'} "
        f"encoder={'ON' if encoder else 'OFF'}."
    )
    ok_he, _code_he = _cycle_set_holder_encoder(
        holder,
        encoder,
        int(params["ms_holder_enc_stable"]),
        expected_extra={"gripper": True},
    )
    if not ok_he:
        return False

    _CYCLE_LATCH["holder_encoder_active"] = bool(holder or encoder)

    jobs = _exp_correction_jobs(tune) if holder else []
    if not jobs:
        return True

    _push_event(
        "Corrección servo "
        + "+".join(j["side"] for j in jobs)
        + " (enable → reverse → corte)."
    )
    if not _cycle_feeder_servo(True):
        return False

    if (
        len(jobs) == 2
        and jobs[0]["rpm"] == jobs[1]["rpm"]
        and jobs[0]["ms"] == jobs[1]["ms"]
    ):
        if not _cycle_feeder_reverse(
            "BOTH", rpm=jobs[0]["rpm"], ms=jobs[0]["ms"]
        ):
            if EXPERIMENTAL_SERVO_OFF:
                _cycle_feeder_servo(False)
            return False
    else:
        for job in jobs:
            if not _cycle_feeder_reverse(
                job["side"], rpm=job["rpm"], ms=job["ms"]
            ):
                if EXPERIMENTAL_SERVO_OFF:
                    _cycle_feeder_servo(False)
                return False

    # Volver a Suelto si el modo experimental lo exige (paso 4 lo dejó así).
    if EXPERIMENTAL_SERVO_OFF:
        if not _cycle_feeder_servo(False):
            return False
    return True


def _cycle_capture_encoder_counts(sent: dict | None = None) -> dict:
    """Lee counts R/L: primero del ack lmguide_move; si no, status."""
    if sent is not None:
        from_move = _encoder_counts_from_msg(_motion_reply_op(sent, "lmguide_move"))
        if from_move.get("R") is not None or from_move.get("L") is not None:
            return from_move
    return _encoder_counts_from_msg(_motion_status_get())


def _cycle_run_step(step_id: str, params: dict) -> bool:
    # Step 1. Start — set_data (omitido si ya viajó en el home de arranque fresco).
    if step_id == "1":
        global _CYCLE_SET_DATA_ON_HOME
        if _CYCLE_SET_DATA_ON_HOME:
            _CYCLE_SET_DATA_ON_HOME = False
            return True
        flat = _motion_data_flat(_MOTION_DATA)
        return _motion_cmd_serial_ok(_cycle_motion_send(flat, timeout_s=2.0), "E043")

    # Step 2. Cerrar holder+encoder para alimentar (siempre ambos ON).
    # FeedStage (paso 3) marca set0 al iniciar: no hace falta encoder_set0 aquí.
    if step_id == "2":
        return _cycle_close_holder_encoder_for_feed(params, mark_latch=True)

    # Step 3. Alimentar (tune + FeedStage en una sola orden). CAPTURA A en la reply.
    if step_id == "3":
        sides = _run_sides()
        payload = _feed_stage_payload(sides if sides in ("R", "L") else None)
        side_name = {"R": "derecho", "L": "izquierdo", "LR": "L y R"}[sides]
        _set_machine_ui(mensaje=_cycle_piece_prefix() + f"Alimentación {side_name}.")
        sent = _cycle_motion_send(payload, timeout_s=65.0)
        found, ok, error_name, feed = _read_feed_stage(sent)
        if not sent.get("sent"):
            _cycle_fail(sent.get("code") or "E043")
            return False
        if not found or not ok:
            _fail_feed_stage(error_name if found else "feed_no_reply")
            return False
        empty = _feed_stage_empty_error(feed, sides)
        if empty:
            _fail_feed_stage(empty)
            return False
        _set_machine_ui(feed=feed)
        global _PIECE_BLOWER_MM, _PIECE_FEED
        prog = _target_length_mm()
        _PIECE_BLOWER_MM = prog if prog > 0 else None
        _PIECE_FEED = {"R": feed.get("R"), "L": feed.get("L")}
        return True

    # Step 4. Pinzas ON + holder/encoder «antes de jalar» (una sola orden PLC).
    if step_id == "4":
        if not _cycle_grip_apply_before_pull(params):
            return False
        # Desde el cierre: si falla el resto del ciclo, retener (no abrir).
        _gripper_holding_set(True)
        if EXPERIMENTAL_SERVO_OFF:
            if not _cycle_feeder_servo(False):
                return False
        return True

    # Step 5. LM Guide a corte; CAPTURA B con la misma referencia del FeedStage.
    if step_id == "5":
        plan = _active_cut_plan()
        if plan is None:
            _cycle_fail("E007")
            return False
        cut_mm = float(plan["first_mm"])
        if plan.get("sequential"):
            _push_event(
                f"LM Guide a corte 1/{plan['first_label']}: {cut_mm:.2f} mm "
                f"(comp R {plan['comp_r_mm']:+.2f} / L {plan['comp_l_mm']:+.2f})."
            )
        else:
            _push_event(f"LM Guide a corte: {cut_mm:.2f} mm.")
        move_sent = _cycle_move_lm_send(cut_mm)
        if not _motion_cmd_serial_ok(move_sent, "E043"):
            return False
        if not _cycle_sleep_ms(int(params["ms_lm_at_cut"])):
            return False
        global _PIECE_ENC_AT_LM
        # CAPTURA B desde el ack del move (misma base que FeedStage; sin status).
        _PIECE_ENC_AT_LM = _cycle_capture_encoder_counts(move_sent)
        return True

    # Step 6. Antes de cortar (experimental): holder/encoder → corrección → corte.
    # Si hay compensación dual aplicable, mover a la segunda posición y cortar
    # el otro lado. Joint: omitir el corte del lado afectado (skip_cut).
    if step_id == "6":
        plan = _active_cut_plan()
        if plan is None:
            _cycle_fail("E007")
            return False

        # Holder/encoder + corrección servo (enable obligatorio: paso 4 lo deja Suelto).
        if not _cycle_apply_exp_before_cut(params):
            return False

        def run_cutter(valve: str, label: str) -> bool:
            _set_machine_ui(mensaje=_cycle_piece_prefix() + f"Corte {label}.")
            ok_c, code_c = _plc_accept_valve(
                _cycle_plc_send(
                    {
                        "op": "valve",
                        "name": valve,
                        "on": True,
                        "ms": int(params["ms_cutter_on"]),
                    }
                )
            )
            if not ok_c:
                if code_c:
                    _cycle_fail(code_c)
                return False
            return _cycle_sleep_ms(int(params["ms_after_cutter"]))

        first_valve = _joint_valve_allowed(str(plan["first_valve"]))
        if first_valve:
            if not run_cutter(first_valve, str(plan["first_label"])):
                return False
        else:
            _push_event(
                f"Corte omitido por Joint ({_JOINT.get('skip_cut') or '?'}): "
                f"{plan['first_label']}.",
                "warn",
            )
        if not plan.get("sequential"):
            return True

        # Segundo lado: su compensación es obligatoria. Solo Joint puede omitir
        # el corte de ese lado; el movimiento a su posición no se “salta” a la ligera.
        second_valve = _joint_valve_allowed(str(plan["second_valve"]))
        second_mm = float(plan["second_mm"])
        first_mm = float(plan["first_mm"])
        # Misma resolución que la compensación HMI (0.01 mm).
        need_move = abs(second_mm - first_mm) >= LMGUIDE_MM_RES
        if need_move:
            _push_event(
                f"LM Guide a corte 2/{plan['second_label']}: {second_mm:.2f} mm "
                f"(comp R {plan['comp_r_mm']:+.2f} / L {plan['comp_l_mm']:+.2f})."
            )
            if not _cycle_move_lm_mm(second_mm):
                return False
            if not _cycle_sleep_ms(int(params["ms_lm_at_cut"])):
                return False
        elif second_valve:
            _push_event(
                f"Corte 2/{plan['second_label']} en {second_mm:.2f} mm "
                f"(misma posición; comp R {plan['comp_r_mm']:+.2f} / "
                f"L {plan['comp_l_mm']:+.2f})."
            )

        if not second_valve:
            _push_event(
                f"Segundo corte omitido por Joint ({_JOINT.get('skip_cut') or '?'}): "
                f"{plan['second_label']}.",
                "warn",
            )
            return True
        return run_cutter(second_valve, str(plan["second_label"]))

    # Step 7. Depósito + soltar pinzas. El solape (1–3) arranca al liberar
    # pinzas (rules §4.7 / §8), no durante el traslado al montón: si el láser
    # sigue viendo la pieza en descarga, FeedStage “ok” con counts 0 y el
    # siguiente ciclo corta en vacío.
    # Experimental: tras llegada al montón, ServoCAN ON en paralelo (no bloquea gripper).
    if step_id == "7":
        pos = _active_cut_deposit()
        if pos is None:
            _cycle_fail("E007")
            return False
        _cut_mm, disc_mm, stack = pos
        _push_event(f"Depósito montón {stack}: {disc_mm:.2f} mm.")
        # Gripper OFF solo espera llegada al montón (lmguide_move ok + settle).
        # ServoCAN ON no bloquea: se dispara después del move, en paralelo.
        if not _cycle_move_lm_mm(disc_mm):
            return False
        if not _cycle_sleep_ms(int(params["ms_lm_at_discharge"])):
            return False
        if EXPERIMENTAL_SERVO_OFF:

            def _feeder_servo_on_bg() -> None:
                if not _cycle_feeder_servo(True):
                    _push_event(
                        "ServoCAN ON (tras depósito) no confirmó; pinzas ya no esperan.",
                        "warn",
                    )

            threading.Thread(target=_feeder_servo_on_bg, daemon=True).start()
        # Error en el tramo con pieza sujeta: no abrir pinzas (retención).
        if _cycle_should_abort():
            return False
        ok_g, code_g = _plc_accept_valve(
            _cycle_plc_send(
                {
                    "op": "valve",
                    "name": "gripper",
                    "on": False,
                    "settle_ms": int(params["ms_after_grip_release"]),
                }
            ),
            {"gripper": False},
        )
        if not ok_g:
            if code_g:
                _cycle_fail(code_g)
            if _cycle_should_stop():
                _overlap_reset()
            return False
        _gripper_holding_set(False)
        # Cadencia = liberación confirmada (no fin del paso 8).
        _cadence_on_gripper_off()
        # Siguiente ciclo: holder + feed en paralelo con ReleasePart (paso 8).
        if (
            _next_piece_follows()
            and not _OVERLAP["on"]
            and not _OVERLAP["feed_done"]
            and not _CYCLE_CTRL["pause_after_step"].is_set()
            and not _cycle_should_stop()
            and not _cycle_error_active()
        ):
            if not _overlap_arm(params):
                return False
        return True

    # Step 8. ReleasePart: despeje → home + blower (una orden Motion).
    if step_id == "8":
        pos = _active_cut_deposit()
        if pos is None:
            _cycle_fail("E007")
            return False
        _cut_mm, deposit, _stack = pos
        clearance = (
            _LOT["clearance_mm"]
            if _LOT["active"]
            else abs(_num(params.get("gripper_clearance_mm"), 0.0))
        )
        clear_at = _clear_of(deposit, clearance)
        if (
            _next_piece_follows()
            and not _OVERLAP["on"]
            and not _OVERLAP["feed_done"]
            and not _cycle_should_stop()
            and not _cycle_error_active()
        ):
            if not _overlap_arm(params):
                return False

        blower_ms = _cycle_blower_duration_ms(params)
        payload = {
            "op": "release_part",
            "blower_delay_ms": int(params["ms_blower_delay"]),
            "blower_ms": blower_ms,
            "home_settle_ms": int(params["ms_at_home_end"]),
        }
        payload.update(_lm_cmd_fields(_load_motion_tune()))
        if abs(clear_at - deposit) >= 0.05:
            payload["clear_puu"] = _mm_to_puu(clear_at)
            _push_event(
                f"ReleasePart: despeje {clear_at:.2f} mm → home + blower "
                f"{blower_ms} ms (L {_cycle_blower_length_mm():.1f} mm)."
            )
        else:
            _push_event(
                f"ReleasePart: home + blower {blower_ms} ms "
                f"(L {_cycle_blower_length_mm():.1f} mm)."
            )

        feed_inject, feed_line = _overlap_feed_hook()
        sent = _cycle_motion_send(
            payload,
            timeout_s=_MOTION_LM_MOVE_TIMEOUT_S * 2.0,
            inject=feed_inject,
            on_line=feed_line,
        )
        if _cycle_should_stop():
            _send_motion_urgent(
                ['{"op":"feeder_stop"}', '{"op":"lmguide_stop"}', '{"op":"blower_off"}']
            )
            return False
        replies = sent.get("replies") or []
        _overlap_ensure_feed_collected(replies, budget_s=4.0)
        if not _motion_cmd_serial_ok(sent, "E043"):
            return False
        rp = _motion_reply_op(sent, "release_part")
        if not (rp and rp.get("lm_homed") is True):
            _cycle_fail("E044")
            return False
        if _OVERLAP["on"] and not _OVERLAP["feed_done"] and not _OVERLAP["feed_sent"]:
            _overlap_start_feed_task()
        # Settle ya corrió en Motion; poll solape residual breve.
        t_end = time.time() + 0.05
        while time.time() < t_end:
            if _cycle_wait_if_paused():
                _send_motion_urgent(['{"op":"blower_off"}'])
                return False
            _overlap_poll_feed(0.05)
            time.sleep(0.02)
        # feed_error del solape es del SIGUIENTE ciclo; ReleasePart de esta pieza ya cerró.
        # _overlap_wait_feed_done corta el lote antes de acreditar/cortar la fantasma.
        return True

    return True


def _next_piece_follows() -> bool:
    """Hay otra pieza del lote después de la que se está cerrando."""
    if _joint_blocks_next_piece():
        return False
    if _CYCLE_CTRL.get("step_mode"):
        return False
    if not _LOT["active"] or _LOT["tray_wait"]:
        return False
    if not _lot_would_need_another_after_credit():
        return False
    done_after = int(_LOT["cycles_done"]) + 1
    deposits = int(_LOT["deposits_per_tray"] or 0)
    if deposits and done_after % deposits == 0:
        return False
    return True


def _overlap_reset() -> None:
    _OVERLAP["gen"] += 1
    _OVERLAP["on"] = False
    _OVERLAP["feed_sent"] = False
    _OVERLAP["feed_done"] = False
    _OVERLAP["holder_error"] = ""
    _OVERLAP["feed"] = None
    _OVERLAP["blower_mm"] = None
    _OVERLAP["t0"] = None
    _OVERLAP["feed_error"] = False
    _OVERLAP["feed_error_name"] = ""
    _OVERLAP["feed_task_started"] = False
    _OVERLAP["sides"] = ""
    _OVERLAP["holder_ready"].clear()
    global _STEP4_CLOSES
    _STEP4_CLOSES = False
    if _MACHINE_STATE.get("ciclo_paralelo_id"):
        _set_machine_ui(paralelo_id="")


def _overlap_apply_feed_error() -> None:
    """Latchea el fallo del solape (tras cerrar la pieza actual)."""
    if not _OVERLAP.get("feed_error"):
        return
    err = str(_OVERLAP.get("feed_error_name") or "").strip() or "feed_no_reply"
    if not _MACHINE_STATE.get("ciclo_error"):
        _fail_feed_stage(err)


def _feed_prime_clear() -> None:
    _FEED_PRIME["ready"] = False
    _FEED_PRIME["feed"] = None
    _FEED_PRIME["sides"] = ""
    _FEED_PRIME["blower_mm"] = None


def _feed_prime_ready() -> bool:
    return bool(_FEED_PRIME.get("ready") and _FEED_PRIME.get("feed") is not None)


def _feed_prime_run(params: dict) -> bool:
    """Alimenta hasta el láser sin contar tiempo de ciclo. No latchea error de ciclo."""
    _feed_prime_clear()
    sides = _lot_cycle_sides_now() or _feed_sides_of(params)
    _set_machine_ui(
        mensaje="Adelantando alimentación para el siguiente arranque…",
        paralelo_id="",
    )
    _push_event(f"Priming alimentación ({sides}) — fuera de tiempo de ciclo.")
    if not _cycle_close_holder_encoder_for_feed(params, cycle_fail=False):
        return False
    # FeedStage aplica tune + set0 interno: una sola orden Motion.
    payload = _feed_stage_payload(sides if sides in ("R", "L") else None)
    sent = _cycle_motion_send(payload, timeout_s=65.0)
    if _cycle_should_stop():
        return False
    found, ok, _err, feed = _read_feed_stage(sent)
    if not sent.get("sent") or not found or not ok:
        return False
    if _feed_stage_empty_error(feed, sides):
        return False
    prog = _target_length_mm()
    _FEED_PRIME["ready"] = True
    _FEED_PRIME["feed"] = {"R": feed.get("R"), "L": feed.get("L")}
    _FEED_PRIME["sides"] = sides
    _FEED_PRIME["blower_mm"] = prog if prog > 0 else None
    _set_machine_ui(feed=feed)
    _push_event("Material en láser. Siguiente arranque desde pinzas.")
    return True


def _feed_prime_prepare_after_segment() -> None:
    """Tras última pieza de lote/charola: dejar material listo (no cuenta en cadencia)."""
    if _CYCLE_CTRL.get("step_mode"):
        return
    if _cycle_should_stop() or _cycle_error_active():
        return
    params = _load_cycle_params()
    if _feed_prime_run(params):
        return
    _feed_prime_clear()
    _push_event(
        "No se pudo adelantar la alimentación. El siguiente Start hará ciclo completo.",
        "warn",
    )


def _feed_prime_consume() -> bool:
    """Si hay prime válido: publica modelo actual, carga feed y arranca en paso 4."""
    global _PIECE_BLOWER_MM, _PIECE_FEED, _PIECE_ENC_AT_LM, _STEP4_CLOSES
    if not _feed_prime_ready():
        return False
    # Modelo/longitud pueden haber cambiado; el corte usa la medida nueva.
    sent_data = _cycle_motion_send(_motion_data_flat(_MOTION_DATA), timeout_s=2.0)
    if not sent_data.get("sent") or not _motion_cmd_ok(sent_data, "set_data"):
        _feed_prime_clear()
        _push_event("Prime descartado: no se pudo publicar set_data.", "warn")
        return False
    feed = _FEED_PRIME.get("feed") or {"R": None, "L": None}
    mm = _FEED_PRIME.get("blower_mm")
    _feed_prime_clear()
    _STEP4_CLOSES = True
    _PIECE_ENC_AT_LM = {"R": None, "L": None}
    _PIECE_FEED = {"R": feed.get("R"), "L": feed.get("L")}
    if mm is not None:
        _PIECE_BLOWER_MM = mm
    _set_machine_ui(feed=_PIECE_FEED)
    _push_event("Arranque con material ya alimentado — desde paso 4.")
    return True


def _plc_accepted(sent: dict, op: str) -> bool:
    if not sent.get("sent"):
        return False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            return bool(msg.get("ok", True))
    return False


def _overlap_fail_holder(code: str) -> None:
    """Falla del holder en solape: aborta el ciclo al instante (no espera al paso 8)."""
    if not code or code == "stop":
        _OVERLAP["holder_error"] = "stop"
        return
    _OVERLAP["holder_error"] = code
    _OVERLAP["feed_error"] = True
    _cycle_fail(code)


def _overlap_start_holder(params: dict) -> None:
    gen = int(_OVERLAP["gen"])
    worker_gen = getattr(_CYCLE_WORKER_TLS, "gen", None)

    def run() -> None:
        if worker_gen is not None:
            _CYCLE_WORKER_TLS.gen = worker_gen
        if _OVERLAP["gen"] != gen or _cycle_should_abort():
            return
        t_holder = time.perf_counter()
        nxt = int(_LOT["cycles_done"]) + 2
        _set_machine_ui(paralelo_id="2")
        # Solape: cerrar holder+encoder para la siguiente alimentación.
        if not _cycle_close_holder_encoder_for_feed(params, cycle_fail=False):
            if _OVERLAP["gen"] != gen:
                return
            if _cycle_should_stop() or _cycle_should_abort():
                _OVERLAP["holder_error"] = "stop"
                return
            _overlap_fail_holder("E010")
            return
        if _OVERLAP["gen"] != gen:
            return
        # FeedStage del solape hace set0 al arrancar; no hace falta encoder_set0 aquí.
        _OVERLAP["holder_ready"].set()
        _prod_log_op(
            "2",
            time.perf_counter() - t_holder,
            piece=nxt,
            parallel=True,
        )

    threading.Thread(target=run, daemon=True).start()


def _overlap_arm(params: dict) -> bool:
    """Paso 1 del siguiente ciclo tras liberar pinzas. Holder/feed con ReleasePart."""
    if _OVERLAP["on"] or _OVERLAP["feed_done"]:
        return True
    t_arm = time.perf_counter()
    if not _motion_cmd_serial_ok(
        _cycle_motion_send(_motion_data_flat(_MOTION_DATA), timeout_s=2.0),
        "E043",
    ):
        return False
    # Tune va embebido en el feed_stage del solape (sin set_tune aparte).
    _OVERLAP["on"] = True
    _OVERLAP["t0"] = time.time()
    _OVERLAP["feed_sent"] = False
    _OVERLAP["feed_done"] = False
    _OVERLAP["holder_error"] = ""
    _OVERLAP["feed"] = None
    _OVERLAP["blower_mm"] = None
    _OVERLAP["feed_error"] = False
    _OVERLAP["feed_task_started"] = False
    _OVERLAP["sides"] = _lot_predict_next_cycle_sides()
    _OVERLAP["holder_ready"].clear()
    nxt = int(_LOT["cycles_done"]) + 2
    total = int(_LOT["cycles_total"])
    next_sides = _OVERLAP["sides"] or "LR"
    _set_machine_ui(
        paralelo_id="1",
        mensaje=(
            _cycle_piece_prefix()
            + f"Pinzas abiertas. Ciclo {nxt} de {total} ({next_sides}) alimenta en paralelo."
        ),
    )
    _push_event(
        f"Ciclo {nxt} de {total}: paso 1 ({next_sides}) en paralelo tras liberar pinzas."
    )
    _prod_log_op(
        "1",
        time.perf_counter() - t_arm,
        piece=nxt,
        parallel=True,
    )
    _overlap_start_holder(params)
    return True


def _overlap_feed_hook():
    """Alimentación del siguiente ciclo mientras el eje todavía se mueve.

    Si feed_stage ya se inyectó (p. ej. en el depósito) o ya falló, no rearmar
    inject en ReleasePart: eso haría esperar otra respuesta que nunca llega.
    """
    if (
        not _OVERLAP["on"]
        or _OVERLAP["feed_done"]
        or _OVERLAP.get("feed_error")
        or _OVERLAP.get("feed_sent")
    ):
        return None, None
    return _overlap_feed_inject, _overlap_on_feed_line


def _overlap_on_feed_line(msg: dict) -> None:
    if msg.get("op") != "feed_stage" or _OVERLAP["feed_done"] or _OVERLAP.get("feed_error"):
        return
    if not _overlap_accept_feed([_compact(msg)]):
        return
    _set_machine_ui(
        mensaje=(
            _cycle_piece_prefix()
            + "Pasos 1 a 3 listos. Esperando home con pinzas abiertas para cerrar."
        ),
    )


def _overlap_feed_inject():
    """None sigue esperando al holder. [] no escribe. Una lista se manda una vez."""
    if (
        not _OVERLAP["on"]
        or _OVERLAP["feed_done"]
        or _OVERLAP["feed_sent"]
        or _OVERLAP.get("feed_error")
    ):
        return []
    if _cycle_should_abort() or _OVERLAP["holder_error"]:
        return []
    if not _OVERLAP["holder_ready"].is_set():
        return None
    _OVERLAP["feed_sent"] = True
    _OVERLAP["feed_t0"] = time.perf_counter()
    nxt = int(_LOT["cycles_done"]) + 2
    total = int(_LOT["cycles_total"])
    next_sides = _lot_predict_next_cycle_sides()
    _OVERLAP["sides"] = next_sides
    _set_machine_ui(
        paralelo_id="3",
        mensaje=_cycle_piece_prefix() + f"Home y soplo. Ciclo {nxt} de {total} alimenta ({next_sides}).",
    )
    _push_event(
        f"Ciclo {nxt} de {total}: alimentación {next_sides} en paralelo con el regreso a home."
    )
    payload = _feed_stage_payload(next_sides if next_sides in ("R", "L") else None)
    return [_compact(payload)]


def _replies_have_op(replies, op: str) -> bool:
    for raw in replies or []:
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            return True
    return False


def _overlap_accept_feed(replies) -> bool:
    if _OVERLAP["feed_done"]:
        return True
    found, ok, error_name, feed = _read_feed_stage({"replies": replies})
    if _cycle_should_stop() and (not found or not ok):
        return False
    next_sides = str(_OVERLAP.get("sides") or "") or _lot_predict_next_cycle_sides()
    empty = _feed_stage_empty_error(feed, next_sides) if found and ok else ""
    if not found or not ok or empty:
        # No latchear ciclo_error aquí: tumbaría la pieza actual a media descarga.
        # Se aplica en _overlap_wait_feed_done tras acreditar la pieza en curso.
        _OVERLAP["feed_error"] = True
        _OVERLAP["feed_error_name"] = empty or (
            error_name if found else "feed_no_reply"
        )
        if empty:
            _push_event(
                "Alimentación solapada sin avance de encoder. "
                "Se pausa al cerrar esta pieza.",
                "warn",
            )
        return False
    _OVERLAP["feed_done"] = True
    _OVERLAP["feed"] = feed
    prog = _target_length_mm()
    _OVERLAP["blower_mm"] = prog if prog > 0 else None
    _set_machine_ui(feed=feed, paralelo_id="")
    nxt = int(_LOT["cycles_done"]) + 2
    total = int(_LOT["cycles_total"])
    _push_event(f"Ciclo {nxt} de {total}: pasos 1 a 3 listos.")
    t0 = _OVERLAP.get("feed_t0")
    if t0 is not None:
        _prod_log_op(
            "3",
            time.perf_counter() - float(t0),
            piece=nxt,
            parallel=True,
            feed=feed,
        )
    return True


def _motion_drain_replies(
    want_op: str, budget_s: float, *, blocking: bool = True
) -> list[str]:
    """Lee respuestas pendientes en Motion sin enviar órdenes nuevas."""
    port = _serial_port("motion")
    if not port:
        return []
    try:
        import serial
    except ImportError:
        return []
    global _motion_serial
    wait_s = max(0.05, min(budget_s, 5.0))
    if blocking:
        if not _serial_lock.acquire(timeout=wait_s):
            return []
    elif not _serial_lock.acquire(blocking=False):
        return []
    try:
        ser = _motion_serial
        if ser is None or not ser.is_open or getattr(ser, "port", None) != port:
            return []
        return _read_lines_until_op(ser, want_op, budget_s, trace_prefix=None)
    finally:
        _serial_lock.release()


def _overlap_try_accept_from_replies(replies) -> None:
    if _OVERLAP["feed_done"]:
        return
    if replies and _replies_have_op(replies, "feed_stage"):
        _overlap_accept_feed(replies)


def _overlap_poll_feed(budget_s: float = 0.35) -> None:
    if not _OVERLAP["on"] or _OVERLAP["feed_done"] or not _OVERLAP["feed_sent"]:
        return
    lines = _motion_drain_replies("feed_stage", budget_s, blocking=budget_s >= 0.35)
    if lines:
        _overlap_accept_feed(lines)


def _overlap_ensure_feed_collected(replies, budget_s: float = 3.0) -> None:
    """Recolecta feed_stage si la respuesta llegó tras lmguide_move/home."""
    if not _OVERLAP["on"] or _OVERLAP["feed_done"]:
        return
    _overlap_try_accept_from_replies(replies)
    if _OVERLAP["feed_done"]:
        return
    if _OVERLAP["feed_sent"]:
        _overlap_poll_feed(budget_s)


def _overlap_send_feed_blocking() -> bool:
    """Envía feed_stage y procesa respuesta (hilo o espera antes del paso 4)."""
    if _OVERLAP["feed_done"]:
        return True
    if _cycle_should_stop():
        return False
    if _OVERLAP["holder_error"] and not _OVERLAP["holder_ready"].is_set():
        _cycle_fail(_OVERLAP["holder_error"] if _OVERLAP["holder_error"] != "stop" else "E010")
        return False
    if not _OVERLAP["holder_ready"].wait(3.0):
        if _cycle_should_stop():
            return False
        _cycle_fail(_OVERLAP["holder_error"] or "E010")
        return False
    if _OVERLAP["feed_sent"]:
        _overlap_poll_feed(2.0)
        return bool(_OVERLAP["feed_done"])
    _OVERLAP["feed_sent"] = True
    _OVERLAP["feed_t0"] = time.perf_counter()
    _set_machine_ui(paralelo_id="3")
    next_sides = str(_OVERLAP.get("sides") or "") or _lot_predict_next_cycle_sides()
    _OVERLAP["sides"] = next_sides
    payload = _feed_stage_payload(next_sides if next_sides in ("R", "L") else None)
    sent = _cycle_motion_send(payload, timeout_s=65.0)
    if _cycle_should_stop():
        return False
    if not sent.get("sent"):
        _cycle_fail(sent.get("code") or "E043")
        return False
    return _overlap_accept_feed(sent.get("replies") or [])


def _overlap_start_feed_task() -> None:
    """Alimentación del siguiente ciclo sin bloquear el paso 8 en curso."""
    if (
        not _OVERLAP["on"]
        or _OVERLAP["feed_done"]
        or _OVERLAP["feed_sent"]
        or _OVERLAP["feed_task_started"]
    ):
        return
    _OVERLAP["feed_task_started"] = True
    gen = int(_OVERLAP["gen"])
    worker_gen = getattr(_CYCLE_WORKER_TLS, "gen", None)

    def run() -> None:
        if worker_gen is not None:
            _CYCLE_WORKER_TLS.gen = worker_gen
        try:
            if _OVERLAP["gen"] != gen or _cycle_should_abort():
                return
            _overlap_send_feed_blocking()
        finally:
            if not _OVERLAP["feed_sent"] and not _OVERLAP["feed_done"]:
                _OVERLAP["feed_task_started"] = False

    threading.Thread(target=run, daemon=True).start()


def _overlap_wait_feed_done(timeout_s: float = 120.0) -> bool:
    """Espera la alimentación solapada antes de entregar el paso 4 al siguiente ciclo."""
    if not _OVERLAP["on"]:
        return True
    if _OVERLAP["feed_error"]:
        _overlap_apply_feed_error()
        return False
    if _OVERLAP["feed_done"]:
        return True
    total = int(_LOT["cycles_total"])
    nxt = int(_LOT["cycles_done"]) + 1
    _set_machine_ui(
        estado="running",
        estado_label="Ciclo en marcha",
        mensaje=(
            _cycle_piece_prefix()
            + f"Pieza {nxt} de {total}: esperando alimentación (paso 3 en paralelo)…"
        ),
        en_curso=True,
    )
    if not _OVERLAP["feed_sent"] and not _OVERLAP["feed_task_started"]:
        _overlap_start_feed_task()
    deadline = time.time() + timeout_s
    blocking_after = time.time() + 0.75
    while time.time() < deadline:
        if _cycle_should_abort():
            return False
        if _OVERLAP["feed_done"]:
            return True
        if _OVERLAP["feed_error"]:
            _overlap_apply_feed_error()
            return False
        _overlap_poll_feed(1.0)
        if not _OVERLAP["feed_done"] and not _OVERLAP["feed_sent"]:
            if not _OVERLAP["feed_task_started"]:
                _overlap_start_feed_task()
            elif time.time() >= blocking_after:
                if not _overlap_send_feed_blocking():
                    if _OVERLAP["feed_error"]:
                        _overlap_apply_feed_error()
                        return False
                    if _MACHINE_STATE.get("ciclo_error") or _cycle_should_abort():
                        return False
                blocking_after = time.time() + 2.0
        time.sleep(0.05)
    _cycle_fail("E038", detail="feed_overlap_timeout")
    return False


def _overlap_handoff() -> tuple[int, float | None]:
    """Paso 8 ya acabó (home, pinzas abiertas) y los pasos 1 a 3 también."""
    if not _OVERLAP["feed_done"]:
        return 0, None
    global _PIECE_BLOWER_MM, _PIECE_FEED, _PIECE_ENC_AT_LM, _STEP4_CLOSES
    mm = _OVERLAP["blower_mm"]
    t0 = _OVERLAP["t0"]
    feed = _OVERLAP["feed"]
    next_sides = str(_OVERLAP.get("sides") or "") or _lot_next_cycle_sides()
    _overlap_reset()
    if _LOT.get("active") and next_sides:
        _LOT["cycle_sides"] = next_sides
    _STEP4_CLOSES = True
    _PIECE_ENC_AT_LM = {"R": None, "L": None}
    if mm is not None:
        _PIECE_BLOWER_MM = mm
    if feed is not None:
        _PIECE_FEED = {"R": feed.get("R"), "L": feed.get("L")}
        _set_machine_ui(feed=feed)
    _push_event("Home con pinzas abiertas. El paso 4 cierra y sigue.")
    return 3, t0


def _finish_piece_idle(mensaje: str) -> None:
    global _LAST_PIECE_S, _PIECE_TICK0, _LOT_T0
    lot_elapsed = 0.0
    if _LOT_T0 is not None:
        lot_elapsed = max(0.0, time.time() - _LOT_T0)
    elif _LAST_PIECE_S:
        lot_elapsed = float(_LAST_PIECE_S)
    lot_done = _lot_targets_met() or (
        int(_LOT.get("qty_total") or 0) > 0
        and _lot_pieces_done_total() >= int(_LOT.get("qty_total") or 0)
    )
    _LOT["active"] = False
    _LOT["tray_wait"] = False
    _LOT["home_before"] = False
    _CYCLE_CTRL["step_mode"] = False
    _PIECE_TICK0 = None
    _LOT_T0 = None
    _set_machine_ui(
        estado="idle",
        estado_label="En espera",
        mensaje=mensaje,
        en_curso=False,
        ciclo_error="",
        step_id="",
        step_index=len(CYCLE_STEPS) if lot_done else 0,
        progreso_pct=100 if lot_done else _lot_progress_pct(0),
        tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
        tiempo_corriendo_s=round(lot_elapsed, 1) if lot_done else 0,
        paralelo_id="",
    )
    with _STATE_LOCK:
        _MACHINE_STATE["tiempo_estimado_s"] = 0 if lot_done else None
    _overlap_reset()
    _cycle_latch_reset()
    _prefeeder_lock_after_cycle_async()


def _enter_tray_wait(deposits: int) -> None:
    _LOT["tray_wait"] = True
    _cadence_mark_interrupted()
    lot_elapsed = 0.0
    if _LOT_T0 is not None:
        lot_elapsed = max(0.0, time.time() - _LOT_T0)
    primed = " Material ya alimentado." if _feed_prime_ready() else ""
    _set_machine_ui(
        estado="paused",
        estado_label="Charola llena",
        mensaje=(
            f"Charola llena — {deposits} ciclos. Vacía la charola y confirma.{primed}"
        ),
        en_curso=True,
        step_id="",
        step_index=0,
        progreso_pct=_lot_progress_pct(0),
        tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
        tiempo_corriendo_s=round(lot_elapsed, 1),
        paralelo_id="",
    )
    _push_event(f"Charola llena — {deposits} ciclos.", "warn")
    _overlap_reset()
    _cycle_latch_reset()


def _home_after_tray() -> bool:
    _LOT["home_before"] = False
    _set_machine_ui(
        estado="running",
        estado_label="Ciclo en marcha",
        mensaje="Charola vacía. Regresando el eje a cero.",
        en_curso=True,
    )
    sent = _cycle_motion_send(
        {"op": "lmguide_move", "target": "home"},
        timeout_s=_MOTION_LM_MOVE_TIMEOUT_S,
    )
    if _motion_cmd_serial_ok(sent, "E043"):
        return True
    _LOT["tray_wait"] = True
    _push_event("El eje no volvió a cero. Confirma otra vez cuando pueda moverse.", "error")
    _set_machine_ui(
        estado="paused",
        estado_label="Charola llena",
        mensaje="El eje no volvió a cero. Vacía la charola y confirma otra vez.",
        en_curso=True,
    )
    return False


def _piece_log_name() -> str:
    if _LOT["active"]:
        r, l = _lot_display_pieces(preview=True)
        qty = int(_LOT.get("qty_total") or 0)
        sides = _lot_cycle_sides_now()
        return f"Pieza QTY {qty:03d} | R{r:03d}/L{l:03d} ({sides})"
    msg = str(_MACHINE_STATE.get("mensaje") or "").rstrip(".")
    if msg.startswith("Pieza de prueba"):
        return msg
    return "Pieza"


def _run_one_piece(start_index: int, started_at: float | None = None) -> bool:
    global _PROD_CTX, _PIECE_TICK0, _CADENCE_RECORDED
    global _PIECE_FEED, _PIECE_ENC_AT_LM, _CYCLE_SET_DATA_ON_HOME
    params = _load_cycle_params()
    _CYCLE_SET_DATA_ON_HOME = False
    # Régimen = handoff con solape; la cadencia ya ancla en Gripper OFF (perf_counter).
    regime = bool(started_at is not None and start_index > 0 and _PREV_PIECE_END_T is not None)
    fresh = start_index <= 0 and not regime
    # Índice 6 = paso 7. Si se reanuda en paso 8, conservar cadencia ya registrada.
    if start_index <= 6:
        _CADENCE_RECORDED = False
    if not regime:
        # Conservar _PIECE_TICK0 de Start / reanudación; no mezclar con perf_counter.
        if _PIECE_TICK0 is None:
            _PIECE_TICK0 = time.time()
    piece_wall0 = _PIECE_TICK0 if _PIECE_TICK0 is not None else time.time()
    piece_num = int(_LOT["cycles_done"]) + 1 if _LOT.get("active") else 1
    name = _piece_log_name()
    if fresh:
        _PIECE_FEED = {"R": None, "L": None}
        _PIECE_ENC_AT_LM = {"R": None, "L": None}
        _push_event(f"{name} inicia.")
        # Arranque fresco: primero PUU 0 absoluto. Si el eje quedó a media
        # carrera tras un abort, no se debe tratar el corte como offset relativo.
        _set_machine_ui(
            estado="running",
            estado_label="Ciclo en marcha",
            mensaje=_cycle_piece_prefix() + "LM Guide a home (absoluto).",
            en_curso=True,
            step_id="1",
            step_index=0,
        )
        if not _cycle_lm_home_absolute(with_set_data=True):
            return False
    for idx in range(start_index, len(CYCLE_STEPS)):
        step = CYCLE_STEPS[idx]
        step_id = step["id"]
        t_step = time.perf_counter()
        _PROD_CTX = {
            "piece": piece_num,
            "step_id": step_id,
            "t0": t_step,
            "parallel": False,
        }
        if _cycle_should_abort():
            return False
        if not _joint_check_during_cycle(step_id):
            return False
        _cycle_enter_step(step_id, idx)
        if not _cycle_assert_preconditions(idx):
            return False
        ok = _cycle_run_step(step_id, params)
        if _cycle_error_active():
            return False
        step_s = time.perf_counter() - t_step
        if ok:
            _prod_log_op(step_id, step_s, piece=piece_num, parallel=False)
        elif not _MACHINE_STATE.get("ciclo_error") and not _joint_active():
            # Joint ya se registró como WARNING en _joint_begin_common.
            detail = str(_MACHINE_STATE.get("mensaje") or "falló")
            _prod_log_op(
                step_id,
                step_s,
                piece=piece_num,
                status="ERROR",
                detail=detail,
            )
        if not ok:
            if _MACHINE_STATE.get("ciclo_error"):
                return False
            return False
        _cycle_latch_mark_step_done(idx)
        # Freeze de pausa intra-pieza: pared al inicio de esta pieza (no cadencia).
        elapsed = time.time() - piece_wall0
        _set_machine_ui(tiempo_corriendo_s=round(elapsed, 1))
        if _CYCLE_CTRL["pause_after_step"].is_set():
            _CYCLE_CTRL["pause_after_step"].clear()
            _cadence_mark_interrupted()
            next_idx = idx + 1
            # En paso a paso, el último paso cierra la pieza sin pedir otro avance.
            if next_idx < len(CYCLE_STEPS):
                next_step = CYCLE_STEPS[next_idx]["id"]
                step_mode = bool(_CYCLE_CTRL.get("step_mode"))
                if step_mode:
                    nxt = CYCLE_STEPS[next_idx]
                    msg = (
                        f"Paso {step_id} listo. Siguiente: {nxt['id']} — {nxt['label']}. "
                        "Pulsa «Siguiente paso»."
                    )
                    label = "Paso a paso"
                    event = f"Paso a paso — paso {step_id} completado."
                else:
                    msg = "Ciclo en pausa. Reanudar para continuar."
                    label = "Pausa"
                    event = "Pausa — paso actual completado."
                pause_elapsed = round(max(0.0, elapsed), 1)
                _set_machine_ui(
                    estado="paused",
                    estado_label=label,
                    mensaje=msg,
                    en_curso=True,
                    step_id=next_step,
                    step_index=next_idx,
                    tiempo_corriendo_s=pause_elapsed,
                )
                _push_event(event, "warn")
                _prefeeder_process_halt_async()
                return False
    # Cadencia ya medida en Gripper OFF (paso 7). No usar el fin del paso 8.
    credited = ()
    if _LOT.get("active"):
        credited = _lot_credit_completed_piece()
    if credited:
        _push_event(
            f"{name} cuenta "
            + ("+".join(credited) if credited else "0")
            + f" → R{_LOT.get('pieces_r', 0)}/L{_LOT.get('pieces_l', 0)}."
        )
    if _CADENCE_RECORDED and _LAST_PIECE_S > 0:
        tag = " (régimen)" if regime else (" (primera)" if fresh else "")
        _push_event(f"{name} termina en {_LAST_PIECE_S:.1f} s{tag}.")
    else:
        _push_event(f"{name} termina.")
    return True


def _cycle_worker(start_index: int = 0, gen: int = 0) -> None:
    _CYCLE_WORKER_TLS.gen = int(gen)
    lot_mode = bool(_LOT["active"])
    index = start_index
    started_at = None
    try:
        while True:
            if not _cycle_worker_is_current(gen) or _cycle_should_abort():
                return
            if _LOT["home_before"] and not _home_after_tray():
                return
            # Tras charola: si hay material adelantado, saltar a paso 4.
            if index <= 0 and started_at is None and _feed_prime_consume():
                index = 3
            if not _run_one_piece(index, started_at):
                return
            _mant_add_cycle()
            if not lot_mode:
                if (
                    _joint_active()
                    and str(_recovery_get().get("origin") or "") == "fabricate"
                    and _JOINT.get("finish_piece")
                ):
                    _joint_enter_operator_after_piece()
                    return
                if _cycle_worker_is_current(gen):
                    _finish_piece_idle("Ciclo completado. Listo para siguiente Start.")
                return
            if not _cycle_worker_is_current(gen) or _cycle_should_abort() or not _LOT["active"]:
                return
            _LOT["cycles_done"] += 1
            done = _LOT["cycles_done"]
            total = _LOT["cycles_total"]
            deposits = int(_LOT["deposits_per_tray"] or 0)
            # Catch-up: si un lado quedó atrás, el siguiente ciclo solo ese lado.
            _LOT["cycle_sides"] = _lot_next_cycle_sides()
            _set_machine_ui(
                progreso_pct=_lot_progress_pct(0),
                tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
            )
            if (
                _joint_active()
                and str(_recovery_get().get("origin") or "") == "fabricate"
                and _JOINT.get("finish_piece")
            ):
                _joint_enter_operator_after_piece()
                return
            if _lot_targets_met():
                avg_cadence = (
                    sum(_PROD_CADENCE_SAMPLES) / len(_PROD_CADENCE_SAMPLES)
                    if _PROD_CADENCE_SAMPLES
                    else 0.0
                )
                _prod_log_summary(done, total, avg_cadence)
                pr = int(_LOT.get("pieces_r") or 0)
                pl = int(_LOT.get("pieces_l") or 0)
                qty = int(_LOT.get("qty_total") or 0)
                if _CYCLE_CTRL.get("step_mode"):
                    _push_event("Prueba paso a paso terminada.")
                    _finish_piece_idle("Prueba paso a paso terminada.")
                else:
                    _feed_prime_prepare_after_segment()
                    primed = " Material listo para el siguiente Start." if _feed_prime_ready() else ""
                    _push_event(f"Lote terminado. QTY {qty} | R{pr}/L{pl}.")
                    _finish_piece_idle(
                        f"Lote terminado. QTY {qty} | R{pr}/L{pl}.{primed}"
                    )
                return
            if deposits and done % deposits == 0:
                _feed_prime_prepare_after_segment()
                _enter_tray_wait(deposits)
                return
            _cycle_latch_reset()
            if not _overlap_wait_feed_done():
                return
            if not _cycle_worker_is_current(gen) or _cycle_should_abort():
                return
            index, started_at = _overlap_handoff()
            if _LOT.get("cycle_sides") and _LOT["cycle_sides"] != _LOT.get("sides"):
                _push_event(
                    f"Catch-up lado {_LOT['cycle_sides']}: "
                    f"R{_LOT.get('pieces_r', 0)}/L{_LOT.get('pieces_l', 0)}.",
                    "warn",
                )
            if index == 0:
                _set_machine_ui(
                    feed={"R": None, "L": None},
                    tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
                    tiempo_corriendo_s=0,
                    paralelo_id="",
                )
            else:
                _set_machine_ui(
                    tiempo_ciclo_s=round(_LAST_PIECE_S, 1),
                    tiempo_corriendo_s=0,
                    paralelo_id="",
                )
    except Exception as exc:
        if _cycle_worker_is_current(gen):
            _cycle_motion_halt_for_error()
            if not _cycle_error_active():
                _cycle_fail(
                    "E000",
                    mensaje=f"Ciclo abortado por excepción interna: {exc}",
                )
    finally:
        # Solo el worker oficial limpia stop/lote. Cualquier hilo se quita de
        # thread si es el registrado; un huerfano no toca el control del nuevo.
        with _CYCLE_SPAWN_LOCK:
            is_self = _CYCLE_CTRL.get("thread") is threading.current_thread()
            is_owner = _cycle_worker_is_current(gen) and is_self
            if is_self:
                _CYCLE_CTRL["thread"] = None
                _CYCLE_CTRL["pause_after_step"].clear()
            if not is_owner:
                return
            stop_req = _CYCLE_CTRL["stop"].is_set()
            if stop_req:
                _CYCLE_CTRL["stop"].clear()
                _CYCLE_CTRL["step_mode"] = False
        if stop_req:
            _cycle_stop_idle()
        else:
            # Pausa o error: el lote permanece. Si murio en running sin error, latch E043.
            with _STATE_LOCK:
                stuck_running = (
                    bool(_MACHINE_STATE.get("ciclo_en_curso") or _LOT.get("active"))
                    and _MACHINE_STATE.get("estado") == "running"
                    and not str(_MACHINE_STATE.get("ciclo_error") or "").strip()
                )
            if stuck_running and _cycle_worker_is_current(gen):
                _cycle_fail(
                    "E043",
                    mensaje="Ciclo detenido sin respuesta del Motion. Revisa COM o el firmware.",
                )


def _cycle_blocked_by_routine() -> str:
    if not _checklist_bypassed() and not (_MACHINE_STATE.get("checklist") or {}).get("ok"):
        return "Completa la revisión de arranque antes de Iniciar."
    if _MACHINE_STATE.get("post_error_purga"):
        return "Tras un error: Reset y completa la purga antes de Iniciar o Reanudar."
    if _MACHINE_STATE.get("rutina") == "purga":
        return "Hay una purga en curso."
    if _joint_active() and str(_recovery_get().get("step") or "") != "listo":
        return "Completa la recuperación Joint antes de Iniciar."
    return _motion_owner_busy()


def _cycle_start(*, step_mode: bool = False) -> tuple[bool, str]:
    blocked = _cycle_blocked_by_routine()
    if blocked:
        return False, blocked
    if _LOT["tray_wait"]:
        return False, "La charola está llena. Vacía y confirma, o pulsa Detener."
    modelo_id = str(_MOTION_DATA.get("modelo_id") or _MACHINE_STATE.get("numero_modelo") or "").strip()
    length = _target_length_mm()
    if not modelo_id or length <= 0 or not _find_modelo(modelo_id):
        return False, "Selecciona un modelo antes de Start."
    params = _load_cycle_params()
    sides = _feed_sides_of(params)
    if step_mode:
        # Una sola pieza: con ambos lados basta cantidad 2 para validar el pedido.
        qty = 2 if sides == "LR" else 1
        cycles, qty_err = _order_cycles(qty, sides)
    else:
        try:
            qty = int(_MACHINE_STATE.get("cantidad") or 0)
        except (TypeError, ValueError):
            qty = 0
        cycles, qty_err = _order_cycles(qty, sides)
    if qty_err:
        return False, qty_err
    if step_mode:
        cycles = 1
    plan, plan_err = _tray_plan(length, params)
    if plan_err:
        return False, plan_err
    ready, link_err = _links_ready_for_cycle()
    if not ready:
        return False, link_err
    ready, motion_err = _motion_ready_for_cycle()
    if not ready:
        return False, motion_err
    if _prefeeder_required():
        st = _prefeeder_refresh_cache()
        if not st:
            return False, error_codes.format_message("E092")
        if not _prefeeder_status_ready(st):
            return False, _prefeeder_not_ready_message(st)
    # Joint ya resuelto (step=listo) o latch huérfano: no arrastrar skip_cut al lote nuevo.
    rec0 = _recovery_get()
    if rec0.get("kind") == "joint" and str(rec0.get("step") or "") == "listo":
        _recovery_clear()
    else:
        _joint_latch_clear()
    if not step_mode:
        _announce_trays(cycles, plan)
    global _LOT_SEQ, _PROD_CADENCE_SAMPLES, _PREV_PIECE_END_T, _PIECE_TICK0, _LOT_T0
    global _CADENCE_T0, _CADENCE_INTERRUPTED, _CADENCE_RECORDED
    with _CYCLE_SPAWN_LOCK:
        if _cycle_thread_alive():
            return False, "Hay un ciclo en ejecución."
        if _LOT["active"]:
            return False, "Hay un lote en curso. Reanuda o pulsa Detener."
        _LOT_SEQ += 1
        _PROD_CADENCE_SAMPLES = []
        _CADENCE_INTERRUPTED = False
        _CADENCE_RECORDED = False
        _CADENCE_T0 = time.perf_counter()
        _LOT["active"] = True
        _LOT["log_id"] = _LOT_SEQ
        _LOT["tray_wait"] = False
        _LOT["home_before"] = False
        _LOT["cycles_total"] = cycles
        _LOT["cycles_done"] = 0
        _lot_init_piece_targets(qty, sides)
        _LOT["stacks"] = plan["stacks"]
        _LOT["deposits_per_tray"] = plan["deposits"]
        _LOT["batch_size"] = plan["batch_size"]
        _LOT["cut_mm"] = plan["cut_mm"]
        cut_comps = _cut_comp_params_of(params)
        _LOT["cut_offset_mm"] = 0.0
        _LOT["cut_comp_mode"] = True
        _LOT["cut_comp_r_mm"] = cut_comps["cut_comp_r_mm"]
        _LOT["cut_comp_l_mm"] = cut_comps["cut_comp_l_mm"]
        _LOT["length_mm"] = plan["length_mm"]
        _LOT["gap_mm"] = plan["gap_mm"]
        _LOT["clearance_mm"] = plan["clearance_mm"]
        _CYCLE_CTRL["stop"].clear()
        _CYCLE_CTRL["step_mode"] = bool(step_mode)
        if step_mode:
            _CYCLE_CTRL["pause_after_step"].set()
        else:
            _CYCLE_CTRL["pause_after_step"].clear()
        _overlap_reset()
        _cycle_latch_reset()
        _PREV_PIECE_END_T = None
        _LOT_T0 = time.time()
        _PIECE_TICK0 = _LOT_T0
        start_idx = 0
        use_prime = (not step_mode) and _feed_prime_consume()
        if use_prime:
            start_idx = 3
        if step_mode:
            ui_estado = "running"
            ui_label = "Paso a paso"
            ui_msg = "Prueba paso a paso — ejecutando paso 1."
            event_msg = "Prueba paso a paso iniciada."
        elif use_prime:
            ui_estado = "running"
            ui_label = "Ciclo en marcha"
            ui_msg = "Material listo. Cierra pinzas y continúa."
            event_msg = None
        else:
            ui_estado = "running"
            ui_label = "Ciclo en marcha"
            ui_msg = "Secuencia automática iniciada."
            event_msg = None
        est0 = None
        avg0 = _lot_avg_cycle_s()
        if avg0 is not None and cycles > 0:
            est0 = round(cycles * avg0, 1)
        feed_ui = (
            {"R": (_PIECE_FEED or {}).get("R"), "L": (_PIECE_FEED or {}).get("L")}
            if use_prime
            else {"R": None, "L": None}
        )
        _set_machine_ui(
            estado=ui_estado,
            estado_label=ui_label,
            mensaje=ui_msg,
            en_curso=True,
            ciclo_error="",
            feed=feed_ui,
            step_id="4" if use_prime else "1",
            step_index=start_idx,
            progreso_pct=_cycle_progress_pct(start_idx) if use_prime else 0,
            tiempo_ciclo_s=None,
            tiempo_corriendo_s=0,
        )
        with _STATE_LOCK:
            _MACHINE_STATE["tiempo_estimado_s"] = est0
        ok, err = _cycle_spawn_worker(start_idx, locked=True)
        if not ok:
            _LOT["active"] = False
            _PIECE_TICK0 = None
            _LOT_T0 = None
            return False, err
    _prefeeder_unlock_for_cycle()
    if event_msg:
        _push_event(event_msg)
    return True, ""


def _cycle_pause() -> None:
    if _MACHINE_STATE.get("rutina"):
        _push_event("No se puede pausar durante una rutina.", "warn")
        return
    if not (_CYCLE_CTRL["thread"] and _CYCLE_CTRL["thread"].is_alive()):
        _push_event("No hay ciclo en marcha para pausar.", "warn")
        return
    _CYCLE_CTRL["pause_after_step"].set()
    _set_machine_ui(
        estado="running",
        estado_label="Paso a paso" if _CYCLE_CTRL.get("step_mode") else "Ciclo en marcha",
        mensaje="Pausa solicitada. Terminando el paso en curso…",
    )
    _push_event("Pausa solicitada — se detendrá al completar el paso.", "warn")


def _cycle_resume() -> tuple[bool, str]:
    if _LOT["tray_wait"]:
        return False, "La charola está llena. Vacía y confirma."
    if _MACHINE_STATE.get("ciclo_error"):
        return False, "Hay un error activo. Usa Reset cuando la condición se haya corregido."
    if _joint_active() and str(_recovery_get().get("step") or "") != "listo":
        return False, "Completa la recuperación Joint (Finish / Abrir) antes de Reanudar."
    if _MACHINE_STATE.get("post_error_purga"):
        return False, "Tras un error debes completar la purga antes de Reanudar."
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "Hay una purga en curso."
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    # Joint listo / latch huérfano: no reanudar con skip_cut activo.
    rec_r = _recovery_get()
    if rec_r.get("kind") == "joint" and str(rec_r.get("step") or "") == "listo":
        _recovery_clear()
    else:
        _joint_latch_clear()
    idx = int(_MACHINE_STATE.get("ciclo_paso_indice") or 0)
    # Tras error/purga active puede haberse perdido con el pedido intacto.
    # Reactivar sin _lot_init (no borrar pieces_r/l). Nunca lanzar worker sin lote.
    lot_open = _lot_ensure_active_for_resume()
    en_curso = bool(_MACHINE_STATE.get("ciclo_en_curso"))
    # Sin lote recuperable: Start limpio. Con lote: continuar contadores.
    if not lot_open:
        return _cycle_start()
    if not en_curso:
        en_curso = True
    ready, link_err = _links_ready_for_cycle()
    if not ready:
        return False, link_err
    if idx >= len(CYCLE_STEPS):
        idx = len(CYCLE_STEPS) - 1
    if idx < 0:
        idx = 0
    step_mode = bool(_CYCLE_CTRL.get("step_mode"))
    global _PIECE_TICK0
    # Tras error+purga el reloj de pieza arranca en 0 (reintento).
    # En pausa limpia se conserva el tiempo congelado.
    # Si ya hubo Gripper OFF, no pisar la ancla de cadencia/estimación.
    frozen = float(_MACHINE_STATE.get("tiempo_corriendo_s") or 0)
    with _CYCLE_SPAWN_LOCK:
        if _cycle_thread_alive():
            return False, "Hay un ciclo en ejecución."
        _CYCLE_CTRL["stop"].clear()
        if step_mode:
            _CYCLE_CTRL["pause_after_step"].set()
        else:
            _CYCLE_CTRL["pause_after_step"].clear()
        if not _CADENCE_RECORDED:
            _PIECE_TICK0 = time.time() - max(0.0, frozen)
        ok, err = _cycle_spawn_worker(idx, locked=True)
        if not ok:
            return False, err
    _prefeeder_unlock_for_cycle()
    if step_mode:
        step = CYCLE_STEPS[idx] if 0 <= idx < len(CYCLE_STEPS) else None
        sid = step["id"] if step else "?"
        label = step["label"] if step else ""
        _set_machine_ui(
            estado="running",
            estado_label="Paso a paso",
            mensaje=f"Ejecutando paso {sid}" + (f" — {label}." if label else "."),
            en_curso=True,
        )
        _push_event(f"Paso a paso — ejecutando paso {sid}.")
    else:
        _set_machine_ui(
            estado="running",
            estado_label="Ciclo en marcha",
            mensaje="Reanudando secuencia.",
            en_curso=True,
        )
        _push_event(
            "Ciclo reanudado. "
            f"QTY {int(_LOT.get('qty_total') or 0)} | "
            f"R{int(_LOT.get('pieces_r') or 0):03d}/L{int(_LOT.get('pieces_l') or 0):03d}."
        )
    return True, ""


def _cycle_step_start() -> tuple[bool, str]:
    """Inicia una pieza en modo prueba: pausa al completar cada paso."""
    if _cycle_thread_alive() or _MACHINE_STATE.get("ciclo_en_curso"):
        return False, "Hay un ciclo en curso. Deténlo antes de la prueba paso a paso."
    ok, err = _cycle_start(step_mode=True)
    if not ok:
        return False, err
    return True, "Prueba paso a paso iniciada."


def _cycle_step_next() -> tuple[bool, str]:
    """Avanza un paso en la prueba paso a paso (equivalente a Reanudar con pausa armada)."""
    if not _CYCLE_CTRL.get("step_mode"):
        return False, "No hay una prueba paso a paso activa."
    if _MACHINE_STATE.get("estado") != "paused" or not _MACHINE_STATE.get("ciclo_en_curso"):
        return False, "Espera a que termine el paso en curso."
    idx = int(_MACHINE_STATE.get("ciclo_paso_indice") or 0)
    if idx >= len(CYCLE_STEPS):
        _CYCLE_CTRL["step_mode"] = False
        _finish_piece_idle("Prueba paso a paso terminada.")
        return True, "Prueba paso a paso terminada."
    ok, err = _cycle_resume()
    if not ok:
        return False, err
    return True, "Siguiente paso."


def _charola_vacia(_data=None) -> tuple[bool, str]:
    if not _LOT["tray_wait"] or not _LOT["active"]:
        return False, "La charola no está en espera."
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    ready, link_err = _links_ready_for_cycle()
    if not ready:
        return False, link_err
    with _CYCLE_SPAWN_LOCK:
        if _cycle_thread_alive():
            return False, "Hay un ciclo en ejecución."
        _LOT["tray_wait"] = False
        _LOT["home_before"] = True
        _CYCLE_CTRL["stop"].clear()
        _CYCLE_CTRL["pause_after_step"].clear()
        _cycle_latch_reset()
        _set_machine_ui(
            estado="running",
            estado_label="Ciclo en marcha",
            mensaje="Charola vacía. Regresando el eje a cero.",
            en_curso=True,
        )
        ok, err = _cycle_spawn_worker(0, locked=True)
        if not ok:
            _LOT["tray_wait"] = True
            _LOT["home_before"] = False
            return False, err
    _prefeeder_unlock_for_cycle()
    return True, "Charola vacía. El eje vuelve a cero y el siguiente corte sale del primer montón."


def _cycle_stop_idle(mensaje: str = "Detener — ciclo detenido. Listo para Iniciar.") -> None:
    """Solo Detener (o el finally del hilo tras Detener) debe llegar aquí y borrar el lote."""
    global _STEP4_CLOSES, _PIECE_TICK0, _LOT_T0
    global _PREV_PIECE_END_T, _CADENCE_T0, _CADENCE_INTERRUPTED, _CADENCE_RECORDED
    _STEP4_CLOSES = False
    _LOT["active"] = False
    _LOT["tray_wait"] = False
    _LOT["home_before"] = False
    _LOT["cycles_done"] = 0
    _LOT["cycles_total"] = 0
    _LOT["qty_total"] = 0
    _LOT["pieces_r"] = 0
    _LOT["pieces_l"] = 0
    _LOT["target_r"] = 0
    _LOT["target_l"] = 0
    _LOT["cycle_sides"] = _LOT.get("sides") or "R"
    _CYCLE_CTRL["step_mode"] = False
    _PIECE_TICK0 = None
    _LOT_T0 = None
    _PREV_PIECE_END_T = None
    _CADENCE_T0 = None
    _CADENCE_INTERRUPTED = False
    _CADENCE_RECORDED = False
    _overlap_reset()
    _cycle_latch_reset()
    _feed_prime_clear()
    # Stop cancela lote y Joint: recovery + skip_cut (si no, el próximo corte se omite).
    with _STATE_LOCK:
        _MACHINE_STATE["post_error_purga"] = False
    _recovery_clear()
    _set_machine_ui(
        estado="idle",
        estado_label="En espera",
        mensaje=mensaje,
        en_curso=False,
        ciclo_error="",
        step_id="",
        step_index=0,
        progreso_pct=0,
        tiempo_corriendo_s=0,
        paralelo_id="",
        feed={"R": None, "L": None},
    )
    with _STATE_LOCK:
        _MACHINE_STATE["tiempo_estimado_s"] = None


def _cycle_stop() -> None:
    """Única acción de operador que cancela el lote en curso."""
    with _CYCLE_SPAWN_LOCK:
        th = _CYCLE_CTRL.get("thread")
        running = th is not None and th.is_alive()
        # Invalida workers oficiales y huerfanos: gen cambia → abort inmediato.
        _CYCLE_CTRL["gen"] = int(_CYCLE_CTRL.get("gen") or 0) + 1
        _CYCLE_CTRL["stop"].set()
        _CYCLE_CTRL["pause_after_step"].clear()
        _CYCLE_CTRL["step_mode"] = False
    _ROUTINE["stop"].set()
    if _serial_port("motion"):
        _send_motion_urgent(
            ['{"op":"feeder_stop"}', '{"op":"lmguide_stop"}', '{"op":"blower_off"}']
        )
    if _MACHINE_STATE.get("rutina") == "purga" and not (
        _ROUTINE["thread"] and _ROUTINE["thread"].is_alive()
    ):
        _purga_finish(True)
    _cycle_stop_idle()
    if _prefeeder_required():
        _prefeeder_stop_now()
    _push_event("Detener — ciclo abortado.", "warn")
    if running and th is not None:
        th.join(timeout=2.0)
    with _CYCLE_SPAWN_LOCK:
        if not _cycle_thread_alive():
            _CYCLE_CTRL["stop"].clear()
            if _CYCLE_CTRL.get("thread") is th:
                _CYCLE_CTRL["thread"] = None


def _error_physical_clear_now() -> bool:
    """True si no hay fallas PLC ni seguridad Motion activas (lectura puntual)."""
    plc_port = bool(_serial_port("plc"))
    motion_port = bool(_serial_port("motion"))
    if plc_port:
        st = _plc_status_get()
        if st is None:
            return False
        if _plc_fault_codes_ops(st):
            return False
    if motion_port:
        mst = _motion_status_get()
        if mst is None:
            return False
        if mst.get("safety") is True:
            return False
    if not _prefeeder_fault_clear_now():
        return False
    return True


def _wait_error_physical_clear_stable() -> bool:
    """Condiciones despejadas durante _ERROR_RESET_STABLE_MS seguidos."""
    need_s = max(0.05, _ERROR_RESET_STABLE_MS / 1000.0)
    clear_since = None
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if not _error_physical_clear_now():
            clear_since = None
        else:
            now = time.time()
            if clear_since is None:
                clear_since = now
            elif now - clear_since >= need_s:
                return True
        time.sleep(0.05)
    return False


def _cycle_reset_error() -> tuple[bool, str]:
    global _pf_last_fault_sig, _PF_ERROR_LATCH
    with _STATE_LOCK:
        code = str(_MACHINE_STATE.get("ciclo_error") or "").strip()
        in_err = _MACHINE_STATE.get("estado") == "error" or bool(code)
        msg_now = str(_MACHINE_STATE.get("mensaje") or "")
    if not in_err:
        return True, ""
    def _keep_lot() -> bool:
        return bool(
            _MACHINE_STATE.get("ciclo_en_curso")
            or _LOT.get("active")
            or _LOT.get("tray_wait")
        )

    def _finish_reset_to_purge(keep: bool) -> tuple[bool, str]:
        # Ruta especial: tras Reset la purga es obligatoria antes de Reanudar/Start.
        # El reset PLC apaga válvulas lógicas: libera holder/pinzas (suelta la pieza).
        _gripper_holding_set(False)
        _feed_prime_clear()
        err_fixed = ""
        with _STATE_LOCK:
            _MACHINE_STATE["post_error_purga"] = True
            rec = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
            prev = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
            err_fixed = str(
                rec.get("mensaje")
                or msg_now
                or prev.get("mensaje")
                or ""
            ).strip()
            # Si el banner ya era corto, recuperar descripción del catálogo.
            if err_fixed.endswith("Abre el aviso para continuar.") or not err_fixed:
                c = str(rec.get("code") or code or "").strip()
                if c:
                    try:
                        err_fixed = error_codes.format_message(c)
                    except Exception:
                        err_fixed = c
            _MACHINE_STATE["recovery"] = {
                "active": True,
                "step": "piezas",
                "kind": "",
                "side": "",
                "origin": "",
                "busy": False,
                "lm_at_park": False,
                "code": str(rec.get("code") or code or "").strip(),
                "mensaje": err_fixed,
                "guide": "Confirma que las piezas están bien.",
            }
        code_show = str(_recovery_get().get("code") or code or "").strip()
        _set_machine_ui(
            estado="paused" if keep else "idle",
            estado_label="Recuperación",
            ciclo_error="",
            mensaje=(
                f"{code_show}. Abre el aviso para continuar."
                if code_show
                else "Abre el aviso para continuar."
            ),
            en_curso=keep,
        )
        _push_event("Reset — holder y pieza liberados; sigue la recuperación.")
        return True, ""

    # Error de COM/enlace: no pedir pulso/status al módulo caído.
    if (code in _RECOVERY_LINK_SOFT_RESET) or (not code and _MACHINE_STATE.get("estado") == "error"):
        _pf_last_fault_sig = ()
        _PF_ERROR_LATCH = False
        if code in _RECOVERY_LINK_SOFT_RESET:
            _push_event("Reset — error de enlace limpiado; sigue la recuperación.")
        return _finish_reset_to_purge(_keep_lot())

    # Pulso GPIO Reset PLC (borrar fallos) antes de validar condiciones físicas.
    if _serial_port("plc"):
        sent = _cycle_plc_send({"op": "reset"})
        if not sent.get("sent"):
            fail_code = str(sent.get("code") or "E019")
            # Si el pulso falla por COM, no dejar al operador atrapado.
            if fail_code in _RECOVERY_LINK_SOFT_RESET:
                _pf_last_fault_sig = ()
                _PF_ERROR_LATCH = False
                _push_event("Reset — sin respuesta PLC; se continúa la recuperación.")
                return _finish_reset_to_purge(_keep_lot())
            return False, error_codes.format_message(fail_code)
        _plc_wait_ready(1.5)

    if not _wait_error_physical_clear_stable():
        return False, _error_reset_blocked_message()
    if _prefeeder_required():
        _prefeeder_stop_now()
        soft_pf = code in _PF_SOFT_ERRORS
        if not soft_pf:
            ok_pf, msg_pf = _prefeeder_stabilize_and_lock()
            if not ok_pf:
                return False, msg_pf
            _prefeeder_refresh_cache()
    st = _PF_CACHED_STATUS
    _pf_last_fault_sig = _prefeeder_fault_sig(st) if st else ()
    _PF_ERROR_LATCH = False
    # El lote solo lo borra Stop: tras un error se queda para Reanudar (después de purga).
    en_curso = bool(
        _MACHINE_STATE.get("ciclo_en_curso") or _LOT.get("active") or _LOT.get("tray_wait")
    )
    return _finish_reset_to_purge(en_curso)


# ── Vías especiales (rutinas de operador) ───────────────────────────────────
# Purga, posición inicial, revisión, carga y avance de un lado.
# No agregan pasos a CYCLE_STEPS ni alteran la secuencia del ciclo.


def _checklist_now() -> dict:
    with _STATE_LOCK:
        return dict(_MACHINE_STATE.get("checklist") or {})


def _checklist_update(**fields) -> None:
    with _STATE_LOCK:
        block = dict(_MACHINE_STATE.get("checklist") or {})
        block.update(fields)
        _MACHINE_STATE["checklist"] = block


def _checklist_blank() -> dict:
    return {
        "ok": False,
        "paso": 1,
        "empleado": "",
        "aire_ok": False,
        "eje_ok": False,
        "recorrido_ok": False,
        "alimentador_ok": False,
        "purga_ok": False,
        "piezas_cortadas": False,
    }


def _set_routine_ui(*, rutina=None, rutina_fase=None, rutina_manos=None, mensaje=None, estado_label=None) -> None:
    with _STATE_LOCK:
        if rutina is not None:
            _MACHINE_STATE["rutina"] = rutina
        if rutina_fase is not None:
            _MACHINE_STATE["rutina_fase"] = rutina_fase
        if rutina_manos is not None:
            _MACHINE_STATE["rutina_manos"] = bool(rutina_manos)
    if mensaje is not None or estado_label is not None:
        _set_machine_ui(mensaje=mensaje, estado_label=estado_label)


def _empleado(value) -> str:
    text = str(value or "").strip()
    if len(text) == 5 and text.isdigit():
        return text
    return ""


def _target_length_mm() -> float:
    try:
        return float(_MOTION_DATA.get("target_length_mm") or 0.0)
    except (TypeError, ValueError):
        return 0.0


def _mant_snapshot() -> dict:
    with _STATE_LOCK:
        return {
            "ciclos": int(_MACHINE_STATE.get("ciclos_mant") or 0),
            "ciclos_r": int(_MACHINE_STATE.get("ciclos_mant_r") or 0),
            "ciclos_l": int(_MACHINE_STATE.get("ciclos_mant_l") or 0),
        }


def _mant_write_file(data: dict | None = None) -> None:
    snap = dict(data) if data is not None else _mant_snapshot()
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    MANT_PATH.write_text(
        json.dumps(
            {
                "ciclos": int(snap.get("ciclos") or 0),
                "ciclos_r": int(snap.get("ciclos_r") or 0),
                "ciclos_l": int(snap.get("ciclos_l") or 0),
            }
        )
        + "\n",
        encoding="utf-8",
    )


def _mant_load() -> None:
    n = 0
    n_r = 0
    n_l = 0
    if MANT_PATH.exists():
        try:
            data = json.loads(MANT_PATH.read_text(encoding="utf-8"))
            n = max(0, int(data.get("ciclos") or 0))
            # Contadores por lado: solo si ya existen en archivo (no repartir el global).
            if "ciclos_r" in data:
                n_r = max(0, int(data.get("ciclos_r") or 0))
            if "ciclos_l" in data:
                n_l = max(0, int(data.get("ciclos_l") or 0))
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            n = 0
            n_r = 0
            n_l = 0
    with _STATE_LOCK:
        _MACHINE_STATE["ciclos_mant"] = n
        _MACHINE_STATE["ciclos_mant_r"] = n_r
        _MACHINE_STATE["ciclos_mant_l"] = n_l


def _mant_add_cycle() -> None:
    """Ciclo de máquina (global). No reparte piezas entre R/L."""
    with _STATE_LOCK:
        n = int(_MACHINE_STATE.get("ciclos_mant") or 0) + 1
        _MACHINE_STATE["ciclos_mant"] = n
        snap = {
            "ciclos": n,
            "ciclos_r": int(_MACHINE_STATE.get("ciclos_mant_r") or 0),
            "ciclos_l": int(_MACHINE_STATE.get("ciclos_mant_l") or 0),
        }
    try:
        _mant_write_file(snap)
    except OSError:
        _push_event("No se pudo guardar el contador de mantenimiento.", "warn")


def _mant_add_side_piece(side: str) -> None:
    """Pieza buena acreditada al lado desde el último reset (independiente del lote)."""
    key = "ciclos_mant_r" if side == "R" else "ciclos_mant_l" if side == "L" else ""
    if not key:
        return
    with _STATE_LOCK:
        n = int(_MACHINE_STATE.get(key) or 0) + 1
        _MACHINE_STATE[key] = n
        snap = {
            "ciclos": int(_MACHINE_STATE.get("ciclos_mant") or 0),
            "ciclos_r": int(_MACHINE_STATE.get("ciclos_mant_r") or 0),
            "ciclos_l": int(_MACHINE_STATE.get("ciclos_mant_l") or 0),
        }
    try:
        _mant_write_file(snap)
    except OSError:
        _push_event("No se pudo guardar el contador por lado.", "warn")


def _mant_reset(data) -> tuple[bool, str]:
    empleado = _empleado(data.get("empleado"))
    if not empleado:
        return False, "Hace falta el número de empleado, cinco dígitos."
    with _STATE_LOCK:
        _MACHINE_STATE["ciclos_mant"] = 0
        _MACHINE_STATE["ciclos_mant_r"] = 0
        _MACHINE_STATE["ciclos_mant_l"] = 0
    try:
        _mant_write_file({"ciclos": 0, "ciclos_r": 0, "ciclos_l": 0})
    except OSError:
        return False, "No se pudo guardar el contador."
    _push_event(f"Contador de mantenimiento en cero. Empleado {empleado}.")
    return True, "Contador en cero."


def _checklist_log_read() -> list:
    if not CHECKLIST_LOG_PATH.exists():
        return []
    try:
        saved = json.loads(CHECKLIST_LOG_PATH.read_text(encoding="utf-8"))
        if isinstance(saved, list):
            return list(saved)
    except (OSError, json.JSONDecodeError):
        pass
    return []


def _checklist_log(empleado: str) -> None:
    entry = {
        "empleado": empleado,
        "fecha": time.strftime("%Y-%m-%d %H:%M:%S"),
        "resultado": "ok",
        "ciclos_mant": int(_MACHINE_STATE.get("ciclos_mant") or 0),
    }
    rows = _checklist_log_read()
    rows.append(entry)
    del rows[:-50]
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    CHECKLIST_LOG_PATH.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")


def _rearm_piece(mensaje: str) -> None:
    """Vuelve la pieza al paso 1. No toca cycles_done ni pieces_r/l."""
    global _PIECE_FEED, _PIECE_ENC_AT_LM, _PIECE_BLOWER_MM, _PIECE_TICK0
    _overlap_reset()
    _cycle_latch_reset()
    _PIECE_FEED = {"R": None, "L": None}
    _PIECE_ENC_AT_LM = {"R": None, "L": None}
    _PIECE_BLOWER_MM = None
    _PIECE_TICK0 = None
    # Mantener el lote vivo tras error/purga para que Reanudar no arranque en R000.
    _lot_ensure_active_for_resume()
    _set_machine_ui(
        estado="paused",
        estado_label="Pausa",
        mensaje=mensaje,
        en_curso=True,
        ciclo_error="",
        step_id="1",
        step_index=0,
        progreso_pct=_lot_progress_pct(0) if int(_LOT.get("cycles_total") or 0) > 0 else 0,
        tiempo_corriendo_s=0,
        paralelo_id="",
        feed={"R": None, "L": None},
    )


def _routine_sleep_ms(ms: int) -> bool:
    deadline = time.time() + (max(0, int(ms)) / 1000.0)
    while time.time() < deadline:
        if _ROUTINE["stop"].is_set():
            return False
        time.sleep(0.02)
    return not _ROUTINE["stop"].is_set()


def _run_routine(fn) -> None:
    try:
        fn()
    except Exception as exc:
        _ROUTINE["from_lote"] = False
        _push_event(f"La rutina se detuvo: {exc}", "error")
        _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False, mensaje="La rutina se detuvo.")
    finally:
        _ROUTINE["thread"] = None


def _start_routine(fn) -> tuple[bool, str]:
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    if not _cycle_thread_alive():
        _CYCLE_CTRL["stop"].clear()
    _ROUTINE["stop"].clear()
    th = threading.Thread(target=_run_routine, args=(fn,), daemon=True)
    _ROUTINE["thread"] = th
    th.start()
    return True, ""


def _motion_cmd_ok(sent: dict, op: str) -> bool:
    if not sent.get("sent"):
        return False
    for raw in reversed(sent.get("replies") or []):
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") == op:
            return bool(msg.get("ok", True))
    return False


def _require_links() -> tuple[bool, str]:
    ready, err = _links_ready_for_cycle()
    if not ready:
        return False, err
    return True, ""


def _routine_try_reset_fault() -> bool:
    if not _serial_port("plc"):
        return True
    faults = _plc_active_errors(_plc_status_get())
    if not faults and not _MACHINE_STATE.get("ciclo_error"):
        return True
    _cycle_plc_send({"op": "reset"})
    faults = _plc_active_errors(_plc_status_get())
    if faults:
        _push_event(faults[0]["message"], "error")
        _set_machine_ui(mensaje="La falla sigue presente.")
        return False
    if _MACHINE_STATE.get("ciclo_error"):
        _set_machine_ui(ciclo_error="")
    return True


def _plc_first_active_fault() -> tuple[str, str] | None:
    """(código, mensaje) del primer fallo PLC operativo, o None."""
    if not _serial_port("plc"):
        return None
    faults = _plc_active_errors(_plc_status_get())
    if not faults:
        return None
    return str(faults[0]["code"]), str(faults[0]["message"])


def _purga_abort_keep_fault() -> None:
    """Cancela purga. Si el PLC sigue en falla, latchea error (no deja solo 'En espera')."""
    fault = _plc_first_active_fault()
    recovery = bool(_MACHINE_STATE.get("post_error_purga"))
    rec = _recovery_get()
    in_auto = bool(
        recovery
        and rec.get("active")
        and str(rec.get("step") or "") in ("purga", "purga_error")
    )
    if in_auto and fault:
        code, _msg = fault
        _recovery_auto_purga_fail(code=code)
        return
    _purga_finish(True)
    if fault:
        _enter_machine_error(fault[0], fault[1])


def _feed_stage_payload(side: str | None = None, *, with_tune: bool = True) -> dict:
    """Una sola orden feed_stage; opcionalmente lleva tune embebido (sin set_tune aparte)."""
    tune = _load_motion_tune()
    if with_tune:
        payload = _motion_tune_flat(tune)
        payload["op"] = "feed_stage"
    else:
        payload = {
            "op": "feed_stage",
            "feed_validate_encoder": bool(tune["feed_stage"]["validate_encoder"]),
        }
    if side in ("R", "L"):
        payload["side"] = side
    return payload


def _push_saved_motion_tune() -> dict:
    """Publica Motion.json. El MCU no conserva el botón entre reinicios."""
    return _cycle_motion_send(_motion_tune_flat(_load_motion_tune()), timeout_s=2.0)


def _feed_once(side: str | None):
    payload = _feed_stage_payload(side)
    sent = _cycle_motion_send(payload, timeout_s=65.0)
    found, ok, error_name, feed = _read_feed_stage(sent)
    _set_machine_ui(feed=feed)
    if not sent.get("sent"):
        return False, sent.get("code") or "E043", feed
    if not found or not ok:
        err = error_name or "unknown"
        return False, error_codes.feed_stage_code(err), feed
    return True, "", feed


def _feeder_reverse(
    side: str | None = None, *, rpm=None, ms=None
) -> tuple[bool, str]:
    """PV inverso por tiempo. side R/L/None(=both). Listo para meter en ciclo."""
    busy = _motion_owner_busy()
    if busy:
        return False, busy
    linked, code = _check_module_link("motion")
    if not linked:
        return False, error_codes.format_message(code)

    tune = _load_motion_tune()
    side_s = str(side or "both").strip().upper()
    if side_s not in ("R", "L", "BOTH"):
        side_s = "BOTH"
    corr = _exp_correction(tune, "R" if side_s == "BOTH" else side_s)
    if rpm is None:
        rpm = corr["rpm"]
    if ms is None:
        ms = corr["ms"]
    rpm_i = _exp_rpm(rpm, corr["rpm"])
    ms_i = _exp_ms(ms, corr["ms"])
    payload = {
        "op": "feeder_reverse",
        "side": "both" if side_s == "BOTH" else side_s,
        "rpm": rpm_i,
        "ms": ms_i,
    }
    timeout_s = max(2.0, (ms_i / 1000.0) + 2.0)
    sent = _cycle_motion_send(payload, timeout_s=timeout_s)
    if not sent.get("sent"):
        return False, sent.get("reason") or error_codes.format_message(
            sent.get("code") or "E043"
        )
    if not _motion_cmd_ok(sent, "feeder_reverse"):
        return False, error_codes.format_message("E044")
    return True, ""


def _feeder_test(side: str, tune: dict) -> dict:
    """Cierra holder/encoder, resetea el contador del lado y alimenta hasta el láser."""
    busy = _motion_owner_busy()
    if busy:
        return {"ok": False, "error": busy}

    for module in ("motion", "plc"):
        linked, code = _check_module_link(module)
        if not linked:
            return {"ok": False, "error": error_codes.format_message(code)}

    params = _load_cycle_params()
    holder_ok, code = _plc_accept_valve(
        _cycle_plc_send(
            {
                "op": "valve",
                "name": "holder_encoder",
                "on": True,
                "settle_ms": int(params["ms_holder_enc_stable"]),
            }
        ),
        {"holder": True, "encoder": True},
    )
    if not holder_ok:
        return {
            "ok": False,
            "error": error_codes.format_message(code) if code else "Detenido.",
        }

    reset_sent = _cycle_motion_send(
        {"op": "encoder_reset", "side": side}, timeout_s=2.0
    )
    if not _motion_cmd_ok(reset_sent, "encoder_reset"):
        return {
            "ok": False,
            "error": reset_sent.get("reason")
            or error_codes.format_message(reset_sent.get("code") or "E043"),
        }

    feed_payload = _motion_tune_flat(tune)
    feed_payload["op"] = "feed_stage"
    feed_payload["side"] = side
    feed_sent = _cycle_motion_send(feed_payload, timeout_s=65.0)
    found, feed_ok, error_name, feed = _read_feed_stage(feed_sent)
    _set_machine_ui(feed=feed)
    replies = (reset_sent.get("replies") or []) + (feed_sent.get("replies") or [])
    if not feed_sent.get("sent"):
        return {
            "ok": False,
            "error": feed_sent.get("reason") or error_codes.format_message("E043"),
            "lines": [
                _compact({"op": "encoder_reset", "side": side}),
                _compact(feed_payload),
            ],
            "replies": replies,
        }
    return {
        "ok": True,
        "sent": True,
        "command_ok": bool(found and feed_ok),
        "error": "" if found and feed_ok else error_codes.feed_stage_message(error_name or "unknown"),
        "lines": [
            _compact({"op": "encoder_reset", "side": side}),
            _compact(feed_payload),
        ],
        "replies": replies,
        "feed": feed,
    }


def _purge_feed_side() -> str | None:
    """Lado de purga según feed_sides. None = ambos (LR)."""
    sides = _run_sides()
    return sides if sides in ("R", "L") else None


def _purga_alimentar_progress_msg() -> str:
    side = _purge_feed_side()
    if side == "R":
        return (
            "Se está alimentando el lado derecho hasta que el sensor detecte el material."
        )
    if side == "L":
        return (
            "Se está alimentando el lado izquierdo hasta que el sensor detecte el material."
        )
    return (
        "Se están alimentando ambos lados hasta que el sensor detecte el material."
    )


def _purga_feed_ok_msg() -> str:
    return "La alimentación fue exitosa. Pulsa Continuar para terminar de limpiar."


def _purga_continuar_banner() -> tuple[str, str]:
    return "Proceso de limpieza en curso", "NO INTRODUCIR MANOS"


def _purga_error_action_guide(code: str = "", *, fallback: str = "") -> str:
    """Acción concreta para Error en purga (sin repetir el diagnóstico)."""
    c = str(code or "").strip().upper()
    if c == "E033":
        return "Revisa el sensor láser derecho y pulsa Retry."
    if c == "E034":
        return "Revisa el sensor láser izquierdo y pulsa Retry."
    if c == "E021":
        return "Corrige el encoder y pulsa Retry."
    if c == "E035":
        return "Revisa el encoder derecho y pulsa Retry."
    if c == "E036":
        return "Revisa el encoder izquierdo y pulsa Retry."
    if c == "E020":
        return "Revisa el holder y pulsa Retry."
    if c == "E022":
        return "Revisa el cortador y pulsa Retry."
    if c == "E023":
        return "Revisa las pinzas y pulsa Retry."
    if c == "E044":
        return "Revisa el LM Guide y pulsa Retry."
    text = str(fallback or "").strip()
    if text:
        return text
    return "Revisa el material y pulsa Retry."


def _feed_laser():
    """Purga: lados activos (feed_sides) hasta su láser. Sin encoder ni compensación."""
    side = _purge_feed_side()
    payload = _feed_stage_payload(side)
    payload["mode"] = "laser"
    sent = _cycle_motion_send(payload, timeout_s=20.0)
    found, ok, error_name, feed = _read_feed_stage(sent)
    _set_machine_ui(feed=feed)
    if not sent.get("sent"):
        return False, sent.get("code") or "E043", feed
    if not found or not ok:
        err = error_name or "unknown"
        return False, error_codes.feed_stage_code(err), feed
    return True, "", feed


def _feed_clean():
    side = _purge_feed_side()
    payload = _feed_stage_payload(side)
    payload["mode"] = "clean"
    payload["timeout_ms"] = PURGE_CLEAN_MS
    sent = _cycle_motion_send(payload, timeout_s=8.0)
    found, ok, error_name, feed = _read_feed_stage(sent)
    _set_machine_ui(feed=feed)
    if not sent.get("sent"):
        return False, sent.get("code") or "E043", feed
    if not found or not ok:
        err = error_name or "unknown"
        return False, error_codes.feed_stage_code(err), feed
    return True, "", feed


def _routine_move_lm_mm(mm: float) -> bool:
    tune = _load_motion_tune()
    if _ROUTINE["stop"].is_set():
        return False
    payload = {"op": "lmguide_move", "puu": _mm_to_puu(mm)}
    payload.update(_lm_cmd_fields(tune))
    move = _cycle_motion_send(payload, timeout_s=40.0)
    return _motion_cmd_ok(move, "lmguide_move")


def _purga_leave_holder_encoder_closed() -> None:
    """Fin de purga: apaga soplo/pinzas/cortadores; deja holder y encoder cerrados."""
    _cycle_plc_send({"op": "blower_off"})
    _cycle_plc_send({"op": "valve", "name": "cutters", "on": False})
    _cycle_plc_send({"op": "valve", "name": "gripper", "on": False})
    _gripper_holding_set(False)
    _cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": True})


def _purga_finish(cancelled: bool) -> None:
    with _STATE_LOCK:
        if _MACHINE_STATE.get("rutina") != "purga":
            return
    _feed_prime_clear()
    if cancelled:
        _cycle_plc_send({"op": "all_off"})
        _gripper_holding_set(False)
    else:
        _purga_leave_holder_encoder_closed()
    with _STATE_LOCK:
        paso = (_MACHINE_STATE.get("checklist") or {}).get("paso")
        _MACHINE_STATE["rutina"] = ""
        _MACHINE_STATE["rutina_fase"] = ""
        _MACHINE_STATE["rutina_manos"] = False
        from_lote = bool(_ROUTINE.get("from_lote"))
        _ROUTINE["from_lote"] = False
        recovery = bool(_MACHINE_STATE.get("post_error_purga"))
        keep = bool(
            _MACHINE_STATE.get("ciclo_en_curso")
            or _LOT.get("active")
            or _LOT.get("tray_wait")
        )
    if cancelled:
        _push_event("Purga cancelada.", "warn")
        msg = "Purga cancelada."
        if recovery:
            rec = _recovery_get()
            if rec.get("kind") == "joint":
                _recovery_set(step="operator", busy=False)
                _set_machine_ui(
                    estado="paused" if keep else "idle",
                    estado_label="Warning Joint",
                    mensaje="Purga Joint cancelada. Usa Finish para reintentar.",
                    en_curso=keep,
                )
                return
            _set_machine_ui(
                estado="paused" if keep else "idle",
                estado_label="Purga pendiente",
                mensaje="Purga cancelada. Completa la purga para continuar.",
                en_curso=keep,
            )
            return
    else:
        if paso == 5:
            _checklist_update(purga_ok=True)
        _push_event("Purga terminada.")
        msg = "Purga terminada."
        if recovery:
            with _STATE_LOCK:
                _MACHINE_STATE["post_error_purga"] = False
                rec = dict(_MACHINE_STATE.get("recovery") or _recovery_blank())
                if rec.get("active"):
                    rec["step"] = "listo"
                    rec["busy"] = False
                    rec["guide"] = (
                        "Purga terminada. Vuelve a la pantalla principal y pulsa Reanudar. "
                        "Esta pieza no suma al contador."
                    )
                    _MACHINE_STATE["recovery"] = rec
            joint_rec = _recovery_get()
            if joint_rec.get("kind") == "joint":
                # Conservar piece_counted para el mensaje; limpiar skip_cut ya.
                piece_counted = bool(_JOINT.get("piece_counted"))
                _joint_latch_clear()
                if keep:
                    if _LOT.get("tray_wait"):
                        deposits = int(_LOT.get("deposits_per_tray") or 0)
                        _set_machine_ui(
                            estado="paused",
                            estado_label="Charola llena",
                            mensaje=(
                                f"Purga Joint terminada. Charola llena — {deposits} ciclos. "
                                "Vacía la charola y confirma."
                            ),
                            en_curso=True,
                            ciclo_error="",
                        )
                    elif piece_counted:
                        # Pieza buena ya contada: preparar siguiente sin rearmar.
                        _overlap_reset()
                        _cycle_latch_reset()
                        _set_machine_ui(
                            estado="paused",
                            estado_label="Warning Joint",
                            mensaje="Joint resuelto. Puedes Reanudar el lote.",
                            en_curso=True,
                            ciclo_error="",
                            step_id="1",
                            step_index=0,
                            paralelo_id="",
                            feed={"R": None, "L": None},
                            tiempo_corriendo_s=0,
                        )
                    else:
                        if _LOT.get("active"):
                            piece = int(_LOT.get("cycles_done") or 0) + 1
                            total = int(_LOT.get("cycles_total") or 0)
                            rearm_msg = (
                                f"Joint listo. Pieza {piece} de {total} se reintenta "
                                "desde el inicio. Puedes Reanudar."
                            )
                        else:
                            rearm_msg = (
                                "Joint listo. La pieza se reintenta desde el inicio. "
                                "Puedes Reanudar."
                            )
                        _rearm_piece(rearm_msg)
                else:
                    _set_machine_ui(
                        estado="idle",
                        estado_label="En espera",
                        mensaje="Joint resuelto. Listo para operar.",
                        en_curso=False,
                    )
                return
            if keep:
                if _LOT.get("tray_wait"):
                    deposits = int(_LOT.get("deposits_per_tray") or 0)
                    _set_machine_ui(
                        estado="paused",
                        estado_label="Charola llena",
                        mensaje=(
                            f"Purga terminada. Charola llena — {deposits} ciclos. "
                            "Vacía la charola y confirma."
                        ),
                        en_curso=True,
                        ciclo_error="",
                    )
                else:
                    # Pieza incompleta: se reintenta desde el 1. cycles_done intacto.
                    if _LOT.get("active"):
                        piece = int(_LOT.get("cycles_done") or 0) + 1
                        total = int(_LOT.get("cycles_total") or 0)
                        rearm_msg = (
                            f"Listo. Pieza {piece} de {total} se reintenta desde el inicio. "
                            "Puedes Reanudar."
                        )
                    else:
                        rearm_msg = (
                            "Listo. La pieza se reintenta desde el inicio. Puedes Reanudar."
                        )
                    _rearm_piece(rearm_msg)
            else:
                _set_machine_ui(
                    estado="idle",
                    estado_label="Listo para trabajar",
                    mensaje="",
                    en_curso=False,
                )
            return
    if from_lote:
        _rearm_piece(msg + " El ciclo queda en pausa y la pieza vuelve al paso 1.")
        return
    if cancelled:
        _set_machine_ui(estado="idle", estado_label="En espera", mensaje=msg, en_curso=False)
    else:
        _set_machine_ui(
            estado="idle",
            estado_label="Listo para trabajar",
            mensaje="",
            en_curso=False,
        )


def _purga_prepare() -> None:
    _set_routine_ui(
        rutina="purga",
        rutina_fase="segura",
        rutina_manos=False,
        estado_label="Proceso de limpieza",
        mensaje="Llevando el eje a un lugar seguro.",
    )
    if not _routine_try_reset_fault() or _ROUTINE["stop"].is_set():
        if _ROUTINE["stop"].is_set():
            _purga_finish(True)
        else:
            _purga_abort_keep_fault()
        return
    if not _cycle_plc_send({"op": "all_off"}).get("sent"):
        _purga_finish(True)
        return
    _gripper_holding_set(False)
    if _ROUTINE["stop"].is_set() or not _routine_move_lm_mm(PURGE_PARK_MM):
        _purga_finish(True)
        return
    _set_routine_ui(
        rutina_fase="espera",
        estado_label="Proceso de limpieza",
        mensaje="Selecciona la opción que deseas utilizar:",
    )


def _purga_start(_data=None) -> tuple[bool, str]:
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "Ya hay una purga en curso."
    if _MACHINE_STATE.get("estado") == "error" or _MACHINE_STATE.get("ciclo_error"):
        return False, "Limpia el error con Reset antes de la purga."
    if _MACHINE_STATE.get("estado") == "running":
        return False, "Pausa o detén el ciclo antes de la purga."
    ready, err = _require_links()
    if not ready:
        return False, err
    # En ruta post-error se conserva el paso pausado; no rearmar a paso 1 al terminar.
    recovery = bool(_MACHINE_STATE.get("post_error_purga"))
    _ROUTINE["from_lote"] = bool(_MACHINE_STATE.get("ciclo_en_curso")) and not recovery
    _set_routine_ui(
        rutina="purga",
        rutina_fase="segura",
        rutina_manos=False,
        estado_label="Proceso de limpieza",
        mensaje="Llevando el eje a un lugar seguro.",
    )
    ok, err = _start_routine(_purga_prepare)
    if not ok:
        _ROUTINE["from_lote"] = False
        _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False)
        return False, err
    return True, "Purga iniciada."


def _purga_close_holder() -> bool:
    params = _load_cycle_params()
    ok_hw, code = _plc_accept_valve(
        _cycle_plc_send(
            {
                "op": "valve",
                "name": "holder_encoder",
                "on": True,
                "settle_ms": int(params["ms_holder_enc_stable"]),
            }
        ),
        {"holder": True, "encoder": True},
    )
    if _ROUTINE["stop"].is_set():
        return False
    if not ok_hw:
        if code:
            _push_event(error_codes.format_message(code), "error")
        return False
    return True


def _purga_alimentar_run() -> None:
    _set_routine_ui(
        rutina_fase="alimentar",
        estado_label="Proceso de limpieza",
        mensaje="Cerrando holder y encoder.",
    )
    if not _purga_close_holder():
        if _ROUTINE["stop"].is_set():
            _purga_finish(True)
        else:
            fault = _plc_first_active_fault()
            if fault:
                _purga_abort_keep_fault()
            else:
                _set_routine_ui(
                    rutina_fase="espera",
                    estado_label="Proceso de limpieza",
                    mensaje="No se cerró holder y encoder.",
                )
        return
    _set_routine_ui(
        estado_label="Proceso de limpieza",
        mensaje=_purga_alimentar_progress_msg(),
    )
    ok, code, _feed = _feed_laser()
    if _ROUTINE["stop"].is_set():
        _purga_finish(True)
        return
    if not ok:
        _push_event(error_codes.format_message(code), "error")
        _set_routine_ui(
            rutina_fase="espera",
            estado_label="Proceso de limpieza",
            mensaje="No llegó al láser. Puedes repetir, continuar o terminar.",
        )
        return
    _set_routine_ui(
        rutina_fase="espera",
        estado_label="Proceso de limpieza",
        mensaje=_purga_feed_ok_msg(),
    )


def _purge_cut() -> bool:
    """Solo purga, al pulsar Continuar: home, corte con pinzas, depósito fijo, home."""
    params = _load_cycle_params()
    label, aviso = _purga_continuar_banner()
    _set_routine_ui(
        rutina_fase="corte",
        rutina_manos=True,
        estado_label=label,
        mensaje=aviso,
    )

    def sent_ok(sent) -> bool:
        op = str((sent.get("last_payload") or {}).get("op") or "")
        return bool(op and _motion_cmd_ok(sent, op)) and not _ROUTINE["stop"].is_set()

    if not _routine_move_lm_mm(0.0) or _ROUTINE["stop"].is_set():
        return False

    # Banner fijo durante cero / corte / descarga / home.
    _set_routine_ui(estado_label=label, mensaje=aviso)
    ok_g, code_g = _plc_accept_valve(
        _cycle_plc_send({"op": "valve", "name": "gripper", "on": True}),
        {"gripper": True},
    )
    if not ok_g or _ROUTINE["stop"].is_set():
        if code_g:
            _push_event(error_codes.format_message(code_g), "error")
        return False
    # Cerrar holder+encoder antes del corte y dejarlos cerrados al terminar.
    ok_hw, code = _plc_accept_valve(
        _cycle_plc_send(
            {
                "op": "valve",
                "name": "holder_encoder",
                "on": True,
                "settle_ms": int(params["ms_holder_enc_stable"]),
            }
        ),
        {"gripper": True, "holder": True, "encoder": True},
    )
    if not ok_hw or _ROUTINE["stop"].is_set():
        if code:
            _push_event(error_codes.format_message(code), "error")
        return False
    ok_c, code_c = _plc_accept_valve(
        _cycle_plc_send(
            {
                "op": "valve",
                "name": "cutters",
                "on": True,
                "ms": int(params["ms_cutter_on"]),
            }
        )
    )
    if not ok_c or _ROUTINE["stop"].is_set():
        if code_c:
            _push_event(error_codes.format_message(code_c), "error")
        return False
    if not _routine_sleep_ms(PURGE_AFTER_CUT_MS):
        return False

    _set_routine_ui(estado_label=label, mensaje=aviso)
    if not _routine_move_lm_mm(PURGE_DEPOSIT_MM):
        return False
    if not _routine_sleep_ms(int(params["ms_lm_at_discharge"])):
        return False
    ok_off, code_off = _plc_accept_valve(
        _cycle_plc_send(
            {
                "op": "valve",
                "name": "gripper",
                "on": False,
                "settle_ms": int(params["ms_after_grip_release"]),
            }
        ),
        {"gripper": False},
    )
    if not ok_off or _ROUTINE["stop"].is_set():
        if code_off:
            _push_event(error_codes.format_message(code_off), "error")
        return False

    _set_routine_ui(estado_label=label, mensaje=aviso)
    blower_ms = _cycle_blower_duration_ms(params)
    payload = {
        "op": "release_part",
        "blower_delay_ms": int(params["ms_blower_delay"]),
        "blower_ms": blower_ms,
        "home_settle_ms": int(params["ms_at_home_end"]),
    }
    payload.update(_lm_cmd_fields(_load_motion_tune()))
    sent = _cycle_motion_send(payload, timeout_s=40.0)
    if _ROUTINE["stop"].is_set():
        _send_motion_urgent(['{"op":"blower_off"}', '{"op":"lmguide_stop"}'])
        return False
    if not _motion_cmd_ok(sent, "release_part"):
        _send_motion_urgent(['{"op":"blower_off"}'])
        return False
    _purga_leave_holder_encoder_closed()
    return not _ROUTINE["stop"].is_set()


def _purga_cut_run() -> None:
    ok = False
    try:
        ok = _purge_cut()
    finally:
        if not ok or _ROUTINE["stop"].is_set():
            _cycle_plc_send({"op": "all_off"})
            _purga_finish(True)
        else:
            _purga_finish(False)


def _purga_limpiar_run() -> None:
    _set_routine_ui(
        rutina_fase="limpiar",
        estado_label="Proceso de limpieza",
        mensaje="Cerrando holder y encoder.",
    )
    if not _purga_close_holder():
        if _ROUTINE["stop"].is_set():
            _purga_finish(True)
        else:
            fault = _plc_first_active_fault()
            if fault:
                _purga_abort_keep_fault()
            else:
                _set_routine_ui(
                    rutina_fase="espera",
                    estado_label="Proceso de limpieza",
                    mensaje="No se cerró holder y encoder.",
                )
        return
    _set_routine_ui(
        estado_label="Proceso de limpieza",
        mensaje="Se está expulsando el material residual. Espera a que termine.",
    )
    ok, code, _feed = _feed_clean()
    if _ROUTINE["stop"].is_set():
        _purga_finish(True)
        return
    if not ok:
        _push_event(error_codes.format_message(code), "error")
        _set_routine_ui(
            rutina_fase="espera",
            estado_label="Proceso de limpieza",
            mensaje="La expulsión no completó. Puedes repetir, continuar o terminar.",
        )
        return
    _set_routine_ui(
        rutina_fase="espera",
        estado_label="Proceso de limpieza",
        mensaje=_purga_feed_ok_msg(),
    )


def _purga_limpiar_action(_data=None) -> tuple[bool, str]:
    if not _purga_claim("limpiar"):
        return False, "La purga no está en espera."
    ok, err = _start_routine(_purga_limpiar_run)
    if not ok:
        _set_routine_ui(rutina_fase="espera", rutina_manos=False)
        return False, err
    return True, "Expulsando."


def _purga_continuar_action(_data=None) -> tuple[bool, str]:
    if not _purga_claim("corte"):
        return False, "La purga no está en espera."
    label, aviso = _purga_continuar_banner()
    _set_routine_ui(
        rutina_fase="corte",
        rutina_manos=True,
        estado_label=label,
        mensaje=aviso,
    )
    ok, err = _start_routine(_purga_cut_run)
    if not ok:
        _set_routine_ui(
            rutina_fase="espera",
            rutina_manos=False,
            estado_label="Proceso de limpieza",
            mensaje="Selecciona la opción que deseas utilizar:",
        )
        return False, err
    return True, "Purga en corte."


def _purga_terminar_action(_data=None) -> tuple[bool, str]:
    if not _purga_claim("terminar"):
        return False, "La purga no está en espera."
    _purga_finish(False)
    return True, "Purga terminada."


def _posicion_inicial_run() -> None:
    in_purge = _MACHINE_STATE.get("rutina") == "purga"
    if not in_purge:
        _set_routine_ui(rutina="posicion_inicial", rutina_fase="", rutina_manos=False, mensaje="Posición inicial.")
    if not _routine_try_reset_fault() or _ROUTINE["stop"].is_set():
        if in_purge and _ROUTINE["stop"].is_set():
            _purga_finish(True)
        elif not in_purge:
            _set_routine_ui(rutina="", mensaje="La falla sigue presente.")
        return
    home = _cycle_motion_send({"op": "lmguide_move", "target": "home"}, timeout_s=40.0)
    if _ROUTINE["stop"].is_set() or not _motion_cmd_ok(home, "lmguide_move"):
        if in_purge:
            _purga_finish(True)
        else:
            _set_routine_ui(rutina="", mensaje="No se llegó a cero.")
        return
    enc = _cycle_motion_send({"op": "encoder_set0", "side": "both"}, timeout_s=2.0)
    _cycle_plc_send({"op": "all_off"})
    _gripper_holding_set(False)
    if not _motion_cmd_ok(enc, "encoder_set0"):
        _push_event("El eje llegó a cero. Los encoders no confirmaron el cero.", "warn")
    if in_purge:
        if _ROUTINE["stop"].is_set():
            _purga_finish(True)
        else:
            _set_routine_ui(
                rutina_fase="espera",
                mensaje="Purga en espera. Eje en cero, encoders en cero y aire apagado.",
            )
        return
    if _MACHINE_STATE.get("ciclo_en_curso"):
        _rearm_piece("Posición inicial. El ciclo queda en pausa y la pieza vuelve al paso 1.")
    else:
        _set_machine_ui(
            estado="idle",
            estado_label="En espera",
            mensaje="Posición inicial lista. Eje en cero, encoders en cero y aire apagado.",
            en_curso=False,
        )
    _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False)


def _posicion_inicial(_data=None) -> tuple[bool, str]:
    if _MACHINE_STATE.get("rutina_manos"):
        return False, "Espera a que termine el tramo de purga."
    rutina = _MACHINE_STATE.get("rutina") or ""
    if rutina == "purga" and _MACHINE_STATE.get("rutina_fase") != "espera":
        return False, "La purga todavía se está moviendo."
    if rutina not in ("", "purga"):
        return False, "Hay otra rutina en curso."
    if _MACHINE_STATE.get("estado") == "running":
        return False, "Detén el ciclo antes de la posición inicial."
    ready, err = _require_links()
    if not ready:
        return False, err
    ok, err = _start_routine(_posicion_inicial_run)
    if not ok:
        return False, err
    return True, "Posición inicial en curso."


_SIDE_NAME = {"L": "izquierda", "R": "derecha"}


def _alimentar_run(side: str) -> None:
    _set_routine_ui(rutina="alimentar", rutina_fase=side, mensaje=f"Alimentando lado {_SIDE_NAME[side]}.")
    ok, code, feed = _feed_once(side)
    block = feed.get(side) or {}
    if ok:
        mm = block.get("mm")
        text = f"Lado {_SIDE_NAME[side]} alimentado"
        if mm is not None:
            text += f" ({mm} mm)"
        _set_machine_ui(mensaje=text + ".")
        _push_event(text + ".")
    else:
        msg = error_codes.format_message(code, detail=side)
        _push_event(msg, "error")
        _set_machine_ui(mensaje=msg)
    _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False)


def _alimentar_izquierda(data=None):
    return False, "Mantén pulsado JOG - L en el panel."


def _alimentar_derecha(data=None):
    return False, "Mantén pulsado JOG - R en el panel."


def _feeder_init_now() -> tuple[bool, str]:
    ok_link, code = _check_module_link("motion")
    if not ok_link:
        return False, error_codes.format_message(code)
    # Sesión nueva con hardware ya vivo: no forzar re-init si Motion lo confirma.
    st = _motion_status_get()
    if _actuators_sync_from_motion_status(st):
        _push_event("Hardware Motion ya estaba listo (CAN + LM home).")
        return True, "Hardware Motion ya estaba listo (CAN + LM home)."
    tune = _load_motion_tune()
    lines = _build_motion_lines("motion_hw_init", tune, {})[1]
    _actuators_set("initializing", "feeder")
    sent = _send_motion_serial(
        lines,
        line_timeout_s=120.0,
        reply_until_op="motion_hw_init",
        on_line=_hw_init_on_line,
    )
    sent["last_payload"] = {"op": "motion_hw_init"}
    if not _motion_cmd_ok(sent, "motion_hw_init"):
        _actuators_set("offline")
        return False, error_codes.format_message(sent.get("code") or "E043")
    _actuators_set("online")
    _push_event("Hardware Motion inicializado (CAN + LM home).")
    return True, "Hardware Motion inicializado (CAN + LM home)."


def _prealimentador_iniciar(_data=None) -> tuple[bool, str]:
    if _MACHINE_STATE.get("rutina"):
        return False, "Hay una rutina en curso."
    if not _prefeeder_required():
        return True, "PreAlimentador sin COM (omite estabilización)."
    ok, msg = _prefeeder_stabilize_and_lock()
    _prefeeder_refresh_cache()
    if not ok:
        st = _PF_CACHED_STATUS
        sig = _prefeeder_fault_sig(st) if st else ()
        if sig:
            _prefeeder_raise_fault(sig)
        else:
            _enter_machine_error("E099", msg, en_curso=False)
        return False, msg
    global _pf_last_fault_sig, _PF_ERROR_LATCH
    _pf_last_fault_sig = ()
    _PF_ERROR_LATCH = False
    with _STATE_LOCK:
        if _MACHINE_STATE.get("estado") == "error" and not _MACHINE_STATE.get("ciclo_en_curso"):
            _set_machine_ui(
                estado="idle",
                estado_label="En espera",
                ciclo_error="",
                mensaje="PreAlimentador estabilizado y bloqueado.",
            )
    _push_event("PreAlimentador estabilizado y bloqueado.")
    return True, "PreAlimentador estabilizado y bloqueado."


def _checklist_motion_free() -> str:
    if _MACHINE_STATE.get("ciclo_en_curso"):
        return "Termina el ciclo antes de este paso de la revisión."
    if _MACHINE_STATE.get("rutina"):
        return "Hay una rutina en curso."
    return _motion_owner_busy()


def _checklist_empezar(data) -> tuple[bool, str]:
    if _checklist_now().get("ok"):
        return False, "La revisión ya está completa. Puedes cerrar la ventana."
    empleado = _empleado(data.get("empleado"))
    if not empleado:
        return False, "Hace falta el número de empleado, cinco dígitos."
    if int(_MACHINE_STATE.get("ciclos_mant") or 0) >= CHECKLIST_MAX_CYCLES:
        return False, "El contador llegó a 10000 ciclos. Resetealo en Mantenimiento."
    _checklist_update(empleado=empleado, paso=2)
    _push_event(f"Revisión de arranque. Empleado {empleado}.")
    return True, "Revisión iniciada."


def _checklist_nueva(_data=None) -> tuple[bool, str]:
    """Conservado por compatibilidad; la HMI ya no ofrece reiniciar tras completar."""
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    _checklist_update(**_checklist_blank())
    _set_machine_ui(mensaje="Revisión de arranque reiniciada.")
    return True, "Revisión reiniciada."


def _checklist_aire(_data=None) -> tuple[bool, str]:
    if _checklist_now().get("paso") != 2:
        return False, "La prueba de aire no corresponde a este paso."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    ok_link, code = _check_module_link("plc")
    if not ok_link:
        return False, error_codes.format_message(code)
    params = _load_cycle_params()
    ok_hw, code = _plc_accept_valve(
        _cycle_plc_send(
            {
                "op": "valve",
                "name": "holder_encoder",
                "on": True,
                "settle_ms": int(params["ms_holder_enc_stable"]),
            }
        ),
        {"holder": True, "encoder": True},
    )
    _cycle_plc_send({"op": "all_off"})
    if not ok_hw:
        return False, error_codes.format_message(code) if code else "Prueba de aire fallida."
    _checklist_update(aire_ok=True, paso=3)
    _push_event("Prueba de aire correcta.")
    return True, "Aire correcto."


def _checklist_holgura(_data=None) -> tuple[bool, str]:
    if _checklist_now().get("paso") != 3:
        return False, "La confirmación de holgura no corresponde a este paso."
    _checklist_update(paso=4)
    _push_event("Holgura confirmada por el operador.")
    return True, "Holgura confirmada."


def _checklist_home_run() -> None:
    _set_routine_ui(rutina="checklist", mensaje="Buscando la referencia mecánica del eje.")
    sent = _cycle_motion_send({"op": "lmguide_home"}, timeout_s=40.0)
    if _ROUTINE["stop"].is_set() or not _motion_cmd_ok(sent, "lmguide_home"):
        _set_routine_ui(rutina="", mensaje="La referencia del eje no se completó.")
        return
    _checklist_update(eje_ok=True)
    _set_routine_ui(rutina="", mensaje="Eje en referencia. Haz el recorrido y confirma que regresó.")


def _checklist_home(_data=None) -> tuple[bool, str]:
    if _checklist_now().get("paso") != 4:
        return False, "La referencia del eje no corresponde a este paso."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    ok_link, code = _check_module_link("motion")
    if not ok_link:
        return False, error_codes.format_message(code)
    ok, err = _start_routine(_checklist_home_run)
    if not ok:
        return False, err
    return True, "Buscando referencia del eje."


def _checklist_recorrido_run() -> None:
    _set_routine_ui(rutina="checklist", mensaje="Recorrido del eje.")
    params = _load_cycle_params()
    pos = _lm_cut_and_discharge(params)
    if pos is not None:
        _cut_mm, disc = pos
        tune = _load_motion_tune()
        puu = _mm_to_puu(disc)
        payload = {"op": "lmguide_move", "puu": puu}
        payload.update(_lm_cmd_fields(tune))
        move = _cycle_motion_send(payload, timeout_s=40.0)
        if _ROUTINE["stop"].is_set() or not _motion_cmd_ok(move, "lmguide_move"):
            _set_routine_ui(rutina="", mensaje="El recorrido no se completó.")
            return
        if not _routine_sleep_ms(int(params["ms_lm_at_discharge"])):
            _set_routine_ui(rutina="", mensaje="Recorrido detenido.")
            return
    else:
        _push_event("Sin longitud suficiente para la descarga. El recorrido solo regresa a cero.", "warn")
    home = _cycle_motion_send({"op": "lmguide_move", "target": "home"}, timeout_s=40.0)
    if _ROUTINE["stop"].is_set() or not _motion_cmd_ok(home, "lmguide_move"):
        _set_routine_ui(rutina="", mensaje="El eje no regresó a cero.")
        return
    _checklist_update(recorrido_ok=True)
    _set_routine_ui(rutina="", mensaje="Recorrido listo. Confirma que el eje regresó.")


def _checklist_recorrido(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 4 or not ch.get("eje_ok"):
        return False, "Primero lleva el eje a la referencia."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    ok, err = _start_routine(_checklist_recorrido_run)
    if not ok:
        return False, err
    return True, "Recorrido en curso."


def _checklist_eje_listo(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 4:
        return False, "Este paso no está activo."
    if not ch.get("eje_ok") or not ch.get("recorrido_ok"):
        return False, "Falta la referencia o el recorrido."
    _checklist_update(paso=5)
    return True, "Eje confirmado."


def _checklist_preparar(_data=None) -> tuple[bool, str]:
    if _checklist_now().get("paso") != 5:
        return False, "Preparar el alimentador no corresponde a este paso."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    ok, err = _feeder_init_now()
    if not ok:
        return False, err
    if _prefeeder_required():
        ok_pf, err_pf = _prealimentador_iniciar()
        if not ok_pf:
            return False, err_pf
        _checklist_update(alimentador_ok=True)
        return True, "Motion y PreAlimentador listos."
    _checklist_update(alimentador_ok=True)
    return True, err


def _checklist_purga(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 5 or not ch.get("alimentador_ok"):
        return False, "Primero prepara el alimentador."
    return _purga_start()


def _checklist_purga_hecha(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 5:
        return False, "Este paso no está activo."
    if not ch.get("purga_ok"):
        return False, "Primero termina la purga."
    _checklist_update(paso=6)
    return True, "Purga de revisión lista."


def _checklist_piezas_run() -> None:
    for n in (1, 2):
        if _ROUTINE["stop"].is_set():
            _set_routine_ui(rutina="", mensaje="Piezas de prueba detenidas.")
            return
        _set_routine_ui(rutina="checklist", rutina_fase="piezas", mensaje=f"Pieza de prueba {n} de 2.")
        _cycle_latch_reset()
        _CYCLE_CTRL["stop"].clear()
        _CYCLE_CTRL["pause_after_step"].clear()
        _set_machine_ui(ciclo_error="", feed={"R": None, "L": None})
        _cycle_worker(0)
        if _MACHINE_STATE.get("ciclo_error") or _MACHINE_STATE.get("estado") != "idle":
            failed = bool(_MACHINE_STATE.get("ciclo_error"))
            _cycle_latch_reset()
            _set_machine_ui(en_curso=False, step_id="", step_index=0, progreso_pct=0)
            _set_routine_ui(
                rutina="",
                rutina_fase="",
                mensaje="La pieza de prueba no se completó." if failed else "Piezas de prueba detenidas.",
            )
            return
    _checklist_update(piezas_cortadas=True)
    _set_routine_ui(
        rutina="",
        rutina_fase="",
        mensaje="Piezas cortadas. Anota la longitud medida de cada lado.",
    )


def _checklist_piezas(_data=None) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 6:
        return False, "Las piezas de prueba no corresponden a este paso."
    if ch.get("piezas_cortadas"):
        return False, "Las piezas ya se cortaron. Anota la medida."
    blocked = _checklist_motion_free()
    if blocked:
        return False, blocked
    modelo_id = str(_MOTION_DATA.get("modelo_id") or _MACHINE_STATE.get("numero_modelo") or "").strip()
    if not modelo_id or _target_length_mm() <= 0 or not _find_modelo(modelo_id):
        return False, "Selecciona un modelo."
    ready, err = _require_links()
    if not ready:
        return False, err
    ok, err = _start_routine(_checklist_piezas_run)
    if not ok:
        return False, err
    return True, "Cortando dos piezas de prueba."


def _checklist_medicion(data) -> tuple[bool, str]:
    ch = _checklist_now()
    if ch.get("paso") != 6 or not ch.get("piezas_cortadas"):
        return False, "Primero corta las dos piezas."
    try:
        mm_l = float(data.get("mm_l"))
        mm_r = float(data.get("mm_r"))
    except (TypeError, ValueError):
        return False, "Anota las dos longitudes en milímetros."
    expected = _target_length_mm()
    tol = float(_load_cycle_params().get("tolerance_mm") or 2.0)
    if abs(mm_l - expected) > tol or abs(mm_r - expected) > tol:
        return False, f"Fuera de tolerancia. Se espera {expected:.1f} mm ±{tol:.0f}."
    try:
        _checklist_log(ch.get("empleado") or "")
    except OSError:
        _push_event("La revisión pasó, pero no se pudo guardar el registro.", "warn")
    _checklist_update(ok=True)
    _set_machine_ui(mensaje="Revisión de arranque completa. Ya se puede iniciar.")
    _push_event("Revisión de arranque completa.")
    return True, "Revisión completa."


def _purga_claim(fase: str) -> bool:
    with _STATE_LOCK:
        if _MACHINE_STATE.get("rutina") != "purga" or _MACHINE_STATE.get("rutina_fase") != "espera":
            return False
        _MACHINE_STATE["rutina_fase"] = fase
        return True


def _purga_alimentar_action(data=None) -> tuple[bool, str]:
    if not _purga_claim("alimentar"):
        return False, "La purga no está esperando alimentación."
    ok, err = _start_routine(_purga_alimentar_run)
    if not ok:
        _set_routine_ui(rutina_fase="espera")
        return False, err
    return True, "Alimentando."


def _recovery_confirm_piezas(_data=None) -> tuple[bool, str]:
    rec = _recovery_get()
    if not rec.get("active"):
        return False, "No hay una recuperación en curso."
    if rec.get("step") != "piezas":
        return False, "Primero limpia el error con Reset."
    if _MACHINE_STATE.get("estado") == "error" or _MACHINE_STATE.get("ciclo_error"):
        return False, "Todavía hay un error activo. Pulsa Reset."
    _recovery_set(step="manguera", lm_at_park=False)
    _recovery_set_guide(
        "Retira el material hasta apagar ambos sensores láser. "
        "Cuando digan Apagado, pulsa Confirmar."
    )
    code_show = str(rec.get("code") or "").strip()
    _set_machine_ui(
        mensaje=(
            f"{code_show}. Abre el aviso para continuar."
            if code_show
            else "Abre el aviso para continuar."
        ),
        estado_label="Recuperación",
    )
    return True, "Confirma sensores láser."


def _recovery_banner_hint() -> str:
    code = str(_recovery_get().get("code") or "").strip()
    return f"{code}. Abre el aviso para continuar." if code else "Abre el aviso para continuar."


def _recovery_auto_purga_fail(msg: str = "", *, code: str = "") -> None:
    """Fallo o cancelación de purga automática."""
    user_cancel = bool(_ROUTINE.pop("purga_user_cancel", False))
    _cycle_plc_send({"op": "all_off"})
    _purga_finish(True)
    if user_cancel:
        _recovery_set(step="manguera", busy=False)
        _recovery_set_guide("Purga cancelada. Puedes Confirmar para reintentar.")
        _set_machine_ui(
            estado_label="Recuperación",
            mensaje=_recovery_banner_hint(),
        )
        return
    fail_code = str(code or "").strip().upper()
    fail_msg = str(msg or "").strip()
    # Si el fallo de purga trae código propio, actualizar el recuadro de error.
    if fail_code:
        try:
            fail_msg = error_codes.format_message(fail_code)
        except Exception:
            if not fail_msg:
                fail_msg = fail_code
        _recovery_set(
            step="purga_error",
            busy=False,
            code=fail_code,
            mensaje=fail_msg,
        )
    else:
        _recovery_set(step="purga_error", busy=False)
    # INSTRUCCIONES: solo la acción; el diagnóstico queda en el recuadro.
    _recovery_set_guide(
        _purga_error_action_guide(
            fail_code,
            fallback="Revisa el material y pulsa Retry.",
        )
    )
    _set_machine_ui(
        estado_label="Purga — error",
        mensaje=_recovery_banner_hint(),
    )


def _recovery_auto_purga_run() -> None:
    """Purga automática post-error: park + holder/encoder + feed + corte."""
    _ROUTINE["from_lote"] = False
    label, aviso = _purga_continuar_banner()
    _recovery_set(step="purga", busy=True)
    _set_routine_ui(
        rutina="purga",
        rutina_fase="segura",
        rutina_manos=True,
        estado_label=label,
        mensaje=aviso,
    )
    _recovery_set_guide("")
    _set_machine_ui(mensaje=_recovery_banner_hint())

    # Tiempo para leer el aviso antes de mover.
    if not _routine_sleep_ms(500):
        _recovery_auto_purga_fail("Purga cancelada.")
        return
    if not _routine_try_reset_fault() or _ROUTINE["stop"].is_set():
        if _ROUTINE["stop"].is_set():
            _recovery_auto_purga_fail("Purga cancelada o falla de reset.")
        else:
            _purga_abort_keep_fault()
        return
    if not _cycle_plc_send({"op": "all_off"}).get("sent"):
        _recovery_auto_purga_fail(
            "No se pudo apagar salidas PLC para la purga.",
        )
        return
    _gripper_holding_set(False)
    # Banner fijo durante toda la limpieza automática.
    _set_routine_ui(estado_label=label, mensaje=aviso)
    if _ROUTINE["stop"].is_set() or not _routine_move_lm_mm(PURGE_PARK_MM):
        _recovery_auto_purga_fail("No se alcanzó Park en la purga.", code="E044")
        return

    # Misma secuencia que Alimentar + Continuar de la purga manual.
    _set_routine_ui(rutina_fase="alimentar", estado_label=label, mensaje=aviso)
    if not _purga_close_holder() or _ROUTINE["stop"].is_set():
        if _ROUTINE["stop"].is_set():
            _recovery_auto_purga_fail(
                "No se cerró holder/encoder.",
            )
        else:
            fault = _plc_first_active_fault()
            if fault:
                code, _msg = fault
                _recovery_auto_purga_fail(code=code)
            else:
                _recovery_auto_purga_fail(
                    "No se cerró holder/encoder.",
                )
        return
    _set_routine_ui(estado_label=label, mensaje=aviso)
    ok_feed, code, _feed = _feed_laser()
    if _ROUTINE["stop"].is_set():
        _recovery_auto_purga_fail("Purga cancelada durante la alimentación.")
        return
    if not ok_feed:
        msg = (
            error_codes.format_message(code)
            if code
            else "Falló la alimentación hasta el láser."
        )
        _push_event(msg, "error")
        _recovery_auto_purga_fail(msg, code=code or "")
        return

    ok = False
    try:
        ok = _purge_cut()
    finally:
        if not ok or _ROUTINE["stop"].is_set():
            _recovery_auto_purga_fail(
                "Falló el corte de purga.",
            )
        else:
            _purga_finish(False)


def _recovery_start_auto_purga() -> tuple[bool, str]:
    """Arranca la purga automática post-error (desde manguera o Retry)."""
    if _MACHINE_STATE.get("estado") == "error" or _MACHINE_STATE.get("ciclo_error"):
        return False, "Todavía hay un error activo."
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "Ya hay una purga en curso."
    ready, err = _require_links()
    if not ready:
        return False, err
    _ROUTINE.pop("purga_user_cancel", None)
    label, aviso = _purga_continuar_banner()
    _recovery_set(step="purga", busy=True)
    _set_routine_ui(
        rutina="purga",
        rutina_fase="segura",
        rutina_manos=True,
        estado_label=label,
        mensaje=aviso,
    )
    _recovery_set_guide("")
    _set_machine_ui(mensaje=_recovery_banner_hint())
    ok, start_err = _start_routine(_recovery_auto_purga_run)
    if not ok:
        _set_routine_ui(rutina="", rutina_fase="", rutina_manos=False)
        _recovery_set(step="purga_error", busy=False)
        _recovery_set_guide(
            _purga_error_action_guide(
                "",
                fallback="No se pudo iniciar la purga. Revisa y pulsa Retry.",
            )
        )
        return False, start_err
    return True, "Purga automática iniciada."


def _recovery_confirm_manguera(_data=None) -> tuple[bool, str]:
    rec = _recovery_get()
    if not rec.get("active"):
        return False, "No hay una recuperación en curso."
    if rec.get("step") != "manguera":
        return False, "Primero confirma que las piezas están bien."
    if not _recovery_lasers_clear_now():
        return False, "Los sensores láser aún detectan material. Retíralo hasta Apagado."
    return _recovery_start_auto_purga()


def _recovery_safe_zone_run() -> None:
    """Funciones Extras · Zona segura: eje a Park y abre holder/encoder."""
    rec = _recovery_get()
    _recovery_set_guide("Zona segura: moviendo el eje. Espera.")
    _set_machine_ui(mensaje=_recovery_banner_hint())
    ok_park = (not _ROUTINE["stop"].is_set()) and _routine_move_lm_mm(PURGE_PARK_MM)
    if not ok_park:
        _recovery_set(busy=False, lm_at_park=bool(rec.get("lm_at_park")))
        _recovery_set_guide("Zona segura: no se alcanzó la posición. Reintenta.")
        _set_machine_ui(mensaje=_recovery_banner_hint())
        return
    _recovery_set_guide("Zona segura: abriendo sujeciones…")
    _set_machine_ui(mensaje=_recovery_banner_hint())
    ok_hw, code = _plc_accept_valve(
        _cycle_plc_send({"op": "valve", "name": "holder_encoder", "on": False}),
        {"holder": False, "encoder": False},
    )
    _recovery_set(busy=False, lm_at_park=True)
    if _ROUTINE["stop"].is_set():
        _recovery_set_guide("Zona segura: posición ok, pero no se abrieron las sujeciones. Reintenta.")
        _set_machine_ui(mensaje=_recovery_banner_hint())
        return
    if not ok_hw:
        _recovery_set_guide(
            "Zona segura: posición ok, pero las sujeciones no confirmaron"
            + (f" ({code})." if code else ".")
            + " Reintenta."
        )
    else:
        _recovery_set_guide(
            "Zona segura lista. Retira el material hasta Apagado y pulsa Confirmar."
        )
    _set_machine_ui(mensaje=_recovery_banner_hint())


def _recovery_safe_zone(_data=None) -> tuple[bool, str]:
    rec = _recovery_get()
    if not rec.get("active"):
        return False, "No hay una recuperación en curso."
    if rec.get("step") != "manguera":
        return False, "Zona segura aplica en el paso de sensores láser."
    if rec.get("busy"):
        return False, "Espera a que termine Zona segura."
    if _MACHINE_STATE.get("rutina"):
        return False, "Hay una rutina en curso."
    ready, err = _require_links()
    if not ready:
        return False, err
    _recovery_set(busy=True)
    ok, start_err = _start_routine(_recovery_safe_zone_run)
    if not ok:
        _recovery_set(busy=False)
        return False, start_err
    return True, "Zona segura…"


def _recovery_retry_purga(_data=None) -> tuple[bool, str]:
    rec = _recovery_get()
    if not rec.get("active"):
        return False, "No hay una recuperación en curso."
    if rec.get("step") != "purga_error":
        return False, "Retry aplica cuando la purga automática falló."
    return _recovery_start_auto_purga()


def _recovery_cancel_purga(_data=None) -> tuple[bool, str]:
    rec = _recovery_get()
    if not rec.get("active"):
        return False, "No hay una recuperación en curso."
    if rec.get("step") not in ("purga_error", "purga"):
        return False, "Cancelar aplica durante la purga de recuperación."
    if _MACHINE_STATE.get("rutina") == "purga":
        _ROUTINE["purga_user_cancel"] = True
        _ROUTINE["stop"].set()
        return True, "Cancelando purga…"
    _cycle_plc_send({"op": "all_off"})
    _recovery_set(step="manguera", busy=False)
    _set_machine_ui(
        estado_label="Recuperación",
        mensaje="Purga cancelada. Puedes Confirmar para reintentar.",
    )
    return True, "Purga cancelada."


def _recovery_skip_allowed(rec: dict | None = None) -> tuple[bool, str]:
    """SKIP solo en piezas/manguera/purga/purga_error (límites). No durante purga en curso ni Joint."""
    rec = rec if rec is not None else _recovery_get()
    if not rec.get("active"):
        return False, "No hay una recuperación en curso."
    if rec.get("kind") == "joint":
        return False, "SKIP no aplica a la recuperación Joint."
    step = str(rec.get("step") or "")
    if step not in ("piezas", "manguera", "purga", "purga_error"):
        return False, "SKIP no está disponible en este paso."
    if _MACHINE_STATE.get("estado") == "error" or _MACHINE_STATE.get("ciclo_error"):
        return False, "Todavía hay un error activo. Pulsa Reset."
    # Límite de purga: rutina=="purga" marca inicio hasta _purga_finish (limpia rutina).
    if _MACHINE_STATE.get("rutina") == "purga":
        return False, "SKIP no está disponible durante la purga."
    return True, step


def _recovery_skip(data=None) -> tuple[bool, str]:
    data = data or {}
    empleado = _empleado(data.get("empleado"))
    if not empleado:
        return False, "Hace falta el número de empleado, cinco dígitos."
    rec = _recovery_get()
    ok, detail = _recovery_skip_allowed(rec)
    if not ok:
        return False, detail
    step = detail
    code = str(rec.get("code") or "").strip() or "—"
    _push_event(
        f"SKIP recuperación. Empleado {empleado}. Error {code}. Paso {step}.",
        "warn",
    )
    with _STATE_LOCK:
        _MACHINE_STATE["post_error_purga"] = False
    # Skip cierra la recuperación: no dejar error/código colgado en el banner.
    _recovery_clear()
    keep = bool(
        _MACHINE_STATE.get("ciclo_en_curso")
        or _LOT.get("active")
        or _LOT.get("tray_wait")
    )
    if keep:
        if _LOT.get("tray_wait"):
            deposits = int(_LOT.get("deposits_per_tray") or 0)
            _set_machine_ui(
                estado="paused",
                estado_label="Charola llena",
                mensaje=(
                    f"Charola llena — {deposits} ciclos. "
                    "Vacía la charola y confirma."
                ),
                en_curso=True,
                ciclo_error="",
            )
        elif _LOT.get("active"):
            piece = int(_LOT.get("cycles_done") or 0) + 1
            total = int(_LOT.get("cycles_total") or 0)
            _rearm_piece(
                f"Pieza {piece} de {total} se reintenta desde el inicio. "
                "Puedes Reanudar."
            )
        else:
            _rearm_piece(
                "La pieza se reintenta desde el inicio. Puedes Reanudar."
            )
    else:
        _set_machine_ui(
            estado="idle",
            estado_label="En espera",
            mensaje="Listo para operar.",
            en_curso=False,
        )
    return True, "Recuperación omitida. Listo."


def _recovery_done(_data=None) -> tuple[bool, str]:
    rec = _recovery_get()
    if not rec.get("active"):
        return True, ""
    if rec.get("step") == "purga" or _MACHINE_STATE.get("rutina") == "purga":
        return False, "Espera a que termine la purga."
    if rec.get("kind") == "joint":
        if rec.get("step") == "listo":
            _recovery_clear()
            return True, "Listo."
        # Salir del popup; el WARNING Joint permanece hasta Finish.
        return True, "Puedes volver cuando quieras."
    if rec.get("step") not in ("listo", "piezas", "manguera", "purga_error", "error"):
        return False, "La recuperación aún no termina."
    if rec.get("step") != "listo":
        # Salir del popup está permitido; no borra la recuperación pendiente.
        return True, "Puedes volver cuando quieras."
    _recovery_clear()
    return True, "Listo."


_ROUTINE_ACTIONS = {
    "purga": _purga_start,
    "purga_alimentar": _purga_alimentar_action,
    "purga_limpiar": _purga_limpiar_action,
    "purga_continuar": _purga_continuar_action,
    "purga_terminar": _purga_terminar_action,
    "posicion_inicial": _posicion_inicial,
    "iniciar": _prealimentador_iniciar,
    "checklist_empezar": _checklist_empezar,
    "checklist_nueva": _checklist_nueva,
    "checklist_aire": _checklist_aire,
    "checklist_holgura": _checklist_holgura,
    "checklist_home": _checklist_home,
    "checklist_recorrido": _checklist_recorrido,
    "checklist_eje_listo": _checklist_eje_listo,
    "checklist_preparar": _checklist_preparar,
    "checklist_purga": _checklist_purga,
    "checklist_purga_hecha": _checklist_purga_hecha,
    "checklist_piezas": _checklist_piezas,
    "checklist_medicion": _checklist_medicion,
    "mantenimiento_reset": _mant_reset,
    "charola_vacia": _charola_vacia,
    "recovery_confirm_piezas": _recovery_confirm_piezas,
    "recovery_confirm_manguera": _recovery_confirm_manguera,
    "recovery_safe_zone": _recovery_safe_zone,
    "recovery_retry_purga": _recovery_retry_purga,
    "recovery_cancel_purga": _recovery_cancel_purga,
    "recovery_skip": _recovery_skip,
    "recovery_done": _recovery_done,
    "joint_alimentar": _joint_alimentar_action,
    "joint_liberar": _joint_liberar_action,
    "joint_continuar": _joint_continuar_action,
    "joint_retry": _joint_retry_action,
    "joint_confirm": _joint_confirm_action,
    "joint_continue": _joint_continue_action,
    "joint_finish": _joint_finish_action,
}

_mant_load()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/modelos", methods=["GET"])
def get_modelos():
    try:
        modelos = _load_modelos()
    except (OSError, json.JSONDecodeError):
        return jsonify({"ok": False, "modelos": [], "error": "No se pudo leer el catálogo de modelos"}), 500
    return jsonify({"ok": True, "modelos": modelos})


@app.route("/api/modelo", methods=["POST"])
def set_modelo():
    data = request.get_json(silent=True) or {}
    modelo_id = str(data.get("modelo") or data.get("item") or "").strip()
    desc = data.get("desc")
    if desc is not None:
        desc = str(desc).strip()
    longitud = data.get("longitud_mm")
    if longitud is None:
        longitud = data.get("longitud")
    try:
        ok, err = _apply_modelo_selection(modelo_id, longitud, desc)
    except (OSError, json.JSONDecodeError):
        return jsonify({"ok": False, "error": "No se pudo leer el catálogo de modelos"}), 500
    if not ok:
        return jsonify({"ok": False, "error": err}), 404 if "no encontrado" in err.lower() else 400
    _save_hmi_cycle_extras()
    return jsonify({"ok": True, "state": _public_machine_state(), "motion_data": dict(_MOTION_DATA)})


@app.route("/api/cantidad", methods=["POST"])
def set_cantidad():
    data = request.get_json(silent=True) or {}
    try:
        n = int(data.get("cantidad"))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "La cantidad tiene que ser un entero."}), 400
    if n < 1:
        return jsonify({"ok": False, "error": "La cantidad tiene que ser al menos 1."}), 400
    if _LOT["active"] or _LOT["tray_wait"]:
        return jsonify({"ok": False, "error": "No se cambia la cantidad con un lote en curso."}), 400
    _MACHINE_STATE["cantidad"] = n
    _save_hmi_cycle_extras()
    return jsonify({"ok": True, "state": _public_machine_state()})


@app.route("/api/state", methods=["GET"])
def get_state():
    _prefeeder_maybe_refresh_ui()
    return jsonify({"ok": True, **_public_machine_state()})


@app.route("/api/events/clear", methods=["POST"])
def clear_events():
    rev = _clear_event_log()
    return jsonify({"ok": True, "eventos_rev": rev})


@app.route("/api/errors", methods=["GET"])
def api_errors():
    return jsonify({"ok": True, "catalog": error_codes.catalog_for_api()})


@app.route("/api/cycle", methods=["GET", "POST"])
def cycle_config():
    if request.method == "GET":
        params = _load_cycle_params()
        comps = _cut_comp_params_of(params)
        params["cut_comp_mode"] = True
        params["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
        params["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
        params["cut_offset_mm"] = 0.0
        return jsonify(
            {
                "ok": True,
                "steps": list(CYCLE_STEPS),
                "params": params,
                "state": _public_machine_state(),
            }
        )

    data = request.get_json(silent=True) or {}
    patch = data.get("params") or data
    if not isinstance(patch, dict):
        return jsonify({"ok": False, "error": "JSON inválido"}), 400
    params = _load_cycle_params()
    for key in _DEFAULT_CYCLE_PARAMS:
        # deposit_max_travel_mm es fijo en cycle.json (no editable por HMI).
        if key in patch and patch[key] is not None and key not in (
            "cut_comp_mode",
            "deposit_max_travel_mm",
        ):
            params[key] = patch[key]
    params["feed_sides"] = _feed_sides_of(params)
    for key in ("cut_comp_r_mm", "cut_comp_l_mm"):
        if key in patch and patch[key] is not None:
            params[key] = _num(patch.get(key), 0.0)
    comps = _cut_comp_params_of(params)
    params["cut_comp_mode"] = True
    params["cut_comp_r_mm"] = comps["cut_comp_r_mm"]
    params["cut_comp_l_mm"] = comps["cut_comp_l_mm"]
    params["cut_offset_mm"] = 0.0
    for key in (
        "deposit_stack_gap_mm",
        "deposit_max_travel_mm",
        "gripper_clearance_mm",
        "tolerance_mm",
        "blower_ms_per_mm",
        "cut_comp_r_mm",
        "cut_comp_l_mm",
    ):
        params[key] = _num(params.get(key), _DEFAULT_CYCLE_PARAMS[key])
    params["deposit_batch_size"] = int(_num(params.get("deposit_batch_size"), 50))
    _MOTION_DATA["tolerance_mm"] = _num(params.get("tolerance_mm"), 2.0)
    try:
        params["cantidad"] = max(1, int(_MACHINE_STATE.get("cantidad") or 1))
    except (TypeError, ValueError):
        params["cantidad"] = 1
    params["numero_modelo"] = str(_MACHINE_STATE.get("numero_modelo") or "").strip()
    params["modelo_desc"] = str(_MACHINE_STATE.get("modelo_desc") or "").strip()
    try:
        length = _MACHINE_STATE.get("longitud_modelo_mm")
        params["longitud_modelo_mm"] = float(length) if length is not None else None
    except (TypeError, ValueError):
        params["longitud_modelo_mm"] = None
    try:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        CYCLE_PATH.write_text(
            json.dumps(params, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar cycle.json"}), 500
    return jsonify(
        {
            "ok": True,
            "steps": list(CYCLE_STEPS),
            "params": params,
            "state": _public_machine_state(),
        }
    )


@app.route("/api/motion/tune", methods=["GET", "POST"])
def motion_tune():
    try:
        tune = _load_motion_tune()
    except (OSError, json.JSONDecodeError):
        return jsonify({"ok": False, "error": "No se pudo leer Motion.json"}), 500

    if request.method == "GET":
        return jsonify(
            {
                "ok": True,
                "tune": tune,
                **_motion_scale(),
                "esp32_line": _motion_tune_flat(tune),
            }
        )

    data = request.get_json(silent=True) or {}
    patch = data.get("tune") or data
    if not isinstance(patch, dict):
        return jsonify({"ok": False, "error": "JSON inválido"}), 400
    _deep_merge_motion_tune(tune, patch)
    tune = _canonical_motion_tune(tune)
    try:
        _save_motion_tune(tune)
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar Motion.json"}), 500
    return jsonify(
        {
            "ok": True,
            "tune": tune,
            **_motion_scale(),
            "esp32_line": _motion_tune_flat(tune),
        }
    )


def _encoder_from_status(replies):
    reading = {"R": None, "L": None}
    for raw in replies or []:
        try:
            msg = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if msg.get("op") != "status":
            continue
        for side in ("R", "L"):
            block = msg.get("enc_" + side)
            if not isinstance(block, dict):
                continue
            item = {}
            if block.get("counts") is not None:
                try:
                    item["counts"] = int(block["counts"])
                except (TypeError, ValueError):
                    pass
            elif block.get("counts_live") is not None:
                try:
                    item["counts"] = abs(int(block["counts_live"]))
                except (TypeError, ValueError):
                    pass
            if "laser" in block:
                item["laser"] = bool(block.get("laser"))
            if "joint" in block:
                item["joint"] = bool(block.get("joint"))
            if item:
                reading[side] = item
    return reading


@app.route("/api/motion/status", methods=["GET"])
def motion_status():
    sent = _send_motion_serial(
        ['{"op":"status"}'],
        line_timeout_s=2.0,
        reply_until_op="status",
        nonblocking=True,
    )
    lm_servo_on = None
    lm_homed = None
    feeder_ready = {"R": None, "L": None}
    feeder_servo_on = {"R": None, "L": None}
    safety = None
    for raw in reversed(sent.get("replies") or []):
        try:
            status_reply = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if status_reply.get("op") == "status":
            lm_servo_on = status_reply.get("lm_servo_on")
            lm_homed = status_reply.get("lm_homed")
            feeder_ready["R"] = status_reply.get("feeder_r_ready")
            feeder_ready["L"] = status_reply.get("feeder_l_ready")
            feeder_servo_on["R"] = status_reply.get("feeder_r_servo_on")
            feeder_servo_on["L"] = status_reply.get("feeder_l_servo_on")
            safety = status_reply.get("safety")
            break
    encoder = _encoder_from_status(sent.get("replies"))
    return jsonify(
        {
            "ok": True,
            "sent": bool(sent.get("sent")),
            "encoder": encoder,
            "lm_servo_on": lm_servo_on,
            "lm_homed": lm_homed,
            "feeder_ready": feeder_ready,
            "feeder_servo_on": feeder_servo_on,
            "safety": safety,
        }
    )


@app.route("/api/motion/cmd", methods=["POST"])
def motion_cmd():
    body = request.get_json(silent=True) or {}
    op = (body.get("op") or "").strip()
    try:
        tune = _load_motion_tune()
    except (OSError, json.JSONDecodeError):
        return jsonify({"ok": False, "error": "No se pudo leer Motion.json"}), 500

    patch = body.get("tune")
    if isinstance(patch, dict):
        _deep_merge_motion_tune(tune, patch)
        tune = _canonical_motion_tune(tune)

    if op == "feeder_test":
        side = str(body.get("side") or "").upper()
        if side not in ("R", "L"):
            return jsonify({"ok": False, "error": "El Test pide lado R o L."}), 400
        try:
            _save_motion_tune(tune)
        except OSError:
            return jsonify({"ok": False, "error": "No se pudo guardar Motion.json"}), 500
        result = _feeder_test(side, tune)
        result.update({"op": op, "tune": tune, **_motion_scale()})
        return jsonify(result)

    try:
        tune, lines = _build_motion_lines(op, tune, body)
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400

    if isinstance(patch, dict) or op in ("set_tune", "lmguide_move"):
        try:
            _save_motion_tune(tune)
        except OSError:
            return jsonify({"ok": False, "error": "No se pudo guardar Motion.json"}), 500

    if op == "lmguide_stop":
        sent = _send_motion_urgent(lines)
    elif op == "motion_hw_init":
        _actuators_set("initializing", "feeder")
        sent = _send_motion_serial(
            lines,
            line_timeout_s=120.0,
            reply_until_op="motion_hw_init",
            nonblocking=True,
            on_line=_hw_init_on_line,
        )
    else:
        sent = _send_motion_serial(lines, nonblocking=True)
    try:
        response_op = str(json.loads(lines[-1]).get("op") or op)
    except (IndexError, TypeError, json.JSONDecodeError):
        response_op = op
    command_ok = None
    for raw in reversed(sent.get("replies") or []):
        try:
            reply = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if reply.get("op") == response_op:
            command_ok = bool(reply.get("ok", True))
            break
    if op == "motion_hw_init":
        if command_ok is True:
            _actuators_set("online")
        else:
            _actuators_set("offline")
    return jsonify(
        {
            "ok": True,
            "op": op,
            "tune": tune,
            "lines": lines,
            "command_ok": command_ok,
            **_motion_scale(),
            **sent,
        }
    )


@app.route("/api/motion/data", methods=["GET", "POST"])
def motion_data():
    global _MOTION_DATA
    if request.method == "GET":
        return jsonify(
            {
                "ok": True,
                "data": dict(_MOTION_DATA),
                "esp32_line": _motion_data_flat(_MOTION_DATA),
            }
        )

    data = request.get_json(silent=True) or {}
    patch = data.get("data") or data
    if not isinstance(patch, dict):
        return jsonify({"ok": False, "error": "JSON inválido"}), 400
    for key in ("target_length_mm", "tolerance_mm", "modelo_id", "measure_comp_mm"):
        if key in patch:
            _MOTION_DATA[key] = patch[key]
    if "measure_comp_mm" in patch:
        # Campo legado del protocolo Motion; ya no alimenta el plan de corte HMI.
        try:
            comp = float(patch["measure_comp_mm"])
        except (TypeError, ValueError):
            comp = 0.0
        _MOTION_DATA["measure_comp_mm"] = comp
        _MACHINE_STATE["compensacion_medicion_mm"] = 0.0
    flat = _motion_data_flat(_MOTION_DATA)
    return jsonify({"ok": True, "data": dict(_MOTION_DATA), "esp32_line": flat})


@app.route("/api/plc/status", methods=["GET"])
def plc_status():
    status = _plc_status_get()
    # Blower vive en Motion; se fusiona para la UI del panel manual.
    if isinstance(status, dict):
        status = dict(status)
        status["blower"] = bool(_MOTION_BLOWER_ON)
    elif status is None and _serial_port("motion"):
        status = {"blower": bool(_MOTION_BLOWER_ON)}
    return jsonify(
        {
            "ok": True,
            "sent": bool(_plc_serial_configured() and status is not None),
            "status": status,
            "active_errors": _plc_active_errors(status if isinstance(status, dict) else None),
        }
    )


@app.route("/api/plc/cmd", methods=["POST"])
def plc_cmd():
    body = request.get_json(silent=True) or {}
    op = (body.get("op") or "").strip()
    # Blower: MCU Motion (UI del panel PLC sigue usando este endpoint).
    if op in ("blower_on", "blower_off"):
        sent = _motion_blower_send({"op": op, **body}, push_log=False)
        packed = _compact(sent.get("last_payload") or {"op": op})
        command_ok = None
        for raw in reversed(sent.get("replies") or []):
            try:
                reply = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                continue
            if reply.get("op") == op:
                command_ok = bool(reply.get("ok", True))
                break
        status = _plc_status_get() or {}
        if isinstance(status, dict):
            status = dict(status)
            status["blower"] = bool(_MOTION_BLOWER_ON)
        return jsonify(
            {
                "ok": True,
                "op": op,
                "lines": [packed],
                "command_ok": command_ok,
                "status": status,
                "active_errors": _plc_active_errors(status),
                **sent,
            }
        )
    try:
        line = _build_plc_line(op, body)
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    sent = _plc_send_payload(line, push_log=False)
    packed = _compact(line)
    command_ok = None
    for raw in reversed(sent.get("replies") or []):
        try:
            reply = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            continue
        if reply.get("op") == op:
            command_ok = bool(reply.get("ok", True))
            break
    status = _plc_status_after_command(sent)
    if sent.get("sent") and op in ("valve", "all_off", "reset"):
        status = _plc_wait_ready(2.5) or status
    if isinstance(status, dict):
        status = dict(status)
        status["blower"] = bool(_MOTION_BLOWER_ON)
    return jsonify(
        {
            "ok": True,
            "op": op,
            "lines": [packed],
            "command_ok": command_ok,
            "status": status,
            "active_errors": _plc_active_errors(status),
            **sent,
        }
    )


@app.route("/api/prefeeder/tune", methods=["GET", "POST"])
def prefeeder_tune():
    try:
        tune = _load_prefeeder_tune()
    except (OSError, json.JSONDecodeError):
        return jsonify({"ok": False, "error": "No se pudo leer PreFeeder.json"}), 500

    if request.method == "GET":
        # Leer actuales del maestro (NVS). PreFeeder.json solo es espejo local.
        st = None
        if _serial_port("prefeeder"):
            st = _prefeeder_status_get() or _PF_CACHED_STATUS
            if st:
                _prefeeder_reconcile_pushed_from_status(st)
        changed = False
        for side in ("L", "R"):
            live = _prefeeder_cfg_from_status_side(st, side)
            if live is not None:
                if _prefeeder_cfg_signature(tune[side]) != _prefeeder_cfg_signature(live):
                    changed = True
                tune[side] = live
        if changed:
            try:
                _save_prefeeder_tune(tune)
            except OSError:
                pass
        return jsonify({"ok": True, "tune": tune, "live": bool(st)})

    data = request.get_json(silent=True) or {}
    patch = data.get("tune") or data
    if not isinstance(patch, dict):
        return jsonify({"ok": False, "error": "JSON inválido"}), 400
    merged = dict(tune)
    for side in ("L", "R"):
        if isinstance(patch.get(side), dict):
            merged[side] = {**(tune.get(side) or {}), **patch[side]}
    next_tune = _canonical_prefeeder_tune(merged)

    # Guardar = escribir al maestro. El JSON local solo espeja lo confirmado.
    push = {"L": False, "R": False}
    applied: dict[str, bool | None] = {"L": None, "R": None}
    if not _serial_port("prefeeder"):
        return jsonify(
            {
                "ok": False,
                "error": "Sin COM del maestro; no se pueden guardar parámetros en el ESP32.",
                "tune": next_tune,
                "pushed": push,
                "applied": applied,
            }
        ), 400

    if _PF_CACHED_STATUS:
        _prefeeder_reconcile_pushed_from_status(_PF_CACHED_STATUS)
    for side in ("L", "R"):
        if not isinstance(patch.get(side), dict):
            continue
        ok, _changed = _prefeeder_set_config_side(side, next_tune[side])
        push[side] = ok
    st = _prefeeder_refresh_cache()
    for side in ("L", "R"):
        live = _prefeeder_cfg_from_status_side(st, side)
        if isinstance(patch.get(side), dict):
            if live is None:
                applied[side] = False
            else:
                applied[side] = (
                    _prefeeder_cfg_signature(live)
                    == _prefeeder_cfg_signature(next_tune[side])
                )
        if live is not None:
            next_tune[side] = live
    # Espejo local = lo que quedó en el maestro.
    try:
        _save_prefeeder_tune(next_tune)
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar PreFeeder.json"}), 500
    return jsonify(
        {"ok": True, "tune": next_tune, "pushed": push, "applied": applied}
    )


@app.route("/api/prefeeder/status", methods=["GET"])
def prefeeder_status():
    global _PF_CACHED_STATUS
    port_ok = bool(_serial_port("prefeeder"))
    force = request.args.get("force") == "1"
    status = None
    if port_ok and (force or _prefeeder_comm_active()):
        status = _prefeeder_refresh_cache()
        _prefeeder_watch_once()
    elif _PF_CACHED_STATUS:
        status = _PF_CACHED_STATUS
    return jsonify(
        {
            "ok": True,
            "sent": bool(port_ok and status is not None),
            "status": status,
            "active_errors": _prefeeder_active_errors(status),
        }
    )


@app.route("/api/prefeeder/cmd", methods=["POST"])
def prefeeder_cmd():
    body = request.get_json(silent=True) or {}
    op = str(body.get("op") or "").strip()
    if op == "set_force_feed":
        side = str(body.get("side") or "").upper()
        on = bool(body.get("on"))
        if side not in ("L", "R"):
            return jsonify({"ok": False, "error": "Lado no reconocido."}), 400
        if not _prefeeder_required():
            return jsonify({"ok": False, "error": "Sin COM del PreAlimentador."}), 400
        ok = _prefeeder_set_force(side, on)
        if ok:
            with _PF_MANUAL_HOLD_LOCK:
                _PF_MANUAL_HOLD[side] = on
                other = "R" if side == "L" else "L"
                any_hold = _PF_MANUAL_HOLD["L"] or _PF_MANUAL_HOLD["R"]
            if not on and not any_hold:
                with _STATE_LOCK:
                    in_cycle = bool(_MACHINE_STATE.get("ciclo_en_curso"))
                    running = _MACHINE_STATE.get("estado") == "running"
                    rutina = bool(_MACHINE_STATE.get("rutina"))
                if not in_cycle and not running and not rutina:
                    _prefeeder_set_feed_lock(True)
        return jsonify({"ok": ok, "op": op, "side": side, "on": on})
    return jsonify({"ok": False, "error": "Orden no reconocida."}), 400


@app.route("/api/serial", methods=["GET", "POST"])
def serial_ports():
    """Puertos serie elegidos en la pantalla."""
    if request.method == "GET":
        # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR (GET: ad, andon_skipped, andon_none_value) ---
        listed = _ports_for_ui()
        _prune_missing_serial_assignments(listed)
        with _serial_cfg_lock:
            ad = bool(_serial_cfg.get("andon_disabled"))
        return jsonify(
            {
                "ok": True,
                "ports": listed,
                "motion": _serial_port("motion"),
                "plc": _serial_port("plc"),
                "andon": _serial_port("andon"),
                "prefeeder": _serial_port("prefeeder"),
                "online": _online_snapshot(listed),
                "andon_disabled": ad,
                "andon_skipped": _andon_skipped_effective(),
                "andon_none_value": _ANDON_NONE_TOKEN,
            }
        )

    body = request.get_json(silent=True) or {}
    listed = _ports_for_ui()
    motion_raw = str(body.get("motion") or "").strip()
    plc_raw = str(body.get("plc") or "").strip()
    andon_raw = str(body.get("andon") or "").strip()
    prefeeder_raw = str(body.get("prefeeder") or "").strip()
    prev_motion = _serial_port("motion")
    prev_plc = _serial_port("plc")
    prev_andon = _serial_port("andon")
    prev_prefeeder = _serial_port("prefeeder")
    motion = _accept_assigned_port(motion_raw, listed, prev_motion)
    plc = _accept_assigned_port(plc_raw, listed, prev_plc)
    prefeeder = _accept_assigned_port(prefeeder_raw, listed, prev_prefeeder)
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR (POST: token __none__ y andon_disable_ui) ---
    andon_disable_ui = andon_raw == _ANDON_NONE_TOKEN
    if andon_disable_ui:
        andon = ""
    else:
        andon = _accept_assigned_port(andon_raw, listed, prev_andon)
    if motion_raw and not motion:
        return jsonify({"ok": False, "error": "El puerto de Motion no está conectado."}), 400
    if plc_raw and not plc:
        return jsonify({"ok": False, "error": "El puerto del PLC no está conectado."}), 400
    if andon_raw and not andon_disable_ui and not andon:
        return jsonify({"ok": False, "error": "El puerto del Andon no está conectado."}), 400
    if prefeeder_raw and not prefeeder:
        return jsonify({"ok": False, "error": "El puerto del PreAlimentador no está conectado."}), 400
    chosen = [port for port in (motion, plc, andon, prefeeder) if port]
    if len(chosen) != len({_port_compare_key(port) for port in chosen}):
        return jsonify({"ok": False, "error": "Cada módulo tiene que usar un puerto distinto."}), 400

    with _serial_cfg_lock:
        checklist_bypass = bool(_serial_cfg.get("checklist_bypass"))
    try:
        _save_serial_cfg(motion, plc, andon, andon_disable_ui, checklist_bypass, prefeeder)
    except OSError:
        return jsonify({"ok": False, "error": "No se pudo guardar el puerto."}), 500
    if prefeeder:
        _prefeeder_adopt_live_config()

    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR (respuesta POST: campos extras) ---
    return jsonify(
        {
            "ok": True,
            "motion": motion,
            "plc": plc,
            "andon": andon,
            "prefeeder": prefeeder,
            "online": _online_snapshot(),
            "andon_disabled": andon_disable_ui,
            "andon_skipped": _andon_skipped_effective(),
            "andon_none_value": _ANDON_NONE_TOKEN,
        }
    )


@app.route("/api/settings/checklist-bypass", methods=["GET", "POST"])
def checklist_bypass_setting():
    if request.method == "POST":
        body = request.get_json(silent=True) or {}
        enabled = bool(body.get("enabled"))
        with _serial_cfg_lock:
            motion = str(_serial_cfg.get("motion") or "")
            plc = str(_serial_cfg.get("plc") or "")
            andon = str(_serial_cfg.get("andon") or "")
            prefeeder = str(_serial_cfg.get("prefeeder") or "")
            andon_disabled = bool(_serial_cfg.get("andon_disabled"))
        try:
            _save_serial_cfg(motion, plc, andon, andon_disabled, enabled, prefeeder)
        except OSError:
            return jsonify({"ok": False, "error": "No se pudo guardar la configuración."}), 500
        _push_event("Bypass de checklist activo." if enabled else "Checklist obligatorio.")
    return jsonify({"ok": True, "enabled": _checklist_bypassed()})


@app.route("/api/serial/test", methods=["POST"])
def serial_module_test():
    """Comprueba que el COM elegido responde y es el MCU pedido. Andon incluido."""
    body = request.get_json(silent=True) or {}
    which = str(body.get("module") or "").strip().lower()
    if which not in ("motion", "plc", "andon", "prefeeder"):
        return jsonify({"ok": False, "error": "Módulo no reconocido."}), 400
    # --- ANDON_SKIP_OPCIONAL — TODO_BORRAR (early return si Andon omitido) ---
    if which == "andon" and _andon_skipped_effective():
        msg = "Andon no usado (Conexión o HMI_ANDON_SKIP=1). No se prueba COM."
        try:
            _set_module_online(which, ok=True, message=msg)
        except OSError:
            pass
        return jsonify(
            {
                "ok": True,
                "module": which,
                "skipped": True,
                "message": msg,
                "online": _online_snapshot(),
            }
        )
    # ---
    ok, code = _check_module_link(which, status_timeout_s=15.0)
    if not ok:
        err = error_codes.format_message(code)
        try:
            _set_module_online(which, ok=False, message=err)
        except OSError:
            pass
        return jsonify(
            {
                "ok": False,
                "module": which,
                "code": code,
                "error": err,
                "online": _online_snapshot(),
            }
        )
    port = _serial_port(which)
    msg = f"{which.upper()} en {port} en línea."
    try:
        _set_module_online(which, ok=True, message=msg)
    except OSError:
        pass
    return jsonify(
        {
            "ok": True,
            "module": which,
            "port": port,
            "message": msg,
            "online": _online_snapshot(),
        }
    )


def _action_rejected(err: str):
    """El rechazo queda en el letrero. El panel de eventos puede estar cerrado."""
    if err:
        _push_event(err, "error")
        with _STATE_LOCK:
            already_error = (
                _MACHINE_STATE.get("estado") == "error"
                or bool(str(_MACHINE_STATE.get("ciclo_error") or "").strip())
            )
        # Si ya hay error/recuperación, no relatchear otro código (p. ej. Reset fallido
        # por COM que devolvía E019 y dejaba al operador atrapado).
        if already_error:
            _set_machine_ui(mensaje=err)
        else:
            head = (err.split(maxsplit=1)[0] if err else "") or ""
            if len(head) >= 4 and head[0] == "E" and head[1:].isdigit():
                code = head
                _recovery_begin(code, err)
                _set_machine_ui(
                    estado="error",
                    estado_label="Error",
                    mensaje=err,
                    ciclo_error=code,
                )
            else:
                _set_machine_ui(mensaje=err)
    return jsonify({"ok": False, "error": err, "state": _public_machine_state()}), 400


@app.route("/api/action", methods=["POST"])
def action():
    """Orquestador: acciones de proceso (Start / Pausa / …) y registro del resto."""
    data = request.get_json(silent=True) or {}
    block = data.get("block", "")
    action_name = (data.get("action") or "").strip()
    message = ""

    if action_name in _ROUTINE_ACTIONS:
        ok, err = _ROUTINE_ACTIONS[action_name](data)
        if not ok:
            return _action_rejected(err)
        message = err or "Listo."
    elif block == "proceso":
        if action_name == "start":
            ok, err = _cycle_start()
            if not ok:
                return _action_rejected(err)
            message = "Lote iniciado."
        elif action_name == "step_start":
            ok, err = _cycle_step_start()
            if not ok:
                return _action_rejected(err)
            message = err or "Prueba paso a paso iniciada."
        elif action_name == "step_next":
            ok, err = _cycle_step_next()
            if not ok:
                return _action_rejected(err)
            message = err or "Siguiente paso."
        elif action_name == "pause":
            _cycle_pause()
            message = "Pausa solicitada."
        elif action_name == "resume":
            ok, err = _cycle_resume()
            if not ok:
                return _action_rejected(err)
            message = "Reanudar."
        elif action_name == "stop":
            _cycle_stop()
            message = "Detener solicitado."
        elif action_name == "reset":
            ok, err = _cycle_reset_error()
            if not ok:
                return _action_rejected(err)
            message = "Reset."
        else:
            message = f"Acción proceso: {action_name}"
    else:
        message = f"Acción registrada: {block} / {action_name}"

    return jsonify(
        {
            "ok": True,
            "block": block,
            "action": action_name,
            "message": message,
            "state": _public_machine_state(),
        }
    )


_hydrate_hmi_from_cycle()

threading.Thread(target=_prefeeder_watch_loop, daemon=True, name="prefeeder_watch").start()
threading.Thread(target=_prefeeder_boot_lock, daemon=True, name="prefeeder_boot").start()


def _terminate_sibling_app_py() -> None:
    """Cierra otras copias vivas de este app.py (no puede limpiar zombies de Windows)."""
    me = os.getpid()
    here = str(Path(__file__).resolve().parent).lower()
    try:
        raw = subprocess.check_output(
            [
                "powershell",
                "-NoProfile",
                "-Command",
                "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" "
                "| Select-Object ProcessId,CommandLine | ConvertTo-Json -Compress",
            ],
            text=True,
            timeout=15,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return
    if not raw.strip():
        return
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        return
    if isinstance(data, dict):
        data = [data]
    for row in data:
        pid = int(row.get("ProcessId") or 0)
        cmd = (row.get("CommandLine") or "").lower()
        cmd_norm = cmd.replace("/", "\\")
        if pid in (0, me) or "app.py" not in cmd_norm:
            continue
        if here.replace("/", "\\") not in cmd_norm:
            continue
        try:
            r = subprocess.run(
                ["taskkill", "/F", "/PID", str(pid)],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
            if r.returncode == 0:
                print(f"[HMI] Cerrada instancia previa PID {pid}", flush=True)
            else:
                print(
                    f"[HMI] PID {pid} no se pudo cerrar (posible zombie de Windows).",
                    flush=True,
                )
        except Exception:
            pass


def _port_free_exclusive(host: str, port: int) -> bool:
    """True si este proceso puede reclamar el puerto en exclusivo (Windows)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        sock.bind((host, port))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def _pick_hmi_port(host: str, preferred: int = 5000, span: int = 5) -> int:
    """Elige un puerto libre. Si 5000 está zombie, avanza al siguiente."""
    _terminate_sibling_app_py()
    bind_host = "0.0.0.0" if host in ("0.0.0.0", "::", "") else host
    for port in range(preferred, preferred + span):
        if _port_free_exclusive(bind_host, port):
            if port != preferred:
                print(
                    f"[HMI] Puerto {preferred} bloqueado (instancia zombie). "
                    f"Usando {port}. Abre http://127.0.0.1:{port}/",
                    flush=True,
                )
            return port
    print(
        f"[HMI] No hay puerto libre en {preferred}–{preferred + span - 1}. "
        "Reinicia Windows para liberar el socket zombie.",
        flush=True,
    )
    sys.exit(1)


def _harden_wsgi_bind() -> None:
    """Sin SO_REUSEADDR: evita dos listeners en el mismo puerto (HMI en blanco)."""
    BaseWSGIServer.allow_reuse_address = False
    _orig_bind = BaseWSGIServer.server_bind

    def _bind_exclusive(self):
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        return _orig_bind(self)

    BaseWSGIServer.server_bind = _bind_exclusive  # type: ignore[method-assign]


if __name__ == "__main__":
    # Sin una línea por cada GET de sondeo del HMI (~400–800 ms).
    # use_reloader=False: el reloader mata el hilo del ciclo a mitad de un
    # lmguide_move / feed_stage y al reiniciar deja la HMI en «En espera»
    # sin código de error (estado en RAM se pierde).
    _hmi_host = "0.0.0.0"
    _hmi_port = _pick_hmi_port(_hmi_host, preferred=5000)
    _harden_wsgi_bind()
    app.run(
        debug=True,
        use_reloader=False,
        threaded=True,
        host=_hmi_host,
        port=_hmi_port,
        request_handler=_QuietRequestHandler,
    )
