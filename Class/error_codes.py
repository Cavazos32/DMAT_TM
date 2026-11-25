"""
Catálogo único de errores de la aplicación / orquestador.

Formato visible (obligatorio): E000 Módulo: Descripción.

La descripción del catálogo es solo el fallo. Las acciones de operador van en
INSTRUCTIONS (apartado de instrucciones / recovery.guide), no en banner ni en
la caja de error.
"""

from __future__ import annotations

import re
from typing import Any, Iterable

# code -> (module, description) — solo el error, sin instrucciones.
CATALOG: dict[str, tuple[str, str]] = {
    "E000": ("Main", "Error desconocido."),
    "E001": ("Main", "No se puede alimentar porque holder y encoder no están activos."),
    "E002": ("Main", "No se pueden accionar las pinzas porque la alimentación L/R no ha sido validada."),
    "E003": ("Main", "No se puede ir a corte porque las pinzas, holder o encoder no están en el estado requerido."),
    "E004": ("Main", "No se puede cortar porque LM Guide no está en posición de corte."),
    "E005": ("Main", "No se puede descargar porque el corte no se ha completado."),
    "E006": ("Main", "No se puede regresar a Home porque la pieza no se ha descargado."),
    "E007": ("Main", "Longitud del modelo no definida."),
    "E010": ("PLC", "Sin respuesta de status."),
    "E011": ("PLC", "Holder no confirmado en estado lógico activo."),
    "E012": ("PLC", "Encoder no confirmado en estado lógico activo."),
    "E013": ("PLC", "Pinzas no confirmadas activas."),
    "E014": ("PLC", "Holder no confirmado liberado."),
    "E015": ("PLC", "Encoder no confirmado liberado."),
    "E017": ("PLC", "No hay puerto seleccionado."),
    "E018": ("PLC", "El puerto seleccionado no es el MCU PLC."),
    "E019": ("PLC", "El puerto no respondió."),
    "E020": ("PLC", "Fallo de holder desde el PLC físico."),
    "E021": ("PLC", "Fallo de encoder desde el PLC físico."),
    "E022": ("PLC", "Fallo de cortador desde el PLC físico."),
    "E023": ("PLC", "Fallo de pinzas desde el PLC físico."),
    "E030": ("Motion", "Alimentación ocupada."),
    "E031": ("Motion", "Longitud objetivo no definida."),
    "E032": ("Motion", "Alimentadores CAN no listos."),
    "E033": ("Motion", "Timeout de láser lado R."),
    "E034": ("Motion", "Timeout de láser lado L."),
    "E035": ("Motion", "Encoder fuera de tolerancia lado R."),
    "E036": ("Motion", "Encoder fuera de tolerancia lado L."),
    "E037": ("Motion", "Fallo al aplicar compensación."),
    "E038": ("Motion", "Fallo durante la alimentación."),
    "E039": ("Motion", "Señal de seguridad activa."),
    "E040": ("Motion", "Sin respuesta de status."),
    "E041": ("Motion", "No hay puerto seleccionado."),
    "E042": ("Motion", "El puerto seleccionado no es el MCU Motion."),
    "E043": ("Motion", "El puerto no respondió."),
    "E044": ("Motion", "No se confirmó la posición de destino del LM Guide."),
    "E045": ("Motion", "Alimentación interrumpida."),
    "E046": ("Main", "No se confirmó el restablecimiento del sistema."),
    "E050": ("Motion", "Joint detectado en lado R."),
    "E051": ("Motion", "Joint detectado en lado L."),
    "E052": ("Motion", "Joint detectado en ambos lados."),
    "E053": ("Motion", "Material terminado en lado R."),
    "E054": ("Motion", "Material terminado en lado L."),
    "E055": ("Motion", "Material terminado en ambos lados."),
    "E070": ("Andon", "No hay puerto seleccionado."),
    "E071": ("Andon", "El puerto seleccionado no es el MCU Andon."),
    "E072": ("Andon", "El puerto no respondió."),
    "E090": ("PreAlimentador", "No hay puerto seleccionado."),
    "E091": ("PreAlimentador", "El puerto seleccionado no es el maestro PreAlimentador."),
    "E092": ("PreAlimentador", "El puerto no respondió."),
    "E093": ("PreAlimentador L", "Falla de cilindro."),
    "E095": ("PreAlimentador L", "Holgura no confirmada."),
    "E096": ("PreAlimentador R", "Falla de cilindro."),
    "E098": ("PreAlimentador R", "Holgura no confirmada."),
    "E099": ("PreAlimentador", "No está listo ni estabilizado."),
}

# Error | Comunicación | Warning — según ERRORS.md.
CLASSIFICATION: dict[str, str] = {
    "E000": "Error",
    "E001": "Error",
    "E002": "Error",
    "E003": "Error",
    "E004": "Error",
    "E005": "Error",
    "E006": "Error",
    "E007": "Error",
    "E010": "Comunicación",
    "E011": "Error",
    "E012": "Error",
    "E013": "Error",
    "E014": "Error",
    "E015": "Error",
    "E017": "Comunicación",
    "E018": "Comunicación",
    "E019": "Comunicación",
    "E020": "Error",
    "E021": "Error",
    "E022": "Error",
    "E023": "Error",
    "E030": "Error",
    "E031": "Error",
    "E032": "Error",
    "E033": "Error",
    "E034": "Error",
    "E035": "Error",
    "E036": "Error",
    "E037": "Error",
    "E038": "Error",
    "E039": "Error",
    "E040": "Comunicación",
    "E041": "Comunicación",
    "E042": "Comunicación",
    "E043": "Comunicación",
    "E044": "Error",
    "E045": "Error",
    "E046": "Error",
    "E050": "Warning",
    "E051": "Warning",
    "E052": "Warning",
    "E053": "Warning",
    "E054": "Warning",
    "E055": "Warning",
    "E070": "Comunicación",
    "E071": "Comunicación",
    "E072": "Comunicación",
    "E090": "Comunicación",
    "E091": "Comunicación",
    "E092": "Comunicación",
    "E093": "Error",
    "E095": "Error",
    "E096": "Error",
    "E098": "Error",
    "E099": "Error",
}

# Acciones de operador: solo apartado de instrucciones (recovery.guide), nunca banner/caja.
INSTRUCTIONS: dict[str, str] = {
    "E000": "Solicita revisión técnica; no reanudes hasta identificar la condición.",
    "E001": "Verifica los indicadores de holder y encoder; si no confirman, solicita revisión técnica.",
    "E002": "Verifica el resultado de alimentación de ambos lados habilitados; revisa el error específico si aparece.",
    "E003": "Verifica los estados de las tres sujeciones en la HMI; no ordenes movimiento hasta confirmar la condición requerida.",
    "E004": "Verifica la posición indicada del LM Guide; solicita revisión si no coincide.",
    "E005": "Verifica la confirmación de corte; no retires material ni repitas el corte manualmente.",
    "E006": "Verifica la confirmación de descarga; no ordenes el regreso del eje hasta confirmar la condición.",
    "E007": "Verifica el modelo seleccionado y su longitud programada.",
    "E010": "Abre Conexión y prueba el puerto del PLC; si no responde, solicita revisión del enlace.",
    "E011": "Verifica la confirmación de cierre del holder en la HMI.",
    "E012": "Verifica la confirmación de cierre del encoder en la HMI.",
    "E013": "Verifica la confirmación de cierre de las pinzas; no intervengas mientras puedan moverse.",
    "E014": "Verifica la confirmación de apertura del holder; no fuerces la sujeción.",
    "E015": "Verifica la confirmación de apertura del encoder; no fuerces la sujeción.",
    "E017": "Abre Conexión, selecciona el puerto del PLC y pulsa Probar.",
    "E018": "Abre Conexión, selecciona el puerto correcto del PLC y pulsa Probar.",
    "E019": "Abre Conexión y prueba el puerto del PLC; si persiste, solicita revisión.",
    "E020": "Consulta el fallo del holder; solicita revisión de la sujeción y su confirmación.",
    "E021": "Consulta el fallo del encoder; solicita revisión de su sujeción y confirmación.",
    "E022": "No introduzcas las manos; solicita revisión del cortador y su condición segura.",
    "E023": "Verifica la indicación de pinzas; solicita revisión antes de intervenir en el material.",
    "E030": "Espera a que finalice la operación activa; si permanece ocupada, solicita revisión técnica.",
    "E031": "Verifica el modelo y la longitud objetivo antes de volver a alimentar.",
    "E032": "Verifica el estado de los alimentadores; solicita revisión si no están listos.",
    "E033": "Verifica la presencia de material y el estado del láser derecho; no fuerces el avance.",
    "E034": "Verifica la presencia de material y el estado del láser izquierdo; no fuerces el avance.",
    "E035": "Verifica la medición del lado derecho y la longitud del modelo; solicita revisión si persiste.",
    "E036": "Verifica la medición del lado izquierdo y la longitud del modelo; solicita revisión si persiste.",
    "E037": "Verifica el ajuste de compensación del modelo; solicita revisión antes de repetir el movimiento.",
    "E038": "Verifica el estado de alimentación y solicita revisión si no se identifica una causa específica.",
    "E039": "Identifica la condición de seguridad activa; no restablezcas ni reanudes hasta verificar que es seguro.",
    "E040": "Abre Conexión y prueba el puerto de Motion; si no responde, solicita revisión.",
    "E041": "Abre Conexión, selecciona el puerto de Motion y pulsa Probar.",
    "E042": "Abre Conexión, selecciona el puerto correcto de Motion y pulsa Probar.",
    "E043": "Abre Conexión y prueba el puerto de Motion; si persiste, solicita revisión.",
    "E044": "Verifica la posición indicada y el estado del drive; no retires material ni ordenes movimiento hasta validar una posición segura.",
    "E045": "Verifica por qué se detuvo la alimentación y que no exista una orden de paro activa.",
    "E046": "Consulta si existe un código específico activo; si no se identifica la causa, solicita revisión técnica.",
    "E050": "Espera la habilitación de intervención; sigue Liberar y clasificación o purga según el estado del lado sano.",
    "E051": "Espera la habilitación de intervención; sigue Liberar y clasificación o purga según el estado del lado sano.",
    "E052": "Espera la habilitación de Liberar; confirma segregación y sigue la recuperación indicada.",
    "E053": "Verifica el suministro derecho; sigue la intervención y purga cuando la HMI las habilite.",
    "E054": "Verifica el suministro izquierdo; sigue la intervención y purga cuando la HMI las habilite.",
    "E055": "Verifica el suministro de ambos lados; sigue la segregación y recuperación de material indicada.",
    "E070": "Abre Conexión, selecciona el puerto de Andon y pulsa Probar.",
    "E071": "Abre Conexión, selecciona el puerto correcto de Andon y pulsa Probar.",
    "E072": "Abre Conexión y prueba el puerto de Andon; si persiste, solicita revisión.",
    "E090": "Abre Conexión, selecciona el puerto del maestro PreAlimentador y pulsa Probar.",
    "E091": "Abre Conexión, selecciona el puerto correcto del maestro y pulsa Probar.",
    "E092": "Abre Conexión y prueba el puerto del maestro PreAlimentador; si persiste, solicita revisión.",
    "E093": "Verifica el estado del cilindro izquierdo; solicita revisión antes de manipular el mecanismo.",
    "E095": "Verifica el suministro de material y el indicador de holgura izquierdo; solicita revisión si no se recupera.",
    "E096": "Verifica el estado del cilindro derecho; solicita revisión antes de manipular el mecanismo.",
    "E098": "Verifica el suministro de material y el indicador de holgura derecho; solicita revisión si no se recupera.",
    "E099": "Verifica el estado del PreAlimentador y las fallas activas; usa Iniciar únicamente cuando esté habilitado.",
}

_CODE_RE = re.compile(r"\bE\d{3}\b", re.IGNORECASE)

FEED_STAGE_TO_CODE: dict[str, str] = {
    "ok": "",
    "busy": "E030",
    "no_target": "E031",
    "feeder_not_ready": "E032",
    "laser_timeout_r": "E033",
    "laser_timeout_l": "E034",
    "encoder_tolerance_r": "E035",
    "encoder_tolerance_l": "E036",
    "comp_move": "E037",
    "safety": "E039",
    "stopped": "E045",
    "joint_r": "E050",
    "joint_l": "E051",
    "joint_rl": "E052",
    "material_r": "E053",
    "material_l": "E054",
    "material_rl": "E055",
    "feed_no_reply": "E038",
    "feed_overlap_timeout": "E038",
    "unknown": "E038",
}

PLC_FAULT_FIELDS: tuple[tuple[str, str], ...] = (
    ("fault_holder", "E020"),
    ("fault_encoder", "E021"),
    ("fault_cutter", "E022"),
    ("fault_gripper", "E023"),
)

# fault numérico del firmware PreFeeder (ErrorCode.h) por lado
PFEEDER_FAULT_MAP: dict[tuple[str, int], str] = {
    ("L", 3): "E093",
    ("L", 6): "E095",
    ("R", 3): "E096",
    ("R", 6): "E098",
}


def format_message(code: str, **kwargs: Any) -> str:
    module, text = CATALOG.get(code, CATALOG["E000"])
    if kwargs:
        try:
            text = text.format(**kwargs)
        except (KeyError, ValueError):
            pass
    return f"{code} {module}: {text}"


def codes_from_text(text: str) -> list[str]:
    """Códigos E0xx únicos en orden de aparición."""
    seen: set[str] = set()
    out: list[str] = []
    for m in _CODE_RE.findall(str(text or "")):
        code = m.upper()
        if code in seen:
            continue
        seen.add(code)
        out.append(code)
    return out


def format_messages(codes: Iterable[str], **kwargs: Any) -> str:
    """Une varios errores; conserva cada código distinto."""
    ordered: list[str] = []
    seen_code: set[str] = set()
    for raw in codes:
        code = str(raw or "").strip().upper()
        if not code or code in seen_code:
            continue
        seen_code.add(code)
        ordered.append(code)
    if not ordered:
        return format_message("E000", **kwargs)
    if len(ordered) == 1:
        return format_message(ordered[0], **kwargs)

    lines: list[str] = []
    for code in ordered:
        lines.append(format_message(code, **kwargs))
    return " · ".join(lines) if lines else format_message(ordered[0], **kwargs)


def format_instructions(codes: Iterable[str]) -> str:
    """Texto único para el apartado de instrucciones (sin repetir lo mismo)."""
    ordered: list[str] = []
    seen: set[str] = set()
    for raw in codes:
        code = str(raw or "").strip().upper()
        if not code or code in seen:
            continue
        seen.add(code)
        ordered.append(code)
    if not ordered:
        return ""

    texts: list[str] = []
    seen_text: set[str] = set()
    for code in ordered:
        text = str(INSTRUCTIONS.get(code) or "").strip()
        if not text:
            continue
        key = text.lower()
        if key in seen_text:
            continue
        seen_text.add(key)
        texts.append(text)
    return " ".join(texts)


def feed_stage_code(detail: str) -> str:
    key = (detail or "unknown").strip()
    code = FEED_STAGE_TO_CODE.get(key, "E038")
    if code == "E038":
        return code
    if code == "":
        return "E000"
    return code


def feed_stage_message(detail: str) -> str:
    code = feed_stage_code(detail)
    return format_message(code)


def plc_fault_codes(status: dict | None) -> list[str]:
    if not status:
        return []
    out: list[str] = []
    for field, code in PLC_FAULT_FIELDS:
        if status.get(field):
            out.append(code)
    return out


def plc_fault_messages(status: dict | None) -> list[str]:
    return [format_message(c) for c in plc_fault_codes(status)]


def prefeeder_fault_codes(status: dict | None) -> list[str]:
    if not status:
        return []
    out: list[str] = []
    for side in ("L", "R"):
        block = status.get(side)
        if not isinstance(block, dict):
            continue
        try:
            fault = int(block.get("fault") or 0)
        except (TypeError, ValueError):
            fault = 0
        if not fault:
            continue
        code = PFEEDER_FAULT_MAP.get((side, fault))
        if code:
            out.append(code)
    return out


def prefeeder_fault_messages(status: dict | None) -> list[str]:
    return [format_message(c) for c in prefeeder_fault_codes(status)]


def catalog_for_api() -> dict[str, dict[str, str]]:
    return {
        code: {
            "module": mod,
            "description": desc,
            "classification": CLASSIFICATION.get(code, "Error"),
            "label": format_message(code),
            "instructions": INSTRUCTIONS.get(code, ""),
        }
        for code, (mod, desc) in CATALOG.items()
    }
