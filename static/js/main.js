(function () {
  const STORAGE_EVENTS = "dmat_show_events_panel";
  const STORAGE_THEME = "dmat_theme";
  const STORAGE_MODEL = "dmat_modelo";
  const STORAGE_PLC_BLOWER_MS = "dmat_plc_blower_ms";
  const ACTUATOR_STAGE_ORDER = ["feeder", "lmguide", "home", "ready"];
  // Errores de enlace/COM: banner sin solución; Abrir → Parámetros / Conexión.
  const LINK_COM_ERROR_CODES = {
    E010: 1,
    E017: 1,
    E018: 1,
    E019: 1,
    E040: 1,
    E041: 1,
    E042: 1,
    E043: 1,
    E070: 1,
    E071: 1,
    E072: 1,
    E090: 1,
    E091: 1,
    E092: 1,
  };

  const els = {
    statusBanner: document.getElementById("statusBanner"),
    statusBannerLabel: document.getElementById("statusBannerLabel"),
    statusBannerDetail: document.getElementById("statusBannerDetail"),
    progressPct: document.getElementById("progressPct"),
    progressFill: document.getElementById("progressFill"),
    selectModelo: document.getElementById("selectModelo"),
    modelCombo: document.getElementById("modelCombo"),
    modelComboList: document.getElementById("modelComboList"),
    metricModeloDesc: document.getElementById("metricModeloDesc"),
    metricLongitud: document.getElementById("metricLongitud"),
    metricCantidad: document.getElementById("metricCantidad"),
    metricCutCompR: document.getElementById("metricCutCompR"),
    metricCutCompL: document.getElementById("metricCutCompL"),
    pieceTube: document.getElementById("pieceTube"),
    metricTiempoCiclo: document.getElementById("metricTiempoCiclo"),
    metricPiezasProducidas: document.getElementById("metricPiezasProducidas"),
    metricPiezasTotal: document.getElementById("metricPiezasTotal"),
    metricTiempoEstimado: document.getElementById("metricTiempoEstimado"),
    metricTiempoCorriendo: document.getElementById("metricTiempoCorriendo"),
    eventsPanel: document.getElementById("eventsPanel"),
    eventsLog: document.getElementById("eventsLog"),
    btnClearEvents: document.getElementById("btnClearEvents"),
    selectMotionPort: document.getElementById("selectMotionPort"),
    selectPlcPort: document.getElementById("selectPlcPort"),
    selectAndonPort: document.getElementById("selectAndonPort"),
    selectPrefeederPort: document.getElementById("selectPrefeederPort"),
    btnRefreshPorts: document.getElementById("btnRefreshPorts"),
    btnTestAllPorts: document.getElementById("btnTestAllPorts"),
    btnMotionTest: document.getElementById("btnMotionTest"),
    btnPlcTest: document.getElementById("btnPlcTest"),
    btnAndonTest: document.getElementById("btnAndonTest"),
    btnPrefeederTest: document.getElementById("btnPrefeederTest"),
    serialMotionStatus: document.getElementById("serialMotionStatus"),
    serialPlcStatus: document.getElementById("serialPlcStatus"),
    serialAndonStatus: document.getElementById("serialAndonStatus"),
    serialPrefeederStatus: document.getElementById("serialPrefeederStatus"),
    serialMotionBadge: document.getElementById("serialMotionBadge"),
    serialPlcBadge: document.getElementById("serialPlcBadge"),
    serialAndonBadge: document.getElementById("serialAndonBadge"),
    serialPrefeederBadge: document.getElementById("serialPrefeederBadge"),
    prefeederTuneForm: document.getElementById("prefeederTuneForm"),
    prefeederTuneReload: document.getElementById("prefeederTuneReload"),
    prefeederStatusRefresh: document.getElementById("prefeederStatusRefresh"),
    prefeederEsp32Hint: document.getElementById("prefeederEsp32Hint"),
    pfBanL: document.getElementById("pfBanL"),
    pfBanR: document.getElementById("pfBanR"),
    toggleEventsPanel: document.getElementById("toggleEventsPanel"),
    toggleChecklistBypass: document.getElementById("toggleChecklistBypass"),
    settingsModal: document.getElementById("settingsModal"),
    parametrosModal: document.getElementById("parametrosModal"),
    btnAppSettings: document.getElementById("btnAppSettings"),
    settingsModalClose: document.getElementById("settingsModalClose"),
    parametrosModalClose: document.getElementById("parametrosModalClose"),
    parametrosModalBack: document.getElementById("parametrosModalBack"),
    themeToggle: document.getElementById("themeToggle"),
    motionTuneForm: document.getElementById("motionTuneForm"),
    motionTuneReload: document.getElementById("motionTuneReload"),
    motionStatusRefresh: document.getElementById("motionStatusRefresh"),
    motionEsp32Hint: document.getElementById("motionEsp32Hint"),
    plcManualForm: document.getElementById("plcManualForm"),
    plcEsp32Hint: document.getElementById("plcEsp32Hint"),
    plcBlowerMs: document.getElementById("plcBlowerMs"),
    plcGraceNote: document.getElementById("plcGraceNote"),
    settingsTabs: document.querySelectorAll(".modal-tab[data-tab]"),
    settingsTabPanels: document.querySelectorAll("[data-tab-panel]"),
    cycleStepsList: document.getElementById("cycleStepsList"),
    cycleParamsForm: document.getElementById("cycleParamsForm"),
    cycleStepStart: document.getElementById("cycleStepStart"),
    cycleStepNext: document.getElementById("cycleStepNext"),
    cycleStepStop: document.getElementById("cycleStepStop"),
    cycleStepTestHint: document.getElementById("cycleStepTestHint"),
    statusBannerActions: document.getElementById("statusBannerActions"),
    checklistModal: document.getElementById("checklistModal"),
    checklistModalClose: document.getElementById("checklistModalClose"),
    checklistBody: document.getElementById("checklistBody"),
    mantModal: document.getElementById("mantModal"),
    mantModalClose: document.getElementById("mantModalClose"),
    mantCounter: document.getElementById("mantCounter"),
    mantCounterValue: document.getElementById("mantCounterValue"),
    mantEmpleado: document.getElementById("mantEmpleado"),
    mantReset: document.getElementById("mantReset"),
    mantResetForm: document.getElementById("mantResetForm"),
    mantConfirm: document.getElementById("mantConfirm"),
    mantCancel: document.getElementById("mantCancel"),
    empleadoModal: document.getElementById("empleadoModal"),
    empleadoModalClose: document.getElementById("empleadoModalClose"),
    empleadoInput: document.getElementById("empleadoInput"),
    empleadoConfirm: document.getElementById("empleadoConfirm"),
    btnOpenRecovery: document.getElementById("btnOpenRecovery"),
    actuatorsStrip: document.getElementById("actuatorsStrip"),
    actuatorsStatusLabel: document.getElementById("actuatorsStatusLabel"),
    btnInitActuators: document.getElementById("btnInitActuators"),
    recoveryModal: document.getElementById("recoveryModal"),
    recoveryModalClose: document.getElementById("recoveryModalClose"),
    recoveryModalTitle: document.getElementById("recoveryModalTitle"),
    recoveryStepLabel: document.getElementById("recoveryStepLabel"),
    recoverySkipLink: document.getElementById("recoverySkipLink"),
    recoveryLead: document.getElementById("recoveryLead"),
    recoveryErrorBox: document.getElementById("recoveryErrorBox"),
    recoveryCode: document.getElementById("recoveryCode"),
    recoveryErrorDesc: document.getElementById("recoveryErrorDesc"),
    recoveryText: document.getElementById("recoveryText"),
    recoveryJointLegend: document.getElementById("recoveryJointLegend"),
    recoverySensors: document.getElementById("recoverySensors"),
    recoverySensorR: document.getElementById("recoverySensorR"),
    recoverySensorL: document.getElementById("recoverySensorL"),
    recoveryActions: document.getElementById("recoveryActions"),
  };

  let eventsEnabled = false;
  let pendingEventos = [];
  let eventsRev = -1;
  let clearingLog = false;
  let lastProcessAction = null;
  let modelosCatalog = [];
  let syncingModelSelect = false;
  let cycleParamsDirty = false;
  let motionTuneDirty = false;
  let prefeederDirty = { L: false, R: false };
  let motionStepsPerMm = 770 / 45;
  let motionPuuSign = -1;
  let motionMmRes = 0.1;
  let motionFeedShaftIncPerRev = 10000;  // DSY 6091:02
  let motionEncPollTimer = null;
  let motionEncPollBusy = false;
  let motionCommandInFlight = false;
  let statePollTimer = null;
  let cycleStepsCatalog = [];
  let plcPollTimer = null;
  /** Último espejo PLC del /api/state (para Liberar en purga). */
  let lastPlcOutputs = null;
  let pfPollTimer = null;
  let checklistViewKey = "";
  let empleadoWait = null;
  let recoveryPollTimer = null;
  let recoveryPollBusy = false;
  let recoveryViewKey = "";
  let recoveryWantOpen = false;
  let recoveryAutoKey = "";
  /** "conexion" | "recovery" — destino del botón Abrir del banner. */
  let bannerOpenTarget = "recovery";
  let lastBannerErrorCode = "";

  const PLC_OUTPUTS = [
    ["holder", "holder"],
    ["encoder", "encoder"],
    ["cutter_r", "cutter_r"],
    ["cutter_l", "cutter_l"],
    ["gripper", "gripper"],
  ];

  const PLC_FAULTS = [
    ["plcFaultHolder", "fault_holder"],
    ["plcFaultEncoder", "fault_encoder"],
    ["plcFaultCutter", "fault_cutter"],
    ["plcFaultGripper", "fault_gripper"],
  ];

  const PROCESS_ACTIONS = new Set([
    "start",
    "step_start",
    "step_next",
    "resume",
    "pause",
    "stop",
    "reset",
  ]);

  const bannerClassByEstado = {
    idle: "status-banner--idle",
    running: "status-banner--running",
    paused: "status-banner--paused",
    stopped: "status-banner--stopped",
    error: "status-banner--error",
    manual: "status-banner--manual",
    warning: "status-banner--warning",
  };

  function formatTime(date) {
    return date.toLocaleTimeString("es-MX", {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
    });
  }

  function formatSeconds(total) {
    if (total == null || !Number.isFinite(Number(total)) || Number(total) < 0) {
      return "—";
    }
    const n = Number(total);
    const s = Math.floor(n);
    const m = Math.floor(s / 60);
    const r = s % 60;
    if (m > 0) {
      return m + " min " + String(r).padStart(2, "0") + " s";
    }
    return n.toFixed(1) + " s";
  }

  function formatPiezasCount(n) {
    const v = Number(n);
    if (!Number.isFinite(v) || v < 0) return "—";
    return String(Math.floor(v));
  }

  function formatMm(value) {
    if (value == null) return "—";
    return Number(value).toFixed(2);
  }

  function setCantidadInput(n, locked) {
    if (!els.metricCantidad) return;
    els.metricCantidad.disabled = !!locked;
    if (document.activeElement === els.metricCantidad) return;
    const v = parseInt(n, 10);
    els.metricCantidad.value = Number.isFinite(v) && v > 0 ? String(v) : "1";
  }

  async function saveCantidad(n) {
    try {
      const res = await fetch("/api/cantidad", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ cantidad: n }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se guardó la cantidad.", "error");
        if (data.state) applyState(data.state);
        return;
      }
      if (data.state) applyState(data.state);
    } catch (err) {
      pushLocalEvent("Sin conexión al guardar la cantidad.", "error");
    }
  }

  function setCutCompInput(el, mm) {
    if (!el) return;
    if (document.activeElement === el) return;
    const n = Number(mm);
    el.value = Number.isFinite(n) ? String(n) : "0";
  }

  async function saveCutCompSide(side, mm) {
    const key = side === "R" ? "cut_comp_r_mm" : "cut_comp_l_mm";
    const params = {};
    params[key] = mm;
    try {
      const res = await fetch("/api/cycle", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ params: params }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se guardó la compensación.", "error");
        return;
      }
      if (data.params) fillCycleParamsForm(data.params);
      if (data.state) applyState(data.state);
    } catch (err) {
      pushLocalEvent("No se guardó la compensación.", "error");
    }
  }

  function syncUiFromPersistedState(state) {
    if (!state) return;
    const modelo = state.numero_modelo || "";
    if (modelo) {
      try {
        localStorage.setItem(STORAGE_MODEL, modelo);
      } catch (e) {
        /* ignore */
      }
      syncModelSelect(modelo);
    }
    syncModeloDesc(state);
  }

  function findModeloInCatalog(modeloId) {
    const mid = String(modeloId || "").trim();
    if (!mid) return null;
    for (let i = 0; i < modelosCatalog.length; i++) {
      if (modelosCatalog[i].modelo === mid) return modelosCatalog[i];
    }
    return null;
  }

  function syncModeloDesc(stateOrModelo) {
    if (!els.metricModeloDesc) return;
    let desc = "";
    if (stateOrModelo && typeof stateOrModelo === "object") {
      desc = String(stateOrModelo.modelo_desc || "").trim();
      if (!desc) {
        const hit = findModeloInCatalog(stateOrModelo.numero_modelo);
        desc = hit && hit.desc ? String(hit.desc).trim() : "";
      }
    } else {
      const hit = findModeloInCatalog(stateOrModelo);
      desc = hit && hit.desc ? String(hit.desc).trim() : "";
    }
    els.metricModeloDesc.textContent = desc || "";
    els.metricModeloDesc.title = desc || "";
    els.metricModeloDesc.hidden = !desc;
  }

  function updatePieceVisual(state) {
    if (!els.pieceTube) return;
    const len = Number(state && state.longitud_modelo_mm);
    const hasLen = Number.isFinite(len) && len > 0;
    let scale = 0.35;
    if (hasLen) {
      let maxLen = len;
      modelosCatalog.forEach(function (item) {
        const n = Number(item.longitud_mm);
        if (Number.isFinite(n) && n > maxLen) maxLen = n;
      });
      scale = Math.max(0.2, Math.min(1, len / Math.max(maxLen, 1)));
    }
    els.pieceTube.style.setProperty("--piece-scale", String(scale));
    els.pieceTube.classList.toggle("is-empty", !hasLen);
  }

  async function maybeRestoreModelFromLocal() {
    let saved = "";
    try {
      saved = localStorage.getItem(STORAGE_MODEL) || "";
    } catch (e) {
      /* ignore */
    }
    if (!saved) return;
    if (els.selectModelo && String(els.selectModelo.value || "").trim()) return;
    if (findModeloInCatalog(saved)) {
      await selectModelo(saved, true);
    }
  }

  function setEventsPanelVisible(visible) {
    eventsEnabled = visible;
    els.eventsPanel.hidden = !visible;
    els.toggleEventsPanel.checked = visible;
    try {
      localStorage.setItem(STORAGE_EVENTS, visible ? "1" : "0");
    } catch (e) {
      /* ignore */
    }
    if (visible && pendingEventos.length) {
      renderEvents(pendingEventos);
    }
  }

  function getTheme() {
    const t = document.documentElement.getAttribute("data-theme");
    return t === "light" ? "light" : "dark";
  }

  function applyTheme(theme) {
    const next = theme === "light" ? "light" : "dark";
    document.documentElement.setAttribute("data-theme", next);
    try {
      localStorage.setItem(STORAGE_THEME, next);
    } catch (e) {
      /* ignore */
    }
    if (els.themeToggle) {
      els.themeToggle.setAttribute(
        "aria-label",
        next === "dark" ? "Activar modo claro" : "Activar modo oscuro"
      );
    }
  }

  function toggleTheme() {
    applyTheme(getTheme() === "dark" ? "light" : "dark");
  }

  function loadThemePreference() {
    try {
      const saved = localStorage.getItem(STORAGE_THEME);
      if (saved === "light" || saved === "dark") {
        applyTheme(saved);
        return;
      }
    } catch (e) {
      /* ignore */
    }
    applyTheme("light");
  }

  function loadEventsPreference() {
    try {
      const v = localStorage.getItem(STORAGE_EVENTS);
      setEventsPanelVisible(v === "1");
    } catch (e) {
      setEventsPanelVisible(false);
    }
  }

  function applyState(state) {
    if (!state) return;

    syncAndonCallButtons(state);

    const estado = state.estado || "idle";
    let bannerEstado = estado;
    const recNow = state.recovery || {};
    const recStep = String(recNow.step || "");
    const jointWarn =
      !!recNow.active && (recNow.kind === "joint" || recNow.kind === "material");
    // listo = recuperación terminada: ya no es banner de error (ni texto rojo al lado).
    const recoveryErrorActive =
      !!recNow.active &&
      recNow.kind !== "joint" &&
      recNow.kind !== "material" &&
      recStep !== "listo";
    if (state.rutina_manos) bannerEstado = "stopped";
    else if (state.rutina) bannerEstado = "manual";
    else if (jointWarn) bannerEstado = "warning";
    else if (recoveryErrorActive || estado === "error") bannerEstado = "error";
    els.statusBanner.className =
      "status-banner status-banner--full " +
      (bannerClassByEstado[bannerEstado] || bannerClassByEstado.idle);
    const showErrorLine =
      recoveryErrorActive || estado === "error" || jointWarn;
    if (showErrorLine) {
      const errCode = recNow.code || state.ciclo_error || "";
      const errMsg = recNow.mensaje || state.mensaje || "";
      const parsed = parseRecoveryError(errCode, errMsg);
      els.statusBannerLabel.textContent = formatErrorStandard(parsed, errCode);
    } else {
      els.statusBannerLabel.textContent = state.estado_label || "En espera";
    }
    // Derecha: solo botón Abrir. Sin mensajes de secuencia. Aviso de manos bajo el estado.
    if (els.statusBannerDetail) {
      const handsWarn = !!state.rutina_manos && !showErrorLine;
      let detail = handsWarn ? "NO INTRODUCIR MANOS" : "";
      if (!detail && state.sin_material) {
        detail = "Se requiere material en ambos lados";
      } else if (!detail && state.poco_material && !jointWarn) {
        detail = "Trabajando con poco material";
      }
      els.statusBannerDetail.textContent = detail;
      els.statusBannerDetail.classList.toggle(
        "is-warn",
        handsWarn || !!state.poco_material || !!state.sin_material
      );
    }
    const lowMatEl = document.getElementById("statusBannerLowMat");
    if (lowMatEl) {
      const showLow = !!state.poco_material && !state.sin_material;
      lowMatEl.hidden = !showLow;
      lowMatEl.textContent = "Trabajando con poco material";
    }

    const pct = Math.min(100, Math.max(0, Number(state.progreso_pct) || 0));
    els.progressPct.textContent = Math.round(pct) + " %";
    els.progressFill.style.width = pct + "%";

    syncModelSelect(state.numero_modelo || "");
    syncModeloDesc(state);
    els.metricLongitud.textContent = formatMm(state.longitud_modelo_mm);
    setCantidadInput(
      state.cantidad,
      !!state.ciclo_en_curso || !!state.charola_llena || !!state.lote_activo
    );
    setCutCompInput(els.metricCutCompR, state.cut_comp_r_mm);
    setCutCompInput(els.metricCutCompL, state.cut_comp_l_mm);
    updatePieceVisual(state);
    if (els.metricTiempoCiclo) {
      els.metricTiempoCiclo.textContent = formatSeconds(state.tiempo_ciclo_s);
    }
    if (els.metricPiezasProducidas) {
      els.metricPiezasProducidas.textContent = formatPiezasCount(
        state.piezas_producidas
      );
    }
    if (els.metricPiezasTotal) {
      const totalPiezas =
        state.piezas_total != null ? state.piezas_total : state.cantidad;
      els.metricPiezasTotal.textContent = formatPiezasCount(totalPiezas);
    }
    els.metricTiempoEstimado.textContent = formatSeconds(state.tiempo_estimado_s);
    els.metricTiempoCorriendo.textContent = formatSeconds(state.tiempo_corriendo_s);

    applyServerEvents(state, estado);

    syncProcessControls(state);
    syncCycleStepTestControls(state);
    syncRoutine(state);
    syncRecovery(state);
    renderCycleSteps(
      state.ciclo_paso_id || "",
      state.ciclo_paso_indice,
      state.ciclo_paralelo_id || ""
    );
    const pfCom = !!(state.prefeeder && state.prefeeder.com);
    applyActuatorsStatus(state.actuators);
    const actuatorsBusy =
      !!(state.actuators && state.actuators.state === "initializing");
    setStatePoll(
      estado === "running" ||
        estado === "paused" ||
        estado === "error" ||
        !!state.ciclo_en_curso ||
        !!state.lote_activo ||
        !!state.rutina ||
        pfCom ||
        actuatorsBusy
    );
  }

  function mapActuatorStageKey(state, stageRaw) {
    const st = String(state || "offline").toLowerCase();
    if (st === "online") return "ready";
    if (st !== "initializing") return "";
    const stage = String(stageRaw || "").trim().toLowerCase();
    if (!stage) return "feeder";
    if (stage.indexOf("feeder") >= 0) return "feeder";
    if (stage.indexOf("lm") >= 0 && stage.indexOf("home") < 0) return "lmguide";
    if (stage.indexOf("home") >= 0) return "home";
    if (stage.indexOf("ready") >= 0 || stage.indexOf("listo") >= 0) return "ready";
    return "feeder";
  }

  function actuatorStageLabel(stageKey) {
    if (stageKey === "feeder") return "Feeder";
    if (stageKey === "lmguide") return "LM Guide";
    if (stageKey === "home") return "Home";
    if (stageKey === "ready") return "Listo";
    return "Offline";
  }

  function applyActuatorsStatus(actuators) {
    const strip = els.actuatorsStrip;
    const label = els.actuatorsStatusLabel;
    const btn = els.btnInitActuators;
    if (!strip || !label) return;
    const act = actuators || {};
    const state = String(act.state || "offline").toLowerCase();
    const stageRaw = String(act.stage || "").trim();
    let mode = "offline";
    let stageKey = "";
    if (state === "online") {
      mode = "online";
      stageKey = "ready";
    } else if (state === "initializing") {
      mode = "initializing";
      stageKey = mapActuatorStageKey(state, stageRaw);
    }
    const text =
      mode === "online"
        ? "Listo"
        : mode === "initializing"
          ? actuatorStageLabel(stageKey)
          : "Offline";
    strip.setAttribute("data-actuators-state", mode);
    strip.setAttribute("data-actuators-stage", stageKey);
    label.textContent = text;
    const activeIdx = ACTUATOR_STAGE_ORDER.indexOf(stageKey);
    strip.querySelectorAll(".actuators-strip__stage").forEach(function (el) {
      const key = el.getAttribute("data-stage");
      const idx = ACTUATOR_STAGE_ORDER.indexOf(key);
      el.classList.toggle("is-active", mode === "initializing" && key === stageKey);
      el.classList.toggle(
        "is-done",
        mode === "online" || (activeIdx >= 0 && idx >= 0 && idx < activeIdx)
      );
    });
    if (btn) {
      btn.disabled = mode === "initializing";
      btn.title =
        mode === "initializing"
          ? "Inicialización en curso: " + text + "."
          : mode === "online"
            ? "Actuadores listos. Puedes volver a inicializar si hace falta."
            : "Inicializa feeder CAN y LM Guide (home).";
    }
  }

  function renderCycleSteps(activeId, activeIndex, parallelId) {
    if (!els.cycleStepsList) return;
    els.cycleStepsList.innerHTML = "";
    const steps = cycleStepsCatalog.length ? cycleStepsCatalog : [];
    steps.forEach(function (step, idx) {
      const li = document.createElement("li");
      li.className = "cycle-steps__item";
      li.dataset.stepId = step.id;
      if (step.id === activeId) {
        li.classList.add("is-active");
      }
      if (parallelId && step.id === parallelId) {
        li.classList.add("is-parallel");
        li.title = "Siguiente ciclo, en paralelo";
      }
      if (activeIndex != null && idx < activeIndex) {
        li.classList.add("is-done");
      }
      const idSpan = document.createElement("span");
      idSpan.className = "cycle-steps__id";
      idSpan.textContent = "Paso " + step.id;
      const labelSpan = document.createElement("span");
      labelSpan.className = "cycle-steps__label";
      labelSpan.textContent = step.label;
      const actorSpan = document.createElement("span");
      actorSpan.className = "cycle-steps__actor";
      actorSpan.textContent = step.actor;
      li.appendChild(idSpan);
      li.appendChild(labelSpan);
      li.appendChild(actorSpan);
      els.cycleStepsList.appendChild(li);
    });
  }

  function fillCycleParamsForm(params) {
    if (!els.cycleParamsForm || !params) return;
    if (cycleParamsDirty) return;
    const active = document.activeElement;
    const editing = !!(active && els.cycleParamsForm.contains(active));
    if (editing) return;
    els.cycleParamsForm.querySelectorAll("[data-cycle-param]").forEach(function (input) {
      const key = input.dataset.cycleParam;
      const val = params[key];
      if (input.type === "checkbox") {
        input.checked = !!val && val !== 0 && val !== "0" && val !== "false";
        return;
      }
      if (val == null || val === "") {
        input.value = "";
        return;
      }
      input.value = String(val);
    });
  }

  async function loadCycleConfig() {
    try {
      const res = await fetch("/api/cycle");
      const data = await res.json();
      if (!data.ok) return;
      cycleStepsCatalog = data.steps || [];
      fillCycleParamsForm(data.params);
      if (data.state) applyState(data.state);
      else renderCycleSteps("", 0);
    } catch (err) {
      pushLocalEvent("No se pudo cargar la secuencia de ciclo.", "error");
    }
  }

  async function saveCycleParams(ev) {
    if (ev) ev.preventDefault();
    if (!els.cycleParamsForm) return;
    const params = {};
    els.cycleParamsForm.querySelectorAll("[data-cycle-param]").forEach(function (input) {
      const key = input.dataset.cycleParam;
      if (input.type === "checkbox") {
        params[key] = !!input.checked;
        return;
      }
      const raw = input.value.trim();
      if (raw === "") {
        params[key] = null;
        return;
      }
      if (key === "feed_sides") {
        params[key] = raw.toUpperCase();
      } else if (key.indexOf("_mm") >= 0 || key.indexOf("per_mm") >= 0) {
        params[key] = parseFloat(raw);
      } else {
        params[key] = parseInt(raw, 10);
      }
    });
    try {
      const res = await fetch("/api/cycle", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ params: params }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se guardaron parámetros de ciclo.", "error");
        return;
      }
      cycleParamsDirty = false;
      if (data.params && els.cycleParamsForm) {
        els.cycleParamsForm.querySelectorAll("[data-cycle-param]").forEach(function (input) {
          const key = input.dataset.cycleParam;
          const val = data.params[key];
          if (input.type === "checkbox") {
            input.checked = !!val && val !== 0 && val !== "0" && val !== "false";
            return;
          }
          input.value = val == null || val === "" ? "" : String(val);
        });
      }
      if (data.state) applyState(data.state);
      if (data.tray_error) {
        pushLocalEvent(data.tray_error, "error");
      } else if (data.warn) {
        pushLocalEvent(data.warn, "warn");
      } else {
        pushLocalEvent("Parámetros de ciclo guardados.");
      }
    } catch (err) {
      pushLocalEvent("Sin conexión al guardar ciclo.", "error");
    }
  }

  function setStatePoll(on) {
    if (statePollTimer) {
      window.clearInterval(statePollTimer);
      statePollTimer = null;
    }
    if (!on) return;
    statePollTimer = window.setInterval(fetchState, 400);
  }

  function getProcessPanel() {
    return document.querySelector('[data-block="proceso"]');
  }

  function setToggleLabel(btn, on) {
    if (!btn) return;
    if (btn.getAttribute("data-motion-op") === "lmguide_servo") {
      btn.textContent = on ? "Encendido" : "Apagado";
      return;
    }
    if (btn.getAttribute("data-motion-op") === "feeder_servo") {
      btn.textContent = on ? "Enable" : "Disable";
      return;
    }
    if (btn.hasAttribute("data-plc-op")) {
      if (btn.hasAttribute("data-plc-keep-label")) return;
      btn.textContent = on ? "Encendido" : "Apagado";
    }
  }

  function plcHolderEncoderButton() {
    return document.querySelector(
      '#statusBannerActions [data-plc-name="holder_encoder"]'
    );
  }

  function motionCuttersButton() {
    return document.querySelector(
      '#motionTuneForm [data-plc-name="cutters"]'
    );
  }

  function syncMotionCuttersButton(status) {
    const btn = motionCuttersButton();
    if (!btn || !status) return;
    if (
      typeof status.cutter_r !== "boolean" ||
      typeof status.cutter_l !== "boolean"
    ) {
      return;
    }
    setPlcPressed(btn, status.cutter_r && status.cutter_l);
  }

  function syncPlcHolderEncoderButton(status) {
    const btn = plcHolderEncoderButton();
    if (!btn || !status) return;
    // No pisar el toggle optimista mientras la orden Liberar está en vuelo.
    if (btn.dataset.plcBusy === "1") return;
    if (status.known === false) return;
    if (
      typeof status.holder !== "boolean" ||
      typeof status.encoder !== "boolean"
    ) {
      return;
    }
    // Activo = holder y encoder ON (cerrados / sujetando).
    setPlcPressed(btn, status.holder && status.encoder);
  }

  function setRecoverySensorEl(el, on) {
    if (!el) return;
    el.textContent = on ? "Encendido" : "Apagado";
    el.classList.toggle("is-on", !!on);
    el.classList.toggle("is-off", !on);
  }

  function isLinkComError(code) {
    const c = String(code || "")
      .trim()
      .toUpperCase();
    return !!LINK_COM_ERROR_CODES[c];
  }

  function stripLinkComSolution(desc) {
    return String(desc || "")
      .replace(/\s*Elígelo en Parámetros[^.]*\.?/gi, "")
      .replace(/\s*Revisa el puerto en Parámetros[^.]*\.?/gi, "")
      .replace(/\s*Revisa el cable, la placa[^.]*\.?/gi, "")
      .replace(/\s{2,}/g, " ")
      .trim();
  }

  /** Quita instrucciones embebidas: banner/caja solo muestran el fallo. */
  function stripErrorInstructions(desc) {
    return String(desc || "")
      .replace(/\s*Abre el aviso para continuar\.?/gi, "")
      .replace(/\s*Abre Purga para continuar\.?/gi, "")
      .replace(/\s*Recuperación de unión requerida\.?/gi, "")
      .replace(/\s*Recuperación de material requerida\.?/gi, "")
      .replace(/\s*\(usa Iniciar o revisa fallas\)\.?/gi, "")
      .replace(/\s*\(revisar drive o bus\)\.?/gi, "")
      .replace(/\s{2,}/g, " ")
      .replace(/\s+\./g, ".")
      .trim();
  }

  function parseRecoveryError(code, mensaje) {
    const raw = String(mensaje || "").trim();
    const fallbackCode = String(code || "").trim();
    const items = [];
    // Estándar: "Error E043 Motion: …" (también acepta legado sin "Error ").
    const re = /(?:Error\s+)?([A-Z]?\d+)\s+([^:]+):\s*/gi;
    const starts = [];
    let m;
    while ((m = re.exec(raw)) !== null) {
      starts.push({
        index: m.index,
        end: m.index + m[0].length,
        code: m[1],
        module: String(m[2] || "").trim(),
      });
    }
    if (starts.length) {
      for (let i = 0; i < starts.length; i++) {
        const cur = starts[i];
        const next = starts[i + 1];
        let desc = raw.slice(cur.end, next ? next.index : raw.length).trim();
        desc = desc.replace(/\s·\s*$/, "").trim();
        items.push({ code: cur.code, module: cur.module, desc: desc });
      }
    } else if (raw) {
      items.push({ code: fallbackCode, module: "", desc: raw });
    }

    const primary = items[0] || {
      code: fallbackCode,
      module: "",
      desc: "La máquina se detuvo.",
    };
    if (fallbackCode && !primary.code) primary.code = fallbackCode;

    const uniqueDescs = [];
    items.forEach(function (it) {
      if (it.desc && uniqueDescs.indexOf(it.desc) < 0) uniqueDescs.push(it.desc);
    });

    const desc =
      uniqueDescs.length > 1
        ? uniqueDescs.join(" ")
        : uniqueDescs[0] || primary.desc || "La máquina se detuvo.";

    return {
      code: primary.code || fallbackCode,
      module: primary.module,
      header: formatErrorStandard(
        {
          code: primary.code || fallbackCode,
          module: primary.module,
          desc: desc,
          items: items.length ? items : null,
        },
        fallbackCode
      ),
      desc: desc,
      items: items,
    };
  }

  function formatErrorStandard(parsed, fallbackCode) {
    function one(it) {
      const code = String((it && it.code) || fallbackCode || "").trim() || "E000";
      const moduleName = String((it && it.module) || "").trim() || "Main";
      let desc = String((it && it.desc) || "").trim();
      if (!desc) desc = "La máquina se detuvo.";
      // Quitar prefijo legado "Error E090…" si el texto ya lo traía.
      desc = desc.replace(/^(?:Error\s+)?[A-Z]?\d+\s+[^:]+:\s*/i, "").trim() || desc;
      if (isLinkComError(code)) {
        desc = stripLinkComSolution(desc) || desc;
      }
      desc = stripErrorInstructions(desc) || desc;
      return code + " " + moduleName + ": " + desc;
    }
    const items = parsed && parsed.items && parsed.items.length ? parsed.items : null;
    if (items && items.length > 1) {
      const lines = [];
      items.forEach(function (it) {
        const line = one(it);
        if (lines.indexOf(line) >= 0) return;
        lines.push(line);
      });
      return lines.join(" · ");
    }
    return one({
      code: parsed && parsed.code,
      module: parsed && parsed.module,
      desc: parsed && parsed.desc,
    });
  }

  function openBannerTarget() {
    if (bannerOpenTarget === "conexion" || isLinkComError(lastBannerErrorCode)) {
      closeRecoveryModal();
      openParametros("conexion");
      return;
    }
    openRecoveryModal();
  }

  function setRecoveryPoll(on) {
    if (on) {
      if (recoveryPollTimer) return;
      pollRecoverySensors();
      recoveryPollTimer = setInterval(pollRecoverySensors, 500);
      return;
    }
    if (recoveryPollTimer) {
      clearInterval(recoveryPollTimer);
      recoveryPollTimer = null;
    }
  }

  async function pollRecoverySensors() {
    if (recoveryPollBusy) return;
    if (!els.recoveryModal || els.recoveryModal.hidden) return;
    recoveryPollBusy = true;
    try {
      const mode = String(els.recoveryModal.dataset.sensorMode || "");
      if (mode === "joint" || mode === "laser") {
        const res = await fetch("/api/motion/status");
        const data = await res.json();
        const enc = (data && data.encoder) || {};
        const busy =
          !!els.recoveryModal &&
          els.recoveryModal.dataset.recoveryBusy === "1";
        if (mode === "joint") {
          const onR = !!(enc.R && enc.R.joint === true);
          const onL = !!(enc.L && enc.L.joint === true);
          setRecoverySensorEl(els.recoverySensorR, onR);
          setRecoverySensorEl(els.recoverySensorL, onL);
          const cont = els.recoveryActions
            ? els.recoveryActions.querySelector(
                '[data-recovery="joint_continuar"]'
              )
            : null;
          if (cont) {
            const side = String(
              (els.recoveryModal && els.recoveryModal.dataset.jointSide) || ""
            ).toUpperCase();
            const blocked =
              side === "L" ? onL : side === "R" ? onR : onR || onL;
            cont.disabled = busy || blocked;
          }
        } else {
          const onR = !!(enc.R && enc.R.laser === true);
          const onL = !!(enc.L && enc.L.laser === true);
          setRecoverySensorEl(els.recoverySensorR, onR);
          setRecoverySensorEl(els.recoverySensorL, onL);
          const cont = els.recoveryActions
            ? els.recoveryActions.querySelector('[data-recovery="manguera"]')
            : null;
          if (cont) cont.disabled = busy || onR || onL;
        }
      }
    } catch (e) {
      /* ignore */
    } finally {
      recoveryPollBusy = false;
    }
  }

  function openRecoveryModal() {
    if (!els.recoveryModal) return;
    recoveryWantOpen = true;
    els.recoveryModal.hidden = false;
  }

  function closeRecoveryModal() {
    recoveryWantOpen = false;
    setRecoveryPoll(false);
    if (els.recoveryModal) els.recoveryModal.hidden = true;
  }

  function recoveryPrimaryButton(label, className, datasetKey) {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "btn " + className;
    btn.textContent = label;
    btn.setAttribute("data-recovery", datasetKey);
    return btn;
  }

  function recoverySkipAvailable(step, rutina) {
    if (step === "piezas" || step === "manguera" || step === "purga_error") return true;
    // Purga: solo en límites (antes de rutina o ya terminada). No mientras rutina=="purga".
    if (step === "purga" && String(rutina || "") !== "purga") return true;
    return false;
  }

  function syncRecoverySkipLink(step, rutina, jointKind) {
    if (!els.recoverySkipLink) return;
    const show = !jointKind && recoverySkipAvailable(step, rutina);
    els.recoverySkipLink.hidden = !show;
    const row = els.recoverySkipLink.closest(".recovery-step-row");
    if (row) row.hidden = !show;
  }

  function recoveryActionsGroup(kind, label) {
    const group = document.createElement("div");
    group.className = "recovery-actions__group recovery-actions__group--" + kind;
    if (label) {
      const title = document.createElement("p");
      title.className = "recovery-actions__label";
      title.textContent = label;
      group.appendChild(title);
    }
    return group;
  }

  function renderRecoveryActions(step, jointCtx, rutina) {
    if (!els.recoveryActions) return;
    els.recoveryActions.innerHTML = "";
    const joint = jointCtx || null;
    syncRecoverySkipLink(step, rutina, !!joint);
    if (joint) {
      const busy = !!(joint && joint.busy);
      // Tras Reset el Warning sigue el flujo normal piezas → manguera → purga.
      if (
        step === "piezas" ||
        step === "manguera" ||
        step === "purga" ||
        step === "purga_error"
      ) {
        /* cae al render de recuperación estándar más abajo */
      } else if (
        step === "await_piece" ||
        step === "post_cut" ||
        step === "await_home" ||
        step === "park"
      ) {
        const wait = recoveryPrimaryButton(
          step === "await_piece"
            ? "Terminando pieza del lado sano… espera"
            : step === "post_cut"
              ? "Terminando descarga… luego Reset"
              : step === "await_home"
                ? "Regresando a Home… espera"
                : "Preparando… espera",
          "btn-secondary",
          "wait"
        );
        wait.disabled = true;
        els.recoveryActions.appendChild(wait);
        return;
      } else if (step === "await_reset" || step === "segregate" || step === "operator") {
        const required = recoveryActionsGroup("required", "Siguiente");
        const resetBtn = recoveryPrimaryButton(
          "Reset",
          "btn-reset",
          "reset"
        );
        resetBtn.disabled = busy;
        required.appendChild(resetBtn);
        els.recoveryActions.appendChild(required);
        return;
      } else if (step === "classify") {
        const required = recoveryActionsGroup("required", "Clasificación");
        if (joint.segregateOnly) {
          const done = recoveryPrimaryButton(
            "Confirmar segregación",
            "btn-start",
            "joint_segregate_done"
          );
          done.disabled = busy;
          required.appendChild(done);
        } else {
          const good = recoveryPrimaryButton(
            "Pieza buena",
            "btn-start",
            "joint_classify_good"
          );
          const bad = recoveryPrimaryButton(
            "Pieza mala",
            "btn-secondary",
            "joint_classify_bad"
          );
          good.disabled = busy;
          bad.disabled = busy;
          required.appendChild(good);
          required.appendChild(bad);
        }
        els.recoveryActions.appendChild(required);
        return;
      } else if (step === "listo") {
        els.recoveryActions.appendChild(
          recoveryPrimaryButton(
            "Volver a la pantalla principal",
            "btn-start",
            "listo"
          )
        );
        return;
      } else {
        return;
      }
    }
    if (step === "error") {
      els.recoveryActions.appendChild(
        recoveryPrimaryButton("Reset", "btn-reset", "reset")
      );
      return;
    }
    if (step === "piezas") {
      const required = recoveryActionsGroup("required");
      required.appendChild(
        recoveryPrimaryButton("Confirmar", "btn-start", "piezas")
      );
      els.recoveryActions.appendChild(required);
      return;
    }
    if (step === "manguera") {
      const busy = !!(els.recoveryModal && els.recoveryModal.dataset.recoveryBusy === "1");
      const required = recoveryActionsGroup("required", "Siguiente");
      const helpers = recoveryActionsGroup("helpers", "Funciones Extras");
      const cont = recoveryPrimaryButton(
        "Confirmar",
        "btn-start",
        "manguera"
      );
      // Confirmar solo cuando el poll confirme láseres Apagado.
      cont.disabled = true;
      required.appendChild(cont);
      const safe = recoveryPrimaryButton(
        busy ? "Zona segura… espera" : "Zona segura",
        "btn-secondary recovery-helper-btn",
        "safe_zone"
      );
      safe.disabled = busy;
      helpers.appendChild(safe);
      els.recoveryActions.appendChild(required);
      els.recoveryActions.appendChild(helpers);
      return;
    }
    if (step === "purga") {
      if (recoverySkipAvailable(step, rutina)) {
        return;
      }
      const wait = recoveryPrimaryButton(
        "Proceso de limpieza en curso, espera…",
        "btn-secondary",
        "wait"
      );
      wait.disabled = true;
      els.recoveryActions.appendChild(wait);
      els.recoveryActions.appendChild(
        recoveryPrimaryButton("Cancelar", "btn-secondary", "purga_cancel")
      );
      return;
    }
    if (step === "purga_error") {
      const required = recoveryActionsGroup("required", "Siguiente");
      const meta = recoveryActionsGroup("meta");
      required.appendChild(
        recoveryPrimaryButton("Retry", "btn-start", "purga_retry")
      );
      meta.appendChild(
        recoveryPrimaryButton("Cancelar", "btn-secondary", "purga_cancel")
      );
      els.recoveryActions.appendChild(required);
      els.recoveryActions.appendChild(meta);
      return;
    }
    if (step === "listo") {
      els.recoveryActions.appendChild(
        recoveryPrimaryButton(
          "Volver a la pantalla principal",
          "btn-start",
          "listo"
        )
      );
    }
  }

  function syncRecovery(state) {
    const rec = state.recovery || {};
    const active = !!rec.active;
    const step = String(rec.step || "");
    const jointKind = rec.kind === "joint" || rec.kind === "material";
    const enError = state.estado === "error" || !!state.ciclo_error;
    const code = rec.code || state.ciclo_error || "";
    const linkCom = isLinkComError(code);
    lastBannerErrorCode = String(code || "").trim().toUpperCase();
    bannerOpenTarget = linkCom ? "conexion" : "recovery";
    const showBtn = active || enError || !!state.post_error_purga;

    if (els.btnOpenRecovery) {
      els.btnOpenRecovery.hidden = !showBtn;
      els.btnOpenRecovery.disabled = !showBtn;
      if (linkCom) {
        els.btnOpenRecovery.textContent = "Abrir";
      } else if (jointKind) {
        if (step === "listo") {
          els.btnOpenRecovery.textContent = "Ver Warning";
        } else {
          els.btnOpenRecovery.textContent = "Abrir Warning";
        }
      } else if (step === "listo") {
        els.btnOpenRecovery.textContent = "Ver recuperación";
      } else if (enError) {
        els.btnOpenRecovery.textContent = "Abrir";
      } else {
        els.btnOpenRecovery.textContent = "Continuar recuperación";
      }
    }

    // COM/enlace: no popup de recuperación; Abrir lleva a Conexión.
    if (linkCom && enError && !jointKind) {
      setRecoveryPoll(false);
      recoveryWantOpen = false;
      if (els.recoveryModal && !els.recoveryModal.hidden) {
        closeRecoveryModal();
      }
      if (!active) {
        recoveryViewKey = "";
        recoveryAutoKey = "";
        return;
      }
    }

    if (!active && !enError) {
      setRecoveryPoll(false);
      if (els.recoverySkipLink) els.recoverySkipLink.hidden = true;
      if (els.recoveryModal && !els.recoveryModal.hidden) {
        closeRecoveryModal();
      }
      recoveryViewKey = "";
      recoveryAutoKey = "";
      if (bannerOpenTarget !== "conexion") {
        bannerOpenTarget = "recovery";
        lastBannerErrorCode = "";
      }
      return;
    }

    const viewStep = step || (enError ? "error" : "");
    // Solo auto-abrir al surgir un error/paso nuevo. Si el operador cierra (X),
    // no volver a forzar el popup en cada poll: puede usar «Abrir».
    const autoKey =
      (active ? "a" : "e") +
      "|" +
      String(code || "") +
      "|" +
      (jointKind ? "j" : "x") +
      "|" +
      String(viewStep || "");
    if (autoKey !== recoveryAutoKey) {
      recoveryAutoKey = autoKey;
      if (active && step && step !== "listo" && !linkCom) {
        recoveryWantOpen = true;
      }
    }

    const rutina = String(state.rutina || "");
    const guideKey = String(rec.guide || "").slice(0, 80);
    const key =
      viewStep +
      "|" +
      code +
      "|" +
      (jointKind ? "j" : "e") +
      "|" +
      String(rec.side || "") +
      "|" +
      String(rec.origin || "") +
      "|" +
      (recoveryWantOpen ? "1" : "0") +
      "|" +
      (rec.busy ? "b" : "") +
      "|" +
      (rec.lm_at_park ? "p" : "h") +
      "|" +
      rutina +
      "|" +
      guideKey;
    const modalOpen = els.recoveryModal && !els.recoveryModal.hidden;
    if (!modalOpen && !recoveryWantOpen) {
      setRecoveryPoll(false);
      return;
    }
    if (!modalOpen && recoveryWantOpen) {
      openRecoveryModal();
    }

    if (els.recoveryModal) {
      els.recoveryModal.dataset.jointMode = jointKind ? "1" : "";
    }

    if (els.recoveryModalTitle) {
      if (jointKind) {
        const side = String(rec.side || "");
        const sideLabel =
          side === "RL" ? "R y L" : side === "L" ? "L" : side === "R" ? "R" : "";
        const warnTitle =
          rec.kind === "material" ? "Warning Material" : "Warning Joint";
        els.recoveryModalTitle.textContent =
          viewStep === "listo"
            ? "Warning listo"
            : warnTitle + (sideLabel ? " · " + sideLabel : "");
      } else {
        els.recoveryModalTitle.textContent =
          viewStep === "listo" ? "Listo" : "Recuperación";
      }
    }

    const jointOrigin = String(rec.origin || "");
    const errorFixed = String(rec.mensaje || "").trim();
    const stepGuide = String(rec.guide || "").trim();
    // Caja = solo error. Guía = apartado INSTRUCCIONES. Sin "Paso X de Y".
    if (els.recoveryStepLabel) {
      els.recoveryStepLabel.hidden = true;
      els.recoveryStepLabel.textContent = "";
    }
    if (els.recoveryLead) {
      const showHandsWarn = !jointKind && viewStep === "purga";
      els.recoveryLead.hidden = !showHandsWarn;
      els.recoveryLead.textContent = showHandsWarn ? "NO INTRODUCIR MANOS" : "";
      els.recoveryLead.classList.toggle("is-hands-warn", showHandsWarn);
    }
    if (els.recoveryText) {
      // Apartado de instrucciones: solo recovery.guide (nunca mezclado con el error).
      if (!stepGuide || (!jointKind && viewStep === "purga")) {
        els.recoveryText.hidden = true;
        els.recoveryText.textContent = "";
        els.recoveryText.classList.remove("is-hands-warn");
      } else {
        els.recoveryText.hidden = false;
        els.recoveryText.innerHTML =
          '<span class="recovery-text__label">INSTRUCCIONES:</span> ' + stepGuide;
        els.recoveryText.classList.toggle(
          "is-hands-warn",
          !!(jointKind && viewStep === "park")
        );
      }
    }
    if (els.recoveryModal) {
      const panel = els.recoveryModal.querySelector(".modal--recovery");
      if (panel) panel.classList.toggle("is-hands-warn", viewStep === "purga");
    }

    // Caja de error: siempre el fallo original fijo (no las instrucciones del paso).
    const showErrorBox =
      (!!active && !!code) ||
      jointKind ||
      viewStep === "error" ||
      viewStep === "purga_error";
    const parsed = parseRecoveryError(code, errorFixed);
    if (els.recoveryErrorBox) {
      els.recoveryErrorBox.hidden = !showErrorBox;
    }
    if (showErrorBox) {
      const standard = formatErrorStandard(parsed, code);
      if (els.recoveryCode) {
        els.recoveryCode.textContent = standard;
      }
      if (els.recoveryErrorDesc) {
        els.recoveryErrorDesc.textContent = "";
        els.recoveryErrorDesc.hidden = true;
      }
    } else if (els.recoveryErrorDesc) {
      els.recoveryErrorDesc.hidden = false;
    }

    const nameR = document.getElementById("recoverySensorNameR");
    const nameL = document.getElementById("recoverySensorNameL");
    if (jointKind) {
      if (nameR) nameR.textContent = "Joint Right";
      if (nameL) nameL.textContent = "Joint Left";
    } else {
      if (nameR) nameR.textContent = "Sensor láser derecho";
      if (nameL) nameL.textContent = "Sensor láser izquierdo";
    }

    const showSensors =
      (jointKind &&
        (viewStep === "segregate" ||
          viewStep === "await_reset" ||
          viewStep === "classify" ||
          viewStep === "operator" ||
          viewStep === "await_piece" ||
          viewStep === "piezas" ||
          viewStep === "manguera")) ||
      (!jointKind && (viewStep === "manguera" || viewStep === "purga_error"));
    if (els.recoveryJointLegend) {
      // Las descripciones breves van en «Funciones Extras» de las acciones.
      els.recoveryJointLegend.hidden = true;
    }
    if (els.recoveryModal) {
      els.recoveryModal.dataset.jointMode = jointKind ? "1" : "";
      els.recoveryModal.dataset.sensorMode = jointKind ? "joint" : "laser";
      els.recoveryModal.dataset.jointSide = jointKind
        ? String(rec.side || "")
        : "";
      els.recoveryModal.dataset.lmAtPark = rec.lm_at_park ? "1" : "";
      els.recoveryModal.dataset.recoveryBusy = rec.busy ? "1" : "";
    }
    if (els.recoverySensors) {
      els.recoverySensors.hidden = !showSensors;
    }
    setRecoveryPoll(showSensors && !!modalOpen);

    if (key !== recoveryViewKey) {
      recoveryViewKey = key;
      renderRecoveryActions(
        viewStep,
        jointKind
          ? {
              origin: jointOrigin,
              side: rec.side,
              busy: !!rec.busy,
              segregateOnly: !!rec.segregate_only,
              healthySide: String(rec.healthy_side || ""),
            }
          : null,
        rutina
      );
      if (showSensors) pollRecoverySensors();
    } else {
      syncRecoverySkipLink(viewStep, rutina, jointKind);
      if (showSensors) pollRecoverySensors();
    }
  }

  function syncProcessControls(state) {
    const panel = getProcessPanel();
    if (!panel) return;

    const estado = state.estado || "idle";
    const enCurso = !!state.ciclo_en_curso;
    const enError = estado === "error" || !!state.ciclo_error;
    const postErrorPurga = !!state.post_error_purga;
    const running = estado === "running";
    const paused = estado === "paused";
    const rutina = !!state.rutina;
    const purga = state.rutina === "purga";
    const ch = state.checklist || {};
    const checklistReady = !!ch.ok || !!state.checklist_bypass;

    const sidesEl = document.querySelector("[data-cycle-param='feed_sides']");
    const sidesRaw =
      state.feed_sides ||
      (sidesEl ? String(sidesEl.value || "").trim() : "") ||
      "";
    const sides = String(sidesRaw).trim().toUpperCase();
    const piezas = parseInt(String(state.cantidad), 10);
    const ambos =
      sides === "LR" || sides === "RL" || sides === "BOTH" || sides === "AMBOS";
    const cantidadImpar =
      ambos && (!Number.isFinite(piezas) || piezas < 2 || piezas % 2 !== 0);
    const pfCom = !!(state.prefeeder && state.prefeeder.com);

    const start = panel.querySelector('[data-action="start"]');
    const resume = panel.querySelector('[data-action="resume"]');
    const pause = panel.querySelector('[data-action="pause"]');
    const stop = panel.querySelector('[data-action="stop"]');
    const reset = panel.querySelector('[data-action="reset"]');

    panel.querySelectorAll("[data-action]").forEach(function (btn) {
      btn.classList.remove("is-active");
      btn.removeAttribute("aria-pressed");
    });

    const loteActivo = !!state.lote_activo || (!!enCurso && (running || paused || enError));
    const modeloOk = hasModeloSeleccionado(state);
    if (start) {
      start.disabled =
        !checklistReady ||
        !modeloOk ||
        purga ||
        running ||
        (paused && enCurso) ||
        loteActivo ||
        cantidadImpar ||
        enError ||
        postErrorPurga;
      if (!checklistReady) {
        start.title = "Completa la revisión de arranque.";
      } else if (!modeloOk) {
        start.title = "Selecciona un modelo antes de Iniciar.";
      } else if (purga) {
        start.title = "Hay una purga en curso.";
      } else if (enError) {
        start.title = "Abre el error y sigue la recuperación.";
      } else if (postErrorPurga) {
        start.title = "Completa la recuperación antes de Iniciar.";
      } else if (cantidadImpar) {
        start.title =
          "Con ambos lados la cantidad tiene que ser par y de al menos 2 piezas.";
      } else if (loteActivo) {
        start.title = "Hay un lote en curso. Reanuda o pulsa Detener.";
      } else if (running) {
        start.title = "El ciclo ya está en marcha.";
      } else {
        start.title = "";
      }
    }
    if (resume) {
      resume.disabled =
        !(enCurso || !!state.lote_activo) ||
        !paused ||
        purga ||
        enError ||
        postErrorPurga;
      resume.title = resume.disabled
        ? enError
          ? "Abre el error y sigue la recuperación."
          : postErrorPurga
            ? "Completa la recuperación antes de Reanudar."
            : paused && !(enCurso || state.lote_activo)
              ? "No hay un ciclo pausado para reanudar."
              : ""
        : "Continúa el ciclo desde el inicio de la pieza actual.";
    }
    if (pause) {
      pause.disabled = (!running && !(enError && enCurso)) || rutina;
      pause.title = pause.disabled ? "Solo disponible con el ciclo en marcha." : "";
    }
    if (stop) {
      stop.disabled = !(
        running ||
        paused ||
        enCurso ||
        rutina ||
        enError ||
        postErrorPurga ||
        pfCom
      );
      stop.title = stop.disabled
        ? ""
        : "Detiene el ciclo, la rutina o sale de la ruta post-error (cancela el lote).";
    }
    if (reset) {
      reset.disabled = !enError;
      reset.title = enError
        ? "Abre el error y pulsa Reset ahí, o pulsa Reset para soltar holder y pieza."
        : "Solo aplica cuando hay un error activo.";
    }

    if (enError && reset && !reset.disabled) {
      reset.classList.add("is-active");
      reset.setAttribute("aria-pressed", "true");
    } else if (paused && enCurso && resume && !resume.disabled) {
      resume.classList.add("is-active");
      resume.setAttribute("aria-pressed", "true");
    }

  }

  function flashButton(btn) {
    btn.classList.remove("btn--flash");
    void btn.offsetWidth;
    btn.classList.add("btn--flash");

    const latched =
      btn.classList.contains("is-active") ||
      btn.getAttribute("aria-pressed") === "true";

    if (!latched) {
      btn.classList.add("is-vivid");
      window.setTimeout(function () {
        if (
          !btn.classList.contains("is-active") &&
          btn.getAttribute("aria-pressed") !== "true"
        ) {
          btn.classList.remove("is-vivid");
        }
      }, 450);
    }

    btn.addEventListener(
      "animationend",
      function onEnd() {
        btn.classList.remove("btn--flash");
        btn.removeEventListener("animationend", onEnd);
      },
      { once: true }
    );
  }

  function applyServerEvents(state, estado) {
    if (!Array.isArray(state.eventos) || clearingLog) return;
    const rev = Number(state.eventos_rev);
    const known = Number.isFinite(rev);
    if (known && rev < eventsRev) return;
    if (known && rev === eventsRev && !state.eventos.length) return;
    if (!known && !state.eventos.length) return;
    if (known) eventsRev = rev;
    pendingEventos = state.eventos;
    if (eventsEnabled) renderEvents(pendingEventos);
    else if (estado === "error" && state.eventos[0]) {
      setEventsPanelVisible(true);
      if (els.toggleEventsPanel) els.toggleEventsPanel.checked = true;
      renderEvents(pendingEventos);
    }
  }

  async function clearEventsLog() {
    if (!els.btnClearEvents || clearingLog) return;
    clearingLog = true;
    els.btnClearEvents.disabled = true;
    pendingEventos = [];
    if (els.eventsLog) els.eventsLog.value = "";
    try {
      const res = await fetch("/api/events/clear", { method: "POST" });
      const data = await res.json().catch(function () { return {}; });
      if (!res.ok || !data.ok) {
        throw new Error(data.error || "No se pudo borrar el registro.");
      }
      if (Number.isFinite(Number(data.eventos_rev))) eventsRev = Number(data.eventos_rev);
      pendingEventos = [];
      if (els.eventsLog) els.eventsLog.value = "";
    } catch (err) {
      clearingLog = false;
      pushLocalEvent(err.message || "No se pudo borrar el registro.", "error");
    } finally {
      clearingLog = false;
      if (els.btnClearEvents) els.btnClearEvents.disabled = false;
    }
  }

  function formatEventLine(ev) {
    const mark =
      ev.tipo === "error"
        ? "ERROR  "
        : ev.tipo === "warn"
          ? "AVISO  "
          : ev.tipo === "prod"
            ? "PROD   "
            : "";
    const text = mark + (ev.texto || "");
    return ev.hora ? ev.hora + "  " + text : text;
  }

  function renderEvents(eventos) {
    if (!eventsEnabled || !els.eventsLog) return;
    const next = (eventos || []).slice().reverse().map(formatEventLine).join("\n");
    const box = els.eventsLog;
    if (box.value === next) return;
    const prev = box.value;
    const grewAtEnd = next.startsWith(prev);
    const stick = box.scrollHeight - box.scrollTop - box.clientHeight < 24;
    const start = box.selectionStart;
    const end = box.selectionEnd;
    const selecting = document.activeElement === box && start !== end;
    const scrollTop = box.scrollTop;
    box.value = next;
    if (selecting && grewAtEnd) {
      box.setSelectionRange(start, end);
      box.scrollTop = scrollTop;
    } else if (stick) {
      box.scrollTop = box.scrollHeight;
    } else {
      box.scrollTop = scrollTop;
    }
  }

  function pushLocalEvent(texto, tipo) {
    const entry = { texto: texto, tipo: tipo || "info", hora: formatTime(new Date()) };
    pendingEventos.unshift(entry);
    pendingEventos = pendingEventos.slice(0, 300);
    renderEvents(pendingEventos);
  }

  let modelComboFilter = "";
  let modelComboActive = -1;
  let modelComboOpen = false;

  function syncModelSelect(modeloId) {
    if (!els.selectModelo) return;
    if (document.activeElement === els.selectModelo && modelComboOpen) return;
    syncingModelSelect = true;
    els.selectModelo.value = modeloId || "";
    syncingModelSelect = false;
    syncModeloDesc(modeloId);
  }

  function populateModelSelect(modelos) {
    modelosCatalog = modelos || [];
    const current = els.selectModelo ? String(els.selectModelo.value || "").trim() : "";
    if (current) syncModelSelect(current);
    else syncModeloDesc("");
    renderModelComboList(modelComboFilter, false);
  }

  function filteredModelos(query) {
    const q = String(query || "").trim().toLowerCase();
    if (!q) return modelosCatalog.slice();
    return modelosCatalog.filter(function (item) {
      const modelo = String(item.modelo || "").toLowerCase();
      const desc = String(item.desc || "").toLowerCase();
      return modelo.indexOf(q) >= 0 || desc.indexOf(q) >= 0;
    });
  }

  function setModelComboOpen(open) {
    modelComboOpen = !!open;
    if (els.selectModelo) els.selectModelo.setAttribute("aria-expanded", modelComboOpen ? "true" : "false");
    if (els.modelComboList) els.modelComboList.hidden = !modelComboOpen;
    if (!modelComboOpen) modelComboActive = -1;
  }

  function renderModelComboList(query, open) {
    if (!els.modelComboList) return;
    modelComboFilter = String(query || "");
    const items = filteredModelos(modelComboFilter);
    els.modelComboList.innerHTML = "";
    if (!items.length) {
      const empty = document.createElement("li");
      empty.className = "model-combo__empty";
      empty.textContent = "Sin coincidencias";
      els.modelComboList.appendChild(empty);
      modelComboActive = -1;
    } else {
      if (modelComboActive >= items.length) modelComboActive = items.length - 1;
      items.forEach(function (item, idx) {
        const li = document.createElement("li");
        li.setAttribute("role", "option");
        li.id = "modelComboOpt-" + idx;
        const btn = document.createElement("button");
        btn.type = "button";
        btn.className = "model-combo__option" + (idx === modelComboActive ? " is-active" : "");
        btn.textContent = item.modelo;
        btn.title = item.desc || item.modelo;
        btn.dataset.modelo = item.modelo;
        btn.addEventListener("mousedown", function (ev) {
          ev.preventDefault();
          pickModeloFromCombo(item.modelo);
        });
        li.appendChild(btn);
        els.modelComboList.appendChild(li);
      });
      if (els.selectModelo && modelComboActive >= 0) {
        els.selectModelo.setAttribute("aria-activedescendant", "modelComboOpt-" + modelComboActive);
      }
    }
    if (open) setModelComboOpen(true);
  }

  function clearModeloSelection(persist) {
    setModelComboOpen(false);
    syncingModelSelect = true;
    if (els.selectModelo) els.selectModelo.value = "";
    syncingModelSelect = false;
    syncModeloDesc("");
    if (persist === false) return;
    selectModelo("", true);
  }

  function pickModeloFromCombo(modeloId) {
    const mid = String(modeloId || "").trim();
    if (!mid || !findModeloInCatalog(mid)) {
      clearModeloSelection(true);
      return;
    }
    setModelComboOpen(false);
    syncingModelSelect = true;
    if (els.selectModelo) els.selectModelo.value = mid;
    syncingModelSelect = false;
    syncModeloDesc(mid);
    selectModelo(mid, true);
  }

  function hasModeloSeleccionado(state) {
    const mid = String(
      (state && state.numero_modelo) ||
        (els.selectModelo && els.selectModelo.value) ||
        ""
    ).trim();
    return !!findModeloInCatalog(mid);
  }

  async function loadModelos() {
    try {
      const res = await fetch("/api/modelos");
      const data = await res.json();
      if (!data.ok) return;
      populateModelSelect(data.modelos);
    } catch (err) {
      pushLocalEvent("No se pudo cargar el catálogo de modelos.", "error");
    }
  }

  async function selectModelo(modeloId, persist) {
    if (persist === undefined) persist = true;
    try {
      const res = await fetch("/api/modelo", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ modelo: modeloId || "" }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se pudo seleccionar el modelo.", "error");
        return;
      }
      if (persist) {
        try {
          const saved = (data.state && data.state.numero_modelo) || modeloId || "";
          if (saved) localStorage.setItem(STORAGE_MODEL, saved);
          else localStorage.removeItem(STORAGE_MODEL);
        } catch (e) {
          /* ignore */
        }
      }
      if (data.state) applyState(data.state);
    } catch (err) {
      pushLocalEvent("Error al guardar el modelo.", "error");
    }
  }

  function setByPath(obj, path, value) {
    const parts = path.split(".");
    let cur = obj;
    for (let i = 0; i < parts.length - 1; i++) {
      const key = parts[i];
      if (!cur[key] || typeof cur[key] !== "object") cur[key] = {};
      cur = cur[key];
    }
    cur[parts[parts.length - 1]] = value;
  }

  function getByPath(obj, path) {
    const parts = path.split(".");
    let cur = obj;
    for (let i = 0; i < parts.length; i++) {
      if (cur == null) return undefined;
      cur = cur[parts[i]];
    }
    return cur;
  }

  function motionRefreshCalScaleHint(data) {
    const hint = document.getElementById("lmCalScaleHint");
    if (!hint) return;
    const steps = Number((data && data.steps_per_mm) || motionStepsPerMm);
    if (!Number.isFinite(steps) || steps <= 0) {
      hint.textContent = "Escala: — PUU/mm";
      return;
    }
    hint.textContent = "Escala: " + steps.toFixed(6) + " PUU/mm";
  }

  function motionApplyScale(data) {
    if (!data) return;
    const steps = Number(data.steps_per_mm);
    if (Number.isFinite(steps) && steps > 0) motionStepsPerMm = steps;
    if (data.puu_sign === 1 || data.puu_sign === -1) motionPuuSign = data.puu_sign;
    const res = Number(data.mm_res);
    if (Number.isFinite(res) && res > 0) motionMmRes = res;
    const shaft = Number(data.feed_shaft_inc_per_rev);
    if (Number.isFinite(shaft) && shaft > 0) motionFeedShaftIncPerRev = shaft;
    motionRefreshCalScaleHint(data);
    motionRefreshPuuReadout();
    motionRefreshCalPuuReadout();
  }

  function feedRpmToPps(rpm) {
    return Math.round(Math.round(Number(rpm)) * motionFeedShaftIncPerRev / 60);
  }

  function feedPpsToRpmText(pps) {
    const rpm = Math.round((Number(pps) * 60) / (motionFeedShaftIncPerRev || 1));
    return String(Number.isFinite(rpm) ? rpm : 0);
  }

  function motionSnapMm(mm) {
    const res = motionMmRes || 0.1;
    const n = Number(mm);
    if (!Number.isFinite(n)) return NaN;
    return Math.round(n / res) * res;
  }

  function motionMmToPuu(mm) {
    const snapped = motionSnapMm(mm);
    if (!Number.isFinite(snapped)) return 0;
    return Math.round(motionPuuSign * snapped * motionStepsPerMm);
  }

  function motionRefreshPuuReadout() {
    const input = document.getElementById("lmMoveMm");
    const out = document.getElementById("lmMovePuu");
    if (!input || !out) return;
    const raw = input.value.trim();
    const mm = Number(raw);
    if (raw === "" || !Number.isFinite(mm)) {
      out.textContent = "—";
      return;
    }
    out.textContent = String(motionMmToPuu(mm));
  }

  function motionRefreshCalPuuReadout() {
    const input = document.getElementById("lmCalMm");
    const out = document.getElementById("lmCalPuu");
    if (!input || !out) return;
    const raw = input.value.trim();
    const mm = Number(raw);
    if (raw === "" || !Number.isFinite(mm)) {
      out.textContent = "PUU —";
      return;
    }
    out.textContent = "PUU " + String(motionMmToPuu(mm));
  }

  function motionFormatMm(puu) {
    const mm = Math.abs(Number(puu)) / (motionStepsPerMm || 1);
    if (!Number.isFinite(mm)) return "";
    const snapped = motionSnapMm(mm);
    if (!Number.isFinite(snapped)) return "";
    // Siempre un decimal: 500 → "500.0" (evita stepMismatch del input).
    return snapped.toFixed(1);
  }

  function motionFormFromTune(tune) {
    if (!els.motionTuneForm || !tune) return;
    if (motionTuneDirty) return;
    const active = document.activeElement;
    if (active && els.motionTuneForm.contains(active)) return;
    els.motionTuneForm.querySelectorAll("[data-motion-path]").forEach(function (input) {
      const path = input.getAttribute("data-motion-path");
      const val = getByPath(tune, path);
      if (val == null) return;
      if (input.type === "checkbox") {
        input.checked = !!val;
        return;
      }
      const unit = input.getAttribute("data-motion-unit");
      if (unit === "mm") {
        input.value = motionFormatMm(val);
        return;
      }
      if (unit === "feed_rpm") {
        input.value = feedPpsToRpmText(val);
        return;
      }
      input.value = String(val);
    });
    motionRefreshPuuReadout();
  }

  function motionTuneFromForm() {
    const tune = {
      feeder: { R: {}, L: {} },
      lmguide: {},
    };
    if (!els.motionTuneForm) return tune;
    els.motionTuneForm.querySelectorAll("[data-motion-path]").forEach(function (input) {
      const path = input.getAttribute("data-motion-path");
      if (input.type === "checkbox") {
        setByPath(tune, path, input.checked);
        return;
      }
      const raw = input.value.trim();
      if (raw === "") return;
      let num = Number(raw);
      if (!Number.isFinite(num)) {
        setByPath(tune, path, raw);
        return;
      }
      const unit = input.getAttribute("data-motion-unit");
      if (unit === "mm") {
        num = motionMmToPuu(motionSnapMm(num));
      }
      if (unit === "feed_rpm") num = feedRpmToPps(num);
      setByPath(tune, path, num);
    });
    return tune;
  }

  async function loadMotionTune() {
    try {
      const res = await fetch("/api/motion/tune");
      const data = await res.json();
      if (!data.ok) return;
      motionApplyScale(data);
      if (data.tune) motionFormFromTune(data.tune);
    } catch (err) {
      pushLocalEvent("No se pudo cargar ajustes Motion.", "error");
    }
  }

  function setMotionHint(text) {
    if (!els.motionEsp32Hint) return;
    els.motionEsp32Hint.hidden = !text;
    els.motionEsp32Hint.textContent = text || "";
  }

  function latchLmServo(on) {
    const btn = document.getElementById("lmServoToggle");
    if (!btn) return;
    btn.setAttribute("aria-pressed", on ? "true" : "false");
    btn.classList.toggle("is-active", !!on);
    setToggleLabel(btn, on);
  }

  function latchFeederServo(on) {
    const btn = document.getElementById("feederServoToggle");
    if (!btn) return;
    btn.setAttribute("aria-pressed", on ? "true" : "false");
    btn.classList.toggle("is-active", !!on);
    setToggleLabel(btn, on);
  }

  async function sendMotionOp(op, extra) {
    const urgent = op === "lmguide_stop";
    if (motionCommandInFlight && !urgent) {
      pushLocalEvent(
        "Motion está ejecutando otra orden. Usa Paro de emergencia o espera a que termine.",
        "error"
      );
      return null;
    }
    if (!urgent) motionCommandInFlight = true;
    const body = Object.assign({ op: op, tune: motionTuneFromForm() }, extra || {});
    try {
      const res = await fetch("/api/motion/cmd", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se armó la orden Motion.", "error");
        return null;
      }
      if (!data.sent) {
        if (op !== "set_tune") {
          pushLocalEvent(data.reason || "No se envió la orden Motion.", "error");
          return null;
        }
        return data;
      }
      if (data.command_ok === false) {
        pushLocalEvent(data.error || "El MCU Motion rechazó la orden: " + op, "error");
        return null;
      }
      motionApplyScale(data);
      if (data.tune) motionFormFromTune(data.tune);
      if (data.sent) {
        pushLocalEvent("Motion: orden enviada.");
        setMotionHint("Listo.");
      } else {
        const reason = data.reason || "No se envió al puerto.";
        pushLocalEvent(reason, "error");
        setMotionHint(reason);
      }
      return data;
    } catch (err) {
      pushLocalEvent("Error al enviar la orden Motion.", "error");
      return null;
    } finally {
      if (!urgent) motionCommandInFlight = false;
    }
  }

  function clampFeedSecondaryInForm() {
    if (!els.motionTuneForm) return;
    ["R", "L"].forEach(function (side) {
      const pri = els.motionTuneForm.querySelector(
        '[data-motion-path="feeder.' + side + '.profile_velocity_pps"]'
      );
      const sec = els.motionTuneForm.querySelector(
        '[data-motion-path="feeder.' + side + '.secondary_velocity_pps"]'
      );
      if (!pri || !sec) return;
      const p = Number(pri.value);
      const s = Number(sec.value);
      if (!Number.isFinite(p) || !Number.isFinite(s)) return;
      if (s > p) sec.value = String(p);
      if (p > 3000) pri.value = "3000";
      if (Number(sec.value) > 3000) sec.value = "3000";
      if (p < 0) pri.value = "0";
      if (Number(sec.value) < 0) sec.value = "0";
    });
    ["R", "L"].forEach(function (side) {
      const sw = els.motionTuneForm.querySelector(
        '[data-motion-path="feeder.' + side + '.speed_switch_ms"]'
      );
      if (!sw) return;
      let ms = Number(sw.value);
      if (!Number.isFinite(ms) || ms < 0) sw.value = "0";
      else if (ms > 15000) sw.value = "15000";
    });
  }

  async function saveMotionTune(ev) {
    if (ev && ev.preventDefault) ev.preventDefault();
    const lmInput = document.getElementById("lmMoveMm");
    if (lmInput && lmInput.value.trim() !== "") {
      const snapped = motionSnapMm(lmInput.value.trim());
      if (Number.isFinite(snapped)) {
        lmInput.value = snapped.toFixed(1);
        motionRefreshPuuReadout();
      }
    }
    clampFeedSecondaryInForm();
    const tune = motionTuneFromForm();
    try {
      const res = await fetch("/api/motion/tune", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ tune: tune }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se guardaron ajustes Motion.", "error");
        return;
      }
      motionTuneDirty = false;
      motionApplyScale(data);
      if (data.tune) motionFormFromTune(data.tune);
      pushLocalEvent("Ajustes Motion guardados.");
      const pushed = await sendMotionOp("set_tune", { tune: data.tune });
      if (pushed && pushed.sent === false) {
        setMotionHint(pushed.reason || "Guardado en disco; sin puerto Motion para enviar al MCU.");
      }
    } catch (err) {
      pushLocalEvent("Sin conexión al guardar ajustes Motion.", "error");
    }
  }

  async function runMotionButton(btn) {
    const op = btn.getAttribute("data-motion-op");
    if (!op) return;
    flashButton(btn);
    const extra = {};
    if (op === "lmguide_servo") {
      const on = btn.getAttribute("aria-pressed") !== "true";
      extra.on = on;
      const data = await sendMotionOp(op, extra);
      if (data && data.command_ok === true) latchLmServo(on);
      return;
    }
    if (op === "feeder_servo") {
      const on = btn.getAttribute("aria-pressed") !== "true";
      extra.on = on;
      const data = await sendMotionOp(op, extra);
      if (data && data.command_ok === true) latchFeederServo(on);
      return;
    }
    if (op === "lmguide_init") {
      const data = await sendMotionOp(op);
      if (data && data.command_ok === true) latchLmServo(true);
      return;
    }
    if (op === "motion_hw_init") {
      if (motionCommandInFlight) {
        await sendMotionOp(op);
        return;
      }
      applyActuatorsStatus({ state: "initializing", stage: "feeder" });
      const data = await sendMotionOp(op);
      if (data && data.command_ok === true) {
        latchLmServo(true);
        latchFeederServo(true);
        applyActuatorsStatus({ state: "online", stage: "ready" });
      } else {
        applyActuatorsStatus({ state: "offline", stage: "" });
      }
      fetchState();
      return;
    }
    if (op === "lmguide_move" || op === "lmguide_move_home") {
      const forCutAttr = btn.getAttribute("data-for-cut");
      if (forCutAttr != null) {
        extra.for_cut = forCutAttr === "1" || forCutAttr === "true";
      }
    }
    if (op === "lmguide_move") {
      const mmInputId = btn.getAttribute("data-mm-input") || "lmMoveMm";
      const input = document.getElementById(mmInputId);
      const raw = (input && input.value) || "";
      const mm = motionSnapMm(String(raw).trim());
      if (!Number.isFinite(mm)) {
        pushLocalEvent("Escribe la posición del LM Guide en mm.", "error");
        return;
      }
      extra.mm = mm;
      if (input) {
        input.value = mm.toFixed(1);
        if (mmInputId === "lmCalMm") motionRefreshCalPuuReadout();
        else motionRefreshPuuReadout();
      }
    }
    if (op === "encoder_reset") {
      extra.side = btn.getAttribute("data-motion-side") || "R";
      const data = await sendMotionOp(op, extra);
      if (data) pollEncoder();
      return;
    }
    if (op === "feeder_test") {
      extra.side = btn.getAttribute("data-motion-side") || "R";
      const data = await sendMotionOp(op, extra);
      if (data) pollEncoder();
      return;
    }
    if (op === "lmguide_jog") {
      const mm = Math.abs(Number(btn.getAttribute("data-jog-mm") || 0));
      if (!Number.isFinite(mm) || mm === 0) {
        pushLocalEvent("Paso de jog no válido.", "error");
        return;
      }
      extra.jog_mm = mm;
      extra.dir = Number(btn.getAttribute("data-motion-dir") || 1);
    }
    await sendMotionOp(op, extra);
  }

  function motionShowSensorLed(led, side, name, on) {
    if (!led) return;
    const state = led.querySelector(".motion-laser__state");
    const label = on ? "Detectado" : "Sin detección";
    led.classList.toggle("is-on", on);
    if (state && state.textContent !== label) state.textContent = label;
    led.setAttribute("aria-label", name + " lado " + side + (on ? " detectado" : " sin detección"));
  }

  function motionShowEncoder(encoder) {
    ["R", "L"].forEach(function (side) {
      const el = document.getElementById(side === "R" ? "encReadR" : "encReadL");
      const laserLed = document.getElementById(side === "R" ? "laserLedR" : "laserLedL");
      const jointLed = document.getElementById(side === "R" ? "jointLedR" : "jointLedL");
      const materialLed = document.getElementById(
        side === "R" ? "materialLedR" : "materialLedL"
      );
      const sideData = encoder && encoder[side];
      if (el) {
        const counts = sideData && sideData.counts;
        // Solo counts enteros del MCU. No mostrar mm: decimales = ruta con fórmula.
        el.textContent =
          counts != null && Number.isFinite(Number(counts))
            ? String(Math.round(Number(counts)))
            : "—";
      }
      motionShowSensorLed(laserLed, side, "Láser", !!(sideData && sideData.laser === true));
      motionShowSensorLed(jointLed, side, "Joint", !!(sideData && sideData.joint === true));
      // Material: ON = presente. Etiqueta distinta a Joint/Láser.
      if (materialLed) {
        const matOn = !!(sideData && sideData.material === true);
        const stateEl = materialLed.querySelector(".motion-laser__state");
        const label = matOn ? "Presente" : "Ausente";
        materialLed.classList.toggle("is-on", matOn);
        if (stateEl && stateEl.textContent !== label) stateEl.textContent = label;
        materialLed.setAttribute(
          "aria-label",
          "Material lado " + side + (matOn ? " presente" : " ausente")
        );
      }
    });
  }

  async function pollEncoder() {
    if (motionEncPollBusy || motionCommandInFlight) return;
    motionEncPollBusy = true;
    try {
      const res = await fetch("/api/motion/status");
      const data = await res.json();
      if (data.ok && data.sent) {
        motionShowEncoder(data.encoder);
        if (typeof data.lm_servo_on === "boolean") latchLmServo(data.lm_servo_on);
        if (data.feeder_servo_on) {
          const rOn = data.feeder_servo_on.R === true;
          const lOn = data.feeder_servo_on.L === true;
          if (data.feeder_servo_on.R != null || data.feeder_servo_on.L != null) {
            latchFeederServo(rOn && lOn);
          }
        }
      }
    } catch (err) {
      motionShowEncoder(null);
    } finally {
      motionEncPollBusy = false;
    }
  }

  function setEncoderPoll(on) {
    if (motionEncPollTimer) {
      window.clearInterval(motionEncPollTimer);
      motionEncPollTimer = null;
    }
    if (!on) return;
    pollEncoder();
    motionEncPollTimer = window.setInterval(pollEncoder, 400);
  }

  function activateSettingsTab(tabId) {
    els.settingsTabs.forEach(function (btn) {
      const active = btn.dataset.tab === tabId;
      btn.classList.toggle("is-active", active);
      btn.setAttribute("aria-selected", active ? "true" : "false");
    });
    els.settingsTabPanels.forEach(function (panel) {
      const active = panel.dataset.tabPanel === tabId;
      panel.classList.toggle("is-active", active);
      panel.hidden = !active;
    });
    setEncoderPoll(tabId === "motion");
    setPlcPoll(tabId === "plc");
    setPrefeederPoll(tabId === "prefeeder");
    if (tabId === "conexion") {
      loadSerialPorts();
    }
    if (tabId === "cycle") {
      loadCycleConfig();
    }
    if (tabId === "motion") {
      loadMotionTune();
    }
    if (tabId === "plc") {
      restorePlcBlowerMs();
    }
    if (tabId === "prefeeder") {
      loadPrefeederTune();
    }
  }

  async function fetchState() {
    try {
      const res = await fetch("/api/state");
      const data = await res.json();
      if (data.ok) {
        applyState(data);
        return data;
      }
    } catch (err) {
      pushLocalEvent("No se pudo leer el estado de la máquina.", "error");
    }
    return null;
  }

  function labelForAction(action) {
    const labels = {
      start: "Iniciar",
      step_start: "Prueba paso a paso",
      step_next: "Siguiente paso",
      resume: "Reanudar",
      pause: "Pausa",
      stop: "Detener",
      reset: "Reset",
      parametros: "Parámetros",
      mantenimiento: "Mantenimiento",
      checklist: "Checklist",
      purga: "Purga",
      purga_alimentar: "Alimentar",
      purga_limpiar: "Expulsar",
      purga_limpiar_r: "Expulsar R",
      purga_limpiar_l: "Expulsar L",
      purga_continuar: "Continuar",
      purga_terminar: "Terminar",
      posicion_inicial: "Posición inicial",
      charola_vacia: "Charola vacía",
      iniciar: "Iniciar (PreAlimentador)",
      alimentar_izquierda: "JOG - L",
      alimentar_derecha: "JOG - R",
      recovery_confirm_piezas: "Piezas bien",
      recovery_confirm_manguera: "Continuar recuperación",
      recovery_safe_zone: "Safe zone",
      recovery_retry_purga: "Retry purga",
      recovery_cancel_purga: "Cancelar purga",
      recovery_skip: "SKIP recuperación",
      recovery_done: "Volver",
      joint_alimentar: "Joint Alimentar",
      joint_liberar: "Joint Liberar",
      joint_continuar: "Joint Continuar",
      joint_retry: "Joint Retry",
      joint_confirm: "Joint Confirm",
      joint_continue: "Joint Continue",
      joint_finish: "Joint Finish",
      materialista: "Materialista",
      tecnico: "Técnico",
      jefe: "Jefe de grupo",
    };
    return labels[action] || action;
  }

  function syncAndonCallButtons(state) {
    const call = String((state && state.andon_call) || "");
    document.querySelectorAll("[data-andon-call]").forEach(function (btn) {
      const name = btn.getAttribute("data-andon-call") || "";
      const on = !!name && name === call;
      btn.classList.toggle("is-active", on);
      btn.setAttribute("aria-pressed", on ? "true" : "false");
    });
  }

  function syncCycleStepTestControls(state) {
    const stepMode = !!state.ciclo_paso_a_paso;
    const estado = state.estado || "idle";
    const enCurso = !!state.ciclo_en_curso;
    const enError = estado === "error" || !!state.ciclo_error;
    const postErrorPurga = !!state.post_error_purga;
    const running = estado === "running";
    const paused = estado === "paused";
    const rutina = !!state.rutina;
    const ch = state.checklist || {};
    const checklistReady = !!ch.ok || !!state.checklist_bypass;

    if (els.cycleStepStart) {
      const modeloOk = hasModeloSeleccionado(state);
      els.cycleStepStart.disabled =
        !checklistReady ||
        !modeloOk ||
        rutina ||
        running ||
        (paused && enCurso) ||
        enError ||
        postErrorPurga;
      els.cycleStepStart.title = !checklistReady
        ? "Completa la revisión de arranque."
        : !modeloOk
          ? "Selecciona un modelo antes de Iniciar."
          : running || (paused && enCurso)
            ? "Hay un ciclo en curso."
            : enError
              ? "Corrige el error y pulsa Reset."
              : postErrorPurga
                ? "Tras un error debes completar la purga."
                : "Inicia una pieza y pausa al terminar cada paso.";
    }
    if (els.cycleStepNext) {
      const canNext =
        stepMode && paused && enCurso && !enError && !rutina && !postErrorPurga;
      els.cycleStepNext.disabled = !canNext;
      els.cycleStepNext.title = canNext
        ? "Ejecuta el siguiente paso y vuelve a pausar."
        : stepMode && running
          ? "Espera a que termine el paso en curso."
          : postErrorPurga
            ? "Tras un error debes completar la purga."
            : "Solo en pausa durante la prueba paso a paso.";
    }
    if (els.cycleStepStop) {
      els.cycleStepStop.disabled = !(
        stepMode &&
        (running || paused || enCurso || enError || postErrorPurga)
      );
      els.cycleStepStop.title = els.cycleStepStop.disabled
        ? ""
        : "Detiene la prueba paso a paso.";
    }
    if (els.cycleStepTestHint) {
      if (stepMode && running) {
        els.cycleStepTestHint.textContent =
          state.mensaje || "Ejecutando paso…";
      } else if (stepMode && paused && enCurso && !postErrorPurga) {
        els.cycleStepTestHint.textContent =
          state.mensaje || "Paso listo. Pulsa «Siguiente paso».";
      } else if (stepMode && enError) {
        els.cycleStepTestHint.textContent =
          state.ciclo_error || state.mensaje || "Error en la prueba.";
      } else if (stepMode && postErrorPurga) {
        els.cycleStepTestHint.textContent =
          state.mensaje || "Completa la purga para continuar la prueba.";
      } else {
        els.cycleStepTestHint.textContent =
          "Ejecuta una pieza y se detiene al completar cada paso. Usa «Siguiente paso» para continuar.";
      }
    }
  }

  let processActionInFlight = false;

  async function sendAction(block, action, extra) {
    // Evita Start/Resume/Stop duplicados por doble clic (creaba workers gemelos).
    const isProcessGate =
      block === "proceso" &&
      (action === "start" ||
        action === "resume" ||
        action === "stop" ||
        action === "reset" ||
        action === "step_start" ||
        action === "step_next");
    if (isProcessGate) {
      if (processActionInFlight) return;
      processActionInFlight = true;
    }
    try {
      const res = await fetch("/api/action", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(Object.assign({ block: block, action: action }, extra || {})),
      });
      const data = await res.json();
      if (!data.ok) {
        const errorText = data.error || "Error al registrar acción.";
        pushLocalEvent(errorText, "error");
        if (data.state) applyState(data.state);
        return;
      }
      pushLocalEvent("[" + block + "] " + labelForAction(action));
      if (block === "proceso" && PROCESS_ACTIONS.has(action)) {
        lastProcessAction = action;
      }
      if (data.state) applyState(data.state);
      if (
        action === "reset" &&
        data.state &&
        data.state.recovery &&
        data.state.recovery.active &&
        data.state.recovery.step === "piezas"
      ) {
        openRecoveryModal();
      }
    } catch (err) {
      pushLocalEvent("Sin conexión con el servidor.", "error");
    } finally {
      if (isProcessGate) processActionInFlight = false;
    }
  }

  function openParametros(tabId) {
    els.settingsModal.hidden = true;
    els.parametrosModal.hidden = false;
    activateSettingsTab(tabId || "cycle");
    els.parametrosModalClose.focus();
  }

  function closeParametros(reopenSettings) {
    setEncoderPoll(false);
    setPlcPoll(false);
    els.parametrosModal.hidden = true;
    if (reopenSettings) {
      els.settingsModal.hidden = false;
      els.settingsModalClose.focus();
    } else {
      els.btnAppSettings.focus();
    }
  }

  let serialSyncing = false;
  let serialSaveSeq = 0;
  const serialTesting = {};

  /* ==========================================================================
   * ANDON_SKIP_OPCIONAL — TODO_BORRAR
   * Puente de banco: opción «No usar Andon», mensajes y fillPortSelect(extraNone).
   * Buscar este tag en app.py y rutinas-operador.md para eliminar todo junto.
   * ========================================================================== */
  /** True cuando Andon no se usa (Conexión «No usar» o HMI_ANDON_SKIP=1). */
  let serialAndonSkipped = false;
  /** Valor enviado al guardar cuando se elige «No usar Andon» (coincide con app.py). */
  let andonNoneToken = "__none__";

  const serialUi = {
    ports: [],
    assign: { motion: "", plc: "", andon: "", prefeeder: "" },
    online: {},
  };

  const SERIAL_MODULES = [
    {
      key: "motion",
      name: "Motion",
      select: function () { return els.selectMotionPort; },
      status: function () { return els.serialMotionStatus; },
      badge: function () { return els.serialMotionBadge; },
      button: function () { return els.btnMotionTest; },
      required: true,
    },
    {
      key: "plc",
      name: "PLC",
      select: function () { return els.selectPlcPort; },
      status: function () { return els.serialPlcStatus; },
      badge: function () { return els.serialPlcBadge; },
      button: function () { return els.btnPlcTest; },
      required: true,
    },
    {
      key: "andon",
      name: "Andon",
      select: function () { return els.selectAndonPort; },
      status: function () { return els.serialAndonStatus; },
      badge: function () { return els.serialAndonBadge; },
      button: function () { return els.btnAndonTest; },
      required: true,
      extraNone: function () {
        return { value: andonNoneToken, label: "No usar Andon" };
      },
    },
    {
      key: "prefeeder",
      name: "PreAlimentador",
      select: function () { return els.selectPrefeederPort; },
      status: function () { return els.serialPrefeederStatus; },
      badge: function () { return els.serialPrefeederBadge; },
      button: function () { return els.btnPrefeederTest; },
      required: false,
    },
  ];

  function setLinkStatus(el, text, ok) {
    if (!el) return;
    el.textContent = text;
    el.classList.toggle("is-error", ok === false);
    el.classList.toggle("is-ok", ok === true);
    el.classList.toggle("is-offline", ok == null);
  }

  function setModuleLinkState(mod, state, detail) {
    const card = document.querySelector(
      '.conexion-module[data-serial-module="' + mod.key + '"]'
    );
    const badge = mod.badge ? mod.badge() : null;
    const linkState = state === "online" || state === "error" || state === "testing"
      ? state
      : "offline";
    if (card) card.setAttribute("data-link-state", linkState);
    if (badge) {
      badge.textContent =
        linkState === "online"
          ? "Online"
          : linkState === "error"
            ? "Error"
            : linkState === "testing"
              ? "Probando"
              : "Offline";
    }
    const assign = serialUi.assign[mod.key] || "";
    const portLabel =
      assign && assign !== andonNoneToken ? assign : "sin puerto";
    let text = detail || "";
    if (!text) {
      if (linkState === "online") text = "Online · " + portLabel;
      else if (linkState === "error") text = "Error · " + portLabel;
      else if (linkState === "testing") text = "Probando · " + portLabel + "…";
      else text = "Offline · " + portLabel;
    }
    const ok =
      linkState === "online" ? true : linkState === "error" ? false : null;
    setLinkStatus(mod.status(), text, ok);
  }

  function showComErrorInBanner(message) {
    if (!message || !els.statusBanner) return;
    const text = String(message || "").trim();
    els.statusBanner.className =
      "status-banner status-banner--full status-banner--error";
    const parsed = parseRecoveryError("", text);
    const code = String(parsed.code || "").trim().toUpperCase();
    lastBannerErrorCode = code;
    bannerOpenTarget = "conexion";
    const standard = formatErrorStandard(parsed, parsed.code);
    if (els.statusBannerLabel) els.statusBannerLabel.textContent = standard;
    if (els.statusBannerDetail) {
      els.statusBannerDetail.textContent = "";
      els.statusBannerDetail.classList.remove("is-warn");
    }
    if (els.btnOpenRecovery) {
      els.btnOpenRecovery.hidden = false;
      els.btnOpenRecovery.disabled = false;
      els.btnOpenRecovery.textContent = "Abrir";
    }
    pushLocalEvent(text, "error");
  }

  function andonConnectionStatusText() {
    if (serialAndonSkipped && serialUi.assign.andon === andonNoneToken) {
      return "Andon: no usado (puedes conectarlo más tarde en este menú).";
    }
    if (serialAndonSkipped && !serialUi.assign.andon) {
      return "Andon: omitido en esta sesión (variable HMI_ANDON_SKIP=1).";
    }
    return "";
  }

  function portsForModule(moduleKey) {
    const mine = serialUi.assign[moduleKey] || "";
    const taken = {};
    SERIAL_MODULES.forEach(function (mod) {
      const port = serialUi.assign[mod.key] || "";
      if (!port || mod.key === moduleKey) return;
      if (moduleKey === "andon" && port === andonNoneToken) return;
      if (port === andonNoneToken) return;
      taken[port] = mod.name;
    });
    const out = [];
    const seen = {};
    (serialUi.ports || []).forEach(function (port) {
      if (!port || !port.device || seen[port.device]) return;
      seen[port.device] = true;
      if (taken[port.device] && port.device !== mine) return;
      const online = serialUi.online[moduleKey];
      let label = port.description ? port.device + " — " + port.description : port.device;
      if (mine === port.device && online && online.ok && online.port === port.device) {
        label += " · en línea";
      }
      out.push({ device: port.device, label: label });
    });
    return out;
  }

  function fillOnePortSelect(mod) {
    const select = mod.select();
    if (!select) return;
    let current = serialUi.assign[mod.key] || "";
    // COM ausente → Sin puerto (sin opciones fantasma).
    if (current && current !== andonNoneToken && !portIsPresent(current)) {
      current = "";
      serialUi.assign[mod.key] = "";
      if (serialUi.online[mod.key]) delete serialUi.online[mod.key];
    }
    const extraNone = mod.extraNone ? mod.extraNone() : null;
    select.innerHTML = "";
    if (extraNone && extraNone.value) {
      const optNone = document.createElement("option");
      optNone.value = extraNone.value;
      optNone.textContent = extraNone.label || "—";
      select.appendChild(optNone);
    }
    const none = document.createElement("option");
    none.value = "";
    none.textContent = "Sin puerto";
    select.appendChild(none);
    portsForModule(mod.key).forEach(function (port) {
      const opt = document.createElement("option");
      opt.value = port.device;
      opt.textContent = port.label;
      select.appendChild(opt);
    });
    select.value = current;
    if (select.value !== current) select.value = "";
  }

  function rebuildAllPortSelects() {
    serialSyncing = true;
    try {
      SERIAL_MODULES.forEach(fillOnePortSelect);
    } finally {
      serialSyncing = false;
    }
  }

  function portIsPresent(device) {
    if (!device || device === andonNoneToken) return false;
    const want = String(device).toUpperCase();
    return (serialUi.ports || []).some(function (port) {
      return port && String(port.device || "").toUpperCase() === want;
    });
  }

  function paintModuleStatus(mod) {
    if (!mod.status()) return;
    if (serialTesting[mod.key]) {
      setModuleLinkState(mod, "testing");
      return;
    }
    const assign = serialUi.assign[mod.key] || "";
    if (mod.key === "andon") {
      const hint = andonConnectionStatusText();
      if (hint) {
        setModuleLinkState(mod, "offline", hint);
        return;
      }
    }
    if (!assign || (assign !== andonNoneToken && !portIsPresent(assign))) {
      if (assign && assign !== andonNoneToken) {
        serialUi.assign[mod.key] = "";
        if (serialUi.online[mod.key]) delete serialUi.online[mod.key];
      }
      if (mod.required) {
        setModuleLinkState(
          mod,
          "error",
          "Error · sin puerto. El ciclo no arranca."
        );
      } else {
        setModuleLinkState(mod, "offline", "Offline · sin puerto (opcional).");
      }
      return;
    }
    const online = serialUi.online[mod.key];
    if (online && online.port === assign) {
      if (online.ok) {
        setModuleLinkState(
          mod,
          "online",
          "Online · " + assign + (online.message ? " — " + online.message : "")
        );
      } else {
        setModuleLinkState(
          mod,
          "error",
          "Error · " + assign + (online.message ? " — " + online.message : "")
        );
      }
      return;
    }
    setModuleLinkState(
      mod,
      "offline",
      "Offline · " + assign + ". Pulsa Probar."
    );
  }

  function paintAllModuleStatus() {
    SERIAL_MODULES.forEach(paintModuleStatus);
  }

  function applyOnlineMap(online) {
    serialUi.online = {};
    if (!online || typeof online !== "object") return;
    Object.keys(online).forEach(function (key) {
      const item = online[key];
      if (!item || typeof item !== "object") return;
      serialUi.online[key] = {
        port: String(item.port || ""),
        ok: !!item.ok,
        message: String(item.message || ""),
      };
    });
  }

  function applySerialPayload(data) {
    if (!data) return;
    if (data.andon_none_value) andonNoneToken = data.andon_none_value;
    if (Array.isArray(data.ports)) serialUi.ports = data.ports;
    serialUi.assign.motion = data.motion || "";
    serialUi.assign.plc = data.plc || "";
    serialUi.assign.prefeeder = data.prefeeder || "";
    serialAndonSkipped = !!data.andon_skipped;
    if (data.andon_disabled && data.andon_none_value) {
      serialUi.assign.andon = data.andon_none_value;
    } else {
      serialUi.assign.andon = data.andon || "";
    }
    if (data.online) applyOnlineMap(data.online);
    rebuildAllPortSelects();
    paintAllModuleStatus();
  }

  async function loadSerialPorts() {
    if (!els.selectMotionPort || !els.selectPlcPort || !els.selectAndonPort) return;
    serialSyncing = true;
    try {
      const res = await fetch("/api/serial");
      const data = await res.json();
      applySerialPayload(data);
    } catch (err) {
      if (els.serialMotionStatus) {
        setLinkStatus(els.serialMotionStatus, "No se pudo leer la lista de puertos.", false);
      }
    } finally {
      serialSyncing = false;
    }
  }

  async function saveSerialPorts() {
    if (serialSyncing || !els.selectMotionPort || !els.selectPlcPort || !els.selectAndonPort) return;
    const seq = ++serialSaveSeq;
    const payload = {
      motion: els.selectMotionPort.value,
      plc: els.selectPlcPort.value,
      andon: els.selectAndonPort.value,
      prefeeder: els.selectPrefeederPort ? els.selectPrefeederPort.value : "",
    };
    const prev = {
      motion: serialUi.assign.motion,
      plc: serialUi.assign.plc,
      andon: serialUi.assign.andon,
      prefeeder: serialUi.assign.prefeeder,
    };
    serialUi.assign.motion = payload.motion;
    serialUi.assign.plc = payload.plc;
    serialUi.assign.andon = payload.andon;
    serialUi.assign.prefeeder = payload.prefeeder;
    // Al cambiar un puerto se invalida su prueba anterior.
    ["motion", "plc", "andon", "prefeeder"].forEach(function (key) {
      if (prev[key] !== serialUi.assign[key]) delete serialUi.online[key];
    });
    rebuildAllPortSelects();
    paintAllModuleStatus();
    try {
      const res = await fetch("/api/serial", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      const data = await res.json();
      if (seq !== serialSaveSeq) return;
      if (!data.ok) {
        showComErrorInBanner(data.error || "No se pudo guardar el puerto.");
        serialUi.assign.motion = prev.motion;
        serialUi.assign.plc = prev.plc;
        serialUi.assign.andon = prev.andon;
        serialUi.assign.prefeeder = prev.prefeeder;
        rebuildAllPortSelects();
        paintAllModuleStatus();
        return;
      }
      serialUi.assign.motion = data.motion || "";
      serialUi.assign.plc = data.plc || "";
      serialUi.assign.prefeeder = data.prefeeder || "";
      serialAndonSkipped = !!data.andon_skipped;
      if (data.andon_disabled && data.andon_none_value) {
        serialUi.assign.andon = data.andon_none_value;
      } else {
        serialUi.assign.andon = data.andon || "";
      }
      if (data.online) applyOnlineMap(data.online);
      rebuildAllPortSelects();
      paintAllModuleStatus();
    } catch (err) {
      if (seq !== serialSaveSeq) return;
      showComErrorInBanner("Sin conexión con el servidor.");
      serialUi.assign.motion = prev.motion;
      serialUi.assign.plc = prev.plc;
      serialUi.assign.andon = prev.andon;
      serialUi.assign.prefeeder = prev.prefeeder;
      rebuildAllPortSelects();
      paintAllModuleStatus();
    }
  }

  async function loadChecklistBypass() {
    if (!els.toggleChecklistBypass) return;
    try {
      const res = await fetch("/api/settings/checklist-bypass");
      const data = await res.json();
      if (data.ok) els.toggleChecklistBypass.checked = data.enabled === true;
    } catch (err) {
      pushLocalEvent("No se pudo leer el bypass del checklist.", "error");
    }
  }

  async function saveChecklistBypass() {
    if (!els.toggleChecklistBypass) return;
    const enabled = els.toggleChecklistBypass.checked;
    try {
      const res = await fetch("/api/settings/checklist-bypass", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ enabled: enabled }),
      });
      const data = await res.json();
      if (!data.ok) throw new Error(data.error || "No se pudo guardar.");
      els.toggleChecklistBypass.checked = data.enabled === true;
      pushLocalEvent(data.enabled ? "Bypass de checklist activo." : "Checklist obligatorio.");
      await fetchState();
    } catch (err) {
      els.toggleChecklistBypass.checked = !enabled;
      pushLocalEvent(err.message || "No se pudo guardar el bypass del checklist.", "error");
    }
  }

  function openSettings() {
    els.parametrosModal.hidden = true;
    els.settingsModal.hidden = false;
    loadChecklistBypass();
    els.settingsModalClose.focus();
  }

  function closeSettings() {
    els.settingsModal.hidden = true;
    els.btnAppSettings.focus();
  }

  if (els.selectMotionPort) els.selectMotionPort.addEventListener("change", saveSerialPorts);
  if (els.selectPlcPort) els.selectPlcPort.addEventListener("change", saveSerialPorts);
  if (els.selectAndonPort) els.selectAndonPort.addEventListener("change", saveSerialPorts);
  if (els.selectPrefeederPort) els.selectPrefeederPort.addEventListener("change", saveSerialPorts);

  async function testModuleCom(moduleName) {
    const mod = SERIAL_MODULES.find(function (item) { return item.key === moduleName; });
    if (!mod) return;
    const statusEl = mod.status();
    const button = mod.button();
    if (moduleName === "andon" && serialAndonSkipped) {
      setLinkStatus(statusEl, "Andon no usado. No hay puerto que probar.", null);
      return;
    }
    if (serialTesting[moduleName]) return;
    serialTesting[moduleName] = true;
    if (button) button.disabled = true;
    setLinkStatus(statusEl, "Probando…", null);
    const controller = typeof AbortController !== "undefined" ? new AbortController() : null;
    const timer = controller
      ? setTimeout(function () { controller.abort(); }, 12000)
      : null;
    try {
      const res = await fetch("/api/serial/test", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ module: moduleName }),
        signal: controller ? controller.signal : undefined,
      });
      const data = await res.json();
      if (data.online) applyOnlineMap(data.online);
      if (data.ok) {
        serialUi.online[moduleName] = {
          port: data.port || serialUi.assign[moduleName] || "",
          ok: true,
          message: data.message || moduleName.toUpperCase() + " en línea.",
        };
        rebuildAllPortSelects();
        paintAllModuleStatus();
        fetchState();
      } else {
        serialUi.online[moduleName] = {
          port: serialUi.assign[moduleName] || "",
          ok: false,
          message: data.error || "Sin respuesta.",
        };
        showComErrorInBanner(data.error || "El puerto no respondió.");
        paintAllModuleStatus();
      }
    } catch (err) {
      const aborted = err && err.name === "AbortError";
      const msg = aborted
        ? "Tiempo agotado al probar el puerto."
        : "Sin conexión con el servidor.";
      serialUi.online[moduleName] = {
        port: serialUi.assign[moduleName] || "",
        ok: false,
        message: msg,
      };
      showComErrorInBanner(msg);
      paintAllModuleStatus();
    } finally {
      if (timer) clearTimeout(timer);
      serialTesting[moduleName] = false;
      if (button) button.disabled = false;
      paintModuleStatus(mod);
    }
  }

  let serialTestAllBusy = false;

  async function testAllModuleCom() {
    if (serialTestAllBusy) return;
    serialTestAllBusy = true;
    if (els.btnTestAllPorts) els.btnTestAllPorts.disabled = true;
    try {
      for (let i = 0; i < SERIAL_MODULES.length; i++) {
        const mod = SERIAL_MODULES[i];
        if (mod.key === "andon" && serialAndonSkipped) continue;
        const assign = serialUi.assign[mod.key] || "";
        if (!assign || assign === andonNoneToken) {
          paintModuleStatus(mod);
          continue;
        }
        await testModuleCom(mod.key);
      }
      pushLocalEvent("Prueba de puertos terminada.");
    } finally {
      serialTestAllBusy = false;
      if (els.btnTestAllPorts) els.btnTestAllPorts.disabled = false;
    }
  }

  if (els.btnRefreshPorts) els.btnRefreshPorts.addEventListener("click", loadSerialPorts);
  if (els.btnTestAllPorts) {
    els.btnTestAllPorts.addEventListener("click", function () {
      testAllModuleCom();
    });
  }
  if (els.btnMotionTest) {
    els.btnMotionTest.addEventListener("click", function () {
      testModuleCom("motion");
    });
  }
  if (els.btnPlcTest) {
    els.btnPlcTest.addEventListener("click", function () {
      testModuleCom("plc");
    });
  }
  if (els.btnAndonTest) {
    els.btnAndonTest.addEventListener("click", function () {
      testModuleCom("andon");
    });
  }
  if (els.btnPrefeederTest) {
    els.btnPrefeederTest.addEventListener("click", function () {
      testModuleCom("prefeeder");
    });
  }
  /* --- fin ANDON_SKIP_OPCIONAL (serial / conexión Andon en JS) — TODO_BORRAR --- */

  if (els.prefeederTuneReload) {
    els.prefeederTuneReload.addEventListener("click", function () {
      prefeederDirty.L = false;
      prefeederDirty.R = false;
      loadPrefeederTune(true);
    });
  }
  if (els.prefeederStatusRefresh) {
    els.prefeederStatusRefresh.addEventListener("click", pollPrefeeder);
  }
  if (els.prefeederTuneForm) {
    els.prefeederTuneForm.querySelectorAll("[data-pf-save]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        const side = btn.getAttribute("data-pf-save");
        if (side === "L" || side === "R") savePrefeederSide(side);
      });
    });
  }

  els.btnAppSettings.addEventListener("click", openSettings);
  els.settingsModalClose.addEventListener("click", closeSettings);
  els.parametrosModalClose.addEventListener("click", function () {
    closeParametros(false);
  });
  els.parametrosModalBack.addEventListener("click", function () {
    closeParametros(true);
  });
  els.settingsModal.addEventListener("click", function (e) {
    if (e.target === els.settingsModal) closeSettings();
  });
  els.parametrosModal.addEventListener("click", function (e) {
    if (e.target === els.parametrosModal) closeParametros(false);
  });
  document.addEventListener("keydown", function (e) {
    if (e.key !== "Escape") return;
    if (els.recoveryModal && !els.recoveryModal.hidden) {
      closeRecoveryModal();
      return;
    }
    if (els.empleadoModal && !els.empleadoModal.hidden) {
      closeEmpleado();
      return;
    }
    if (els.checklistModal && !els.checklistModal.hidden) {
      closeChecklist();
      return;
    }
    if (els.mantModal && !els.mantModal.hidden) {
      closeMantenimiento();
      return;
    }
    if (!els.parametrosModal.hidden) {
      closeParametros(true);
      return;
    }
    if (!els.settingsModal.hidden) closeSettings();
  });

  if (els.btnOpenRecovery) {
    els.btnOpenRecovery.addEventListener("click", function () {
      openBannerTarget();
      if (bannerOpenTarget !== "conexion" && !isLinkComError(lastBannerErrorCode)) {
        fetchState();
      }
    });
  }
  if (els.recoveryModalClose) {
    els.recoveryModalClose.addEventListener("click", closeRecoveryModal);
  }
  if (els.recoveryModal) {
    els.recoveryModal.addEventListener("click", function (e) {
      if (e.target === els.recoveryModal) closeRecoveryModal();
    });
  }
  async function onRecoveryAction(op, btn) {
    if (op === "salir") {
      closeRecoveryModal();
      return;
    }
    if (op === "reset") {
      await sendAction("proceso", "reset");
      recoveryWantOpen = true;
      openRecoveryModal();
      return;
    }
    if (op === "piezas") {
      await sendAction("operacion_manual", "recovery_confirm_piezas");
      recoveryWantOpen = true;
      openRecoveryModal();
      return;
    }
    if (op === "manguera") {
      await sendAction("operacion_manual", "recovery_confirm_manguera");
      recoveryWantOpen = true;
      openRecoveryModal();
      return;
    }
    if (op === "safe_zone") {
      if (btn) btn.disabled = true;
      await sendAction("operacion_manual", "recovery_safe_zone");
      recoveryWantOpen = true;
      openRecoveryModal();
      fetchState();
      return;
    }
    if (op === "purga_retry") {
      if (btn) btn.disabled = true;
      await sendAction("operacion_manual", "recovery_retry_purga");
      recoveryWantOpen = true;
      openRecoveryModal();
      fetchState();
      return;
    }
    if (op === "purga_cancel") {
      if (btn) btn.disabled = true;
      await sendAction("operacion_manual", "recovery_cancel_purga");
      recoveryWantOpen = true;
      openRecoveryModal();
      fetchState();
      return;
    }
    if (op === "skip") {
      askEmpleado(function (empleado) {
        sendAction("operacion_manual", "recovery_skip", { empleado: empleado }).then(
          function () {
            closeRecoveryModal();
            fetchState();
          }
        );
      });
      return;
    }
    if (op === "listo") {
      await sendAction("operacion_manual", "recovery_done");
      closeRecoveryModal();
      return;
    }
    if (
      op === "joint_alimentar" ||
      op === "joint_liberar" ||
      op === "joint_continuar" ||
      op === "joint_retry" ||
      op === "joint_confirm" ||
      op === "joint_continue" ||
      op === "joint_finish" ||
      op === "joint_classify_good" ||
      op === "joint_classify_bad" ||
      op === "joint_segregate_done"
    ) {
      if (btn) btn.disabled = true;
      await sendAction("operacion_manual", op);
      recoveryWantOpen = true;
      openRecoveryModal();
      fetchState();
    }
  }

  if (els.recoverySkipLink) {
    els.recoverySkipLink.addEventListener("click", function () {
      if (els.recoverySkipLink.hidden) return;
      onRecoveryAction("skip", els.recoverySkipLink);
    });
  }
  if (els.recoveryActions) {
    els.recoveryActions.addEventListener("click", function (e) {
      const btn = e.target.closest("[data-recovery]");
      if (!btn || btn.disabled) return;
      const op = btn.getAttribute("data-recovery");
      onRecoveryAction(op, btn);
    });
  }

  if (els.checklistModalClose) els.checklistModalClose.addEventListener("click", closeChecklist);
  if (els.checklistModal) {
    els.checklistModal.addEventListener("click", function (e) {
      if (e.target === els.checklistModal) closeChecklist();
    });
  }
  if (els.checklistBody) {
    els.checklistBody.addEventListener("click", function (e) {
      const btn = e.target.closest("[data-check]");
      if (!btn || btn.disabled) return;
      const op = btn.getAttribute("data-check");
      const extra = {};
      if (op === "checklist_empezar") {
        const input = document.getElementById("checkEmpleado");
        extra.empleado = input ? input.value.trim() : "";
      }
      if (op === "checklist_medicion") {
        const left = document.getElementById("checkMmL");
        const right = document.getElementById("checkMmR");
        extra.mm_l = left ? left.value : "";
        extra.mm_r = right ? right.value : "";
      }
      sendAction("checklist", op, extra);
    });
  }
  if (els.mantModalClose) els.mantModalClose.addEventListener("click", closeMantenimiento);
  if (els.mantModal) {
    els.mantModal.addEventListener("click", function (e) {
      if (e.target === els.mantModal) closeMantenimiento();
    });
  }
  function showMantResetForm(show) {
    if (els.mantResetForm) els.mantResetForm.hidden = !show;
    if (els.mantReset) els.mantReset.hidden = !!show;
    if (!show && els.mantEmpleado) els.mantEmpleado.value = "";
    if (show && els.mantEmpleado) {
      requestAnimationFrame(function () {
        if (els.mantEmpleado) els.mantEmpleado.focus();
      });
    }
  }

  if (els.mantReset) {
    els.mantReset.addEventListener("click", function () {
      showMantResetForm(true);
    });
  }
  if (els.mantCancel) {
    els.mantCancel.addEventListener("click", function () {
      showMantResetForm(false);
    });
  }
  if (els.mantConfirm) {
    els.mantConfirm.addEventListener("click", async function () {
      const empleado = els.mantEmpleado ? els.mantEmpleado.value.trim() : "";
      if (!/^\d{5}$/.test(empleado)) {
        pushLocalEvent("El número de empleado son cinco dígitos.", "error");
        return;
      }
      await sendAction("mantenimiento", "mantenimiento_reset", {
        empleado: empleado,
      });
      // Solo cierra el formulario si el contador quedó en cero (reset ok).
      if (els.mantCounterValue && els.mantCounterValue.textContent === "0") {
        showMantResetForm(false);
      }
    });
  }
  if (els.mantEmpleado) {
    els.mantEmpleado.addEventListener("keydown", function (e) {
      if (e.key === "Enter") {
        e.preventDefault();
        if (els.mantConfirm) els.mantConfirm.click();
      }
    });
  }
  if (els.empleadoModalClose) els.empleadoModalClose.addEventListener("click", closeEmpleado);
  if (els.empleadoConfirm) els.empleadoConfirm.addEventListener("click", confirmEmpleado);
  if (els.empleadoInput) {
    els.empleadoInput.addEventListener("keydown", function (e) {
      if (e.key === "Enter") {
        e.preventDefault();
        confirmEmpleado();
      }
    });
  }
  if (els.empleadoModal) {
    els.empleadoModal.addEventListener("click", function (e) {
      if (e.target === els.empleadoModal) closeEmpleado();
    });
  }

  if (els.btnClearEvents) els.btnClearEvents.addEventListener("click", clearEventsLog);

  els.toggleEventsPanel.addEventListener("change", function () {
    setEventsPanelVisible(els.toggleEventsPanel.checked);
  });

  if (els.toggleChecklistBypass) {
    els.toggleChecklistBypass.addEventListener("change", saveChecklistBypass);
  }

  els.settingsTabs.forEach(function (btn) {
    btn.addEventListener("click", function () {
      activateSettingsTab(btn.dataset.tab);
    });
  });

  if (els.motionTuneForm) {
    els.motionTuneForm.addEventListener("submit", saveMotionTune);
    function markMotionTuneDirty(ev) {
      const t = ev.target;
      if (t && t.hasAttribute && t.hasAttribute("data-motion-path")) {
        motionTuneDirty = true;
      }
    }
    // checkbox dispara change; number/text también input
    els.motionTuneForm.addEventListener("input", markMotionTuneDirty);
    els.motionTuneForm.addEventListener("change", markMotionTuneDirty);
    const validateEnc = document.getElementById("motionValidateEncoder");
    if (validateEnc) {
      // Toggle aplica al instante: guardar + enviar al MCU (no hace falta «Guardar»).
      validateEnc.addEventListener("change", function () {
        motionTuneDirty = true;
        saveMotionTune();
      });
    }
  }
  if (els.motionTuneReload) {
    els.motionTuneReload.addEventListener("click", function () {
      motionTuneDirty = false;
      loadMotionTune();
    });
  }
  if (els.motionStatusRefresh) {
    els.motionStatusRefresh.addEventListener("click", pollEncoder);
  }
  if (els.cycleParamsForm) {
    els.cycleParamsForm.addEventListener("submit", saveCycleParams);
    els.cycleParamsForm.addEventListener("input", function (ev) {
      const t = ev.target;
      if (t && t.hasAttribute && t.hasAttribute("data-cycle-param")) {
        cycleParamsDirty = true;
      }
    });
  }
  if (els.prefeederTuneForm) {
    els.prefeederTuneForm.addEventListener("input", function (ev) {
      const t = ev.target;
      if (!t || !t.getAttribute) return;
      const side = t.getAttribute("data-pf-side");
      if (side === "L" || side === "R") prefeederDirty[side] = true;
    });
  }
  if (els.plcBlowerMs) {
    els.plcBlowerMs.addEventListener("change", persistPlcBlowerMs);
    els.plcBlowerMs.addEventListener("blur", persistPlcBlowerMs);
  }
  if (els.cycleStepStart) {
    els.cycleStepStart.addEventListener("click", function () {
      sendAction("proceso", "step_start");
    });
  }
  if (els.cycleStepNext) {
    els.cycleStepNext.addEventListener("click", function () {
      sendAction("proceso", "step_next");
    });
  }
  if (els.cycleStepStop) {
    els.cycleStepStop.addEventListener("click", function () {
      sendAction("proceso", "stop");
    });
  }

  const lmMoveMm = document.getElementById("lmMoveMm");
  if (lmMoveMm) {
    lmMoveMm.addEventListener("input", motionRefreshPuuReadout);
    lmMoveMm.addEventListener("change", function () {
      const raw = lmMoveMm.value.trim();
      if (raw === "") return;
      const snapped = motionSnapMm(raw);
      if (!Number.isFinite(snapped)) return;
      lmMoveMm.value = snapped.toFixed(1);
      motionRefreshPuuReadout();
    });
  }

  const lmCalMm = document.getElementById("lmCalMm");
  if (lmCalMm) {
    lmCalMm.addEventListener("input", motionRefreshCalPuuReadout);
    lmCalMm.addEventListener("change", function () {
      const raw = lmCalMm.value.trim();
      if (raw === "") return;
      const snapped = motionSnapMm(raw);
      if (!Number.isFinite(snapped)) return;
      lmCalMm.value = snapped.toFixed(1);
      motionRefreshCalPuuReadout();
    });
  }

  const lmCalibrateBtn = document.getElementById("lmCalibrateBtn");
  if (lmCalibrateBtn) {
    lmCalibrateBtn.addEventListener("click", async function () {
      flashButton(lmCalibrateBtn);
      const cmdInput = document.getElementById("lmCalMm");
      const realInput = document.getElementById("lmCalRealMm");
      const mmCmd = motionSnapMm(String((cmdInput && cmdInput.value) || "").trim());
      const mmReal = Number(String((realInput && realInput.value) || "").trim());
      if (!Number.isFinite(mmCmd) || mmCmd < 1) {
        pushLocalEvent("Escribe el mm pedido (mover) para calibrar.", "error");
        return;
      }
      if (!Number.isFinite(mmReal) || mmReal < 1) {
        pushLocalEvent("Escribe el mm real medido para calibrar.", "error");
        return;
      }
      if (motionCommandInFlight) {
        pushLocalEvent(
          "Motion está ejecutando otra orden. Espera a que termine.",
          "error"
        );
        return;
      }
      motionCommandInFlight = true;
      try {
        const res = await fetch("/api/motion/cmd", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            op: "lmguide_calibrate",
            mm_cmd: mmCmd,
            mm_real: mmReal,
          }),
        });
        const data = await res.json();
        if (!data.ok) {
          pushLocalEvent(data.error || "No se pudo calibrar el LM Guide.", "error");
          return;
        }
        motionApplyScale(data);
        if (data.tune) motionFormFromTune(data.tune);
        const oldK = Number(data.steps_per_mm_old);
        const newK = Number(data.steps_per_mm);
        pushLocalEvent(
          "LM Guide calibrado: " +
            mmCmd.toFixed(1) +
            " mm pedido → " +
            mmReal.toFixed(2) +
            " mm real. Escala " +
            (Number.isFinite(oldK) ? oldK.toFixed(6) : "?") +
            " → " +
            (Number.isFinite(newK) ? newK.toFixed(6) : "?") +
            " PUU/mm."
        );
        setMotionHint("Escala LM Guide actualizada.");
      } catch (err) {
        pushLocalEvent("Error al calibrar el LM Guide.", "error");
      } finally {
        motionCommandInFlight = false;
      }
    });
  }

  if (els.motionTuneForm) {
    els.motionTuneForm.querySelectorAll("[data-motion-op]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        runMotionButton(btn);
      });
    });
  }
  if (els.btnInitActuators) {
    els.btnInitActuators.addEventListener("click", function () {
      runMotionButton(els.btnInitActuators);
    });
  }

  function plcButton(name) {
    if (!els.plcManualForm) return null;
    return els.plcManualForm.querySelector('[data-plc-name="' + name + '"]');
  }

  function setPlcPressed(btn, on) {
    if (!btn) return;
    btn.setAttribute("aria-pressed", on ? "true" : "false");
    btn.classList.toggle("is-active", !!on);
    if (!on) btn.classList.remove("is-vivid");
    setToggleLabel(btn, on);
  }

  function setPlcHint(text) {
    if (!els.plcEsp32Hint) return;
    els.plcEsp32Hint.hidden = !text;
    els.plcEsp32Hint.textContent = text || "";
  }

  function applyPlcStatus(status) {
    if (!status) return;
    PLC_FAULTS.forEach(function (pair) {
      const el = document.getElementById(pair[0]);
      if (el) el.classList.toggle("is-on", status[pair[1]] === true);
    });
    if (els.plcGraceNote) els.plcGraceNote.hidden = status.boot_grace !== true;
    const blowerBtn = els.plcManualForm
      ? els.plcManualForm.querySelector('[data-plc-op="blower"]')
      : null;
    if (typeof status.blower === "boolean") setPlcPressed(blowerBtn, status.blower);
    PLC_OUTPUTS.forEach(function (pair) {
      if (typeof status[pair[0]] === "boolean") setPlcPressed(plcButton(pair[1]), status[pair[0]]);
    });
    if (
      typeof status.holder === "boolean" &&
      typeof status.encoder === "boolean"
    ) {
      lastPlcOutputs = {
        known: true,
        holder: status.holder,
        encoder: status.encoder,
        gripper: status.gripper === true,
        cutter_r: status.cutter_r === true,
        cutter_l: status.cutter_l === true,
      };
    }
    syncPlcHolderEncoderButton(status);
    syncMotionCuttersButton(status);
  }

  function clearPlcOutputs() {
    PLC_OUTPUTS.forEach(function (pair) {
      setPlcPressed(plcButton(pair[1]), false);
    });
    const blowerBtn = els.plcManualForm
      ? els.plcManualForm.querySelector('[data-plc-op="blower"]')
      : null;
    setPlcPressed(blowerBtn, false);
    setPlcPressed(plcHolderEncoderButton(), false);
    setPlcPressed(motionCuttersButton(), false);
  }

  async function sendPlcOp(op, extra) {
    const body = Object.assign({ op: op }, extra || {});
    try {
      const res = await fetch("/api/plc/cmd", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se armó la orden PLC.", "error");
        return null;
      }
      if (!data.sent) {
        pushLocalEvent(data.reason || "No se envió la orden PLC.", "error");
        return null;
      }
      if (data.command_ok !== true) {
        pushLocalEvent("El MCU PLC no confirmó la orden: " + op, "error");
        return null;
      }
      if (data.sent) {
        pushLocalEvent("PLC: orden enviada.");
        setPlcHint("Listo.");
      } else {
        const reason = data.reason || "No se envió al puerto.";
        pushLocalEvent(reason, "error");
        setPlcHint(reason);
      }
      if (data.status) applyPlcStatus(data.status);
      return data;
    } catch (err) {
      pushLocalEvent("Error al enviar la orden PLC.", "error");
      return null;
    }
  }

  async function runPlcButton(btn) {
    const op = btn.getAttribute("data-plc-op");
    if (!op) return;
    // Un click enciende y se mantiene; otro click apaga. No hace falta sostener.
    if (btn.dataset.plcBusy === "1") return;
    flashButton(btn);
    if (op === "valve") {
      const name = btn.getAttribute("data-plc-name");
      // Liberar: decidir con el espejo si está fresco; si no, con aria-pressed.
      let on = btn.getAttribute("aria-pressed") !== "true";
      if (name === "holder_encoder" && lastPlcOutputs && lastPlcOutputs.known) {
        const closed =
          lastPlcOutputs.holder === true && lastPlcOutputs.encoder === true;
        on = !closed;
      }
      btn.dataset.plcBusy = "1";
      setPlcPressed(btn, on);
      const data = await sendPlcOp("valve", { name: name, on: on });
      btn.dataset.plcBusy = "0";
      if (!data || data.command_ok !== true) {
        setPlcPressed(btn, !on);
      } else if (data.status) {
        // sendPlcOp ya aplicó status; reforzar Liberar por si el reply fue parcial.
        syncPlcHolderEncoderButton(data.status);
      }
      return;
    }
    if (op === "blower") {
      const on = btn.getAttribute("aria-pressed") !== "true";
      btn.dataset.plcBusy = "1";
      if (!on) {
        setPlcPressed(btn, false);
        const dataOff = await sendPlcOp("blower_off");
        btn.dataset.plcBusy = "0";
        if (!dataOff || dataOff.command_ok !== true) setPlcPressed(btn, true);
        return;
      }
      persistPlcBlowerMs();
      const raw = els.plcBlowerMs ? String(els.plcBlowerMs.value).trim() : "";
      let saved = "";
      try {
        saved = localStorage.getItem(STORAGE_PLC_BLOWER_MS) || "";
      } catch (err) {
        saved = "";
      }
      const ms = Number(raw !== "" ? raw : saved || "2000");
      if (!Number.isFinite(ms)) {
        btn.dataset.plcBusy = "0";
        pushLocalEvent("El tiempo del blower tiene que ser un número.", "error");
        return;
      }
      if (els.plcBlowerMs) els.plcBlowerMs.value = String(Math.round(ms));
      persistPlcBlowerMs();
      setPlcPressed(btn, true);
      const dataOn = await sendPlcOp("blower_on", { ms: ms });
      btn.dataset.plcBusy = "0";
      if (!dataOn || dataOn.command_ok !== true) setPlcPressed(btn, false);
      return;
    }
    if (op === "all_off" || op === "reset") {
      await sendPlcOp(op);
    }
  }

  async function pollPlc() {
    try {
      const res = await fetch("/api/plc/status");
      const data = await res.json();
      if (data.ok && data.status) applyPlcStatus(data.status);
    } catch (err) {
      /* sin puerto: se conserva el estado de los botones */
    }
  }

  function setPlcPoll(on) {
    if (plcPollTimer) {
      window.clearInterval(plcPollTimer);
      plcPollTimer = null;
    }
    if (!on) return;
    pollPlc();
    plcPollTimer = window.setInterval(pollPlc, 400);
  }

  function setPfHint(text) {
    if (!els.prefeederEsp32Hint) return;
    els.prefeederEsp32Hint.hidden = !text;
    els.prefeederEsp32Hint.textContent = text || "";
  }

  function prefeederFormFromTune(tune, force) {
    if (!els.prefeederTuneForm || !tune) return;
    const active = document.activeElement;
    const editing = !!(active && els.prefeederTuneForm.contains(active));
    els.prefeederTuneForm.querySelectorAll("[data-pf-side][data-pf-key]").forEach(function (input) {
      const side = input.getAttribute("data-pf-side");
      const key = input.getAttribute("data-pf-key");
      if (!force && (prefeederDirty[side] || (editing && active === input))) return;
      const block = tune[side];
      if (!block || block[key] == null) return;
      input.value = String(block[key]);
    });
  }

  function prefeederSideFromForm(side) {
    const out = {};
    if (!els.prefeederTuneForm) return out;
    els.prefeederTuneForm.querySelectorAll('[data-pf-side="' + side + '"][data-pf-key]').forEach(function (input) {
      const key = input.getAttribute("data-pf-key");
      const raw = input.value.trim();
      if (raw === "") return;
      const num = Number(raw);
      if (!Number.isFinite(num)) return;
      out[key] = num;
    });
    return out;
  }

  function pfPaintBanner(el, sideData) {
    if (!el) return;
    if (!sideData || !sideData.online) {
      el.textContent = sideData ? "Sin enlace" : "—";
      return;
    }
    if (sideData.state === "error") {
      el.textContent = "—";
      return;
    }
    if (sideData.state === "busy") {
      el.textContent = "Busy";
      return;
    }
    el.textContent = "Idle";
  }

  function pfPaintSensor(ledEl, on, faultStyle) {
    if (!ledEl) return;
    const state = ledEl.querySelector(".motion-laser__state, .pf-sensor__state");
    const label = on ? (faultStyle ? "Activo" : "OK") : "—";
    ledEl.classList.toggle("is-on", !!on && !faultStyle);
    ledEl.classList.toggle("is-fault", !!(on && faultStyle));
    if (state) state.textContent = label;
  }

  function applyPrefeederStatus(status) {
    if (!status) return;
    const tuneFromMaster = {};
    ["L", "R"].forEach(function (side) {
      const block = status[side];
      pfPaintBanner(side === "L" ? els.pfBanL : els.pfBanR, block);
      const sen = (block && block.sensors) || {};
      pfPaintSensor(document.getElementById("pfCyl" + side), !!sen.cylinder, true);
      pfPaintSensor(document.getElementById("pfHolg" + side), !!sen.holgura, false);
      if (block && block.cfg) tuneFromMaster[side] = block.cfg;
    });
    // No pisar valores que el operador está editando o aún no guardó.
    if (els.prefeederTuneForm && Object.keys(tuneFromMaster).length) {
      prefeederFormFromTune(tuneFromMaster);
    }
  }

  async function loadPrefeederTune(force) {
    try {
      const res = await fetch("/api/prefeeder/tune");
      const data = await res.json();
      if (data.ok && data.tune) prefeederFormFromTune(data.tune, !!force);
    } catch (err) {
      pushLocalEvent("No se pudo cargar parámetros del PreAlimentador.", "error");
    }
    await pollPrefeeder();
  }

  async function savePrefeederSide(side) {
    const patch = {};
    patch[side] = prefeederSideFromForm(side);
    try {
      const res = await fetch("/api/prefeeder/tune", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ tune: patch }),
      });
      const data = await res.json();
      if (!data.ok) {
        pushLocalEvent(data.error || "No se guardó el lado " + side + ".", "error");
        return;
      }
      prefeederDirty[side] = false;
      if (data.tune) prefeederFormFromTune(data.tune, true);
      const pushed = data.pushed && data.pushed[side];
      const applied = data.applied && data.applied[side];
      if (applied === false) {
        setPfHint("No quedó aplicado en el maestro (revisa COM). Los valores no se guardaron en el ESP32.");
        pushLocalEvent(
          "PreAlimentador " + side + ": el maestro no refleja los parámetros guardados.",
          "error"
        );
      } else if (!pushed) {
        setPfHint("Sin respuesta del maestro por serial; no se guardó en el ESP32.");
        pushLocalEvent("No se confirmó el envío al maestro (" + side + ").", "error");
      } else {
        setPfHint("Guardado en el maestro. Solo cambiará con otro Guardar desde el HMI.");
        pushLocalEvent("PreAlimentador " + side + " guardado en el maestro.");
      }
      await pollPrefeeder();
    } catch (err) {
      pushLocalEvent("Error al guardar PreAlimentador " + side + ".", "error");
    }
  }

  function restorePlcBlowerMs() {
    if (!els.plcBlowerMs) return;
    let saved = "";
    try {
      saved = localStorage.getItem(STORAGE_PLC_BLOWER_MS) || "";
    } catch (err) {
      saved = "";
    }
    if (!saved) return;
    const ms = Number(saved);
    if (!Number.isFinite(ms) || ms < 200) return;
    els.plcBlowerMs.value = String(Math.round(ms));
  }

  function persistPlcBlowerMs() {
    if (!els.plcBlowerMs) return;
    const raw = String(els.plcBlowerMs.value).trim();
    const ms = Number(raw);
    if (!Number.isFinite(ms) || ms < 200) return;
    try {
      localStorage.setItem(STORAGE_PLC_BLOWER_MS, String(Math.round(ms)));
    } catch (err) {
      /* ignore */
    }
  }

  async function pollPrefeeder() {
    try {
      const res = await fetch("/api/prefeeder/status?force=1");
      const data = await res.json();
      if (data.ok && data.status) applyPrefeederStatus(data.status);
    } catch (err) {
      /* sin puerto */
    }
  }

  function setPrefeederPoll(on) {
    if (pfPollTimer) {
      window.clearInterval(pfPollTimer);
      pfPollTimer = null;
    }
    if (!on) return;
    pollPrefeeder();
    pfPollTimer = window.setInterval(pollPrefeeder, 700);
  }

  if (els.plcManualForm) {
    els.plcManualForm.addEventListener("submit", function (ev) {
      ev.preventDefault();
    });
    els.plcManualForm.querySelectorAll("[data-plc-op]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        runPlcButton(btn);
      });
    });
  }

  if (els.motionTuneForm) {
    els.motionTuneForm.querySelectorAll("[data-plc-op]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        runPlcButton(btn);
      });
    });
  }

  if (els.statusBannerActions) {
    els.statusBannerActions.querySelectorAll("[data-plc-op]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        runPlcButton(btn);
      });
    });
  }

  if (els.selectModelo) {
    els.selectModelo.addEventListener("focus", function () {
      if (syncingModelSelect) return;
      modelComboActive = -1;
      try {
        els.selectModelo.select();
      } catch (e) {
        /* ignore */
      }
      renderModelComboList("", true);
    });
    els.selectModelo.addEventListener("input", function () {
      if (syncingModelSelect) return;
      modelComboActive = 0;
      renderModelComboList(els.selectModelo.value, true);
      const hit = findModeloInCatalog(els.selectModelo.value);
      if (hit) syncModeloDesc(hit.modelo);
      else if (els.metricModeloDesc) {
        els.metricModeloDesc.textContent = "";
        els.metricModeloDesc.title = "";
        els.metricModeloDesc.hidden = true;
      }
    });
    els.selectModelo.addEventListener("keydown", function (ev) {
      if (syncingModelSelect) return;
      const filter = modelComboOpen ? modelComboFilter : els.selectModelo.value;
      const items = filteredModelos(filter);
      if (ev.key === "ArrowDown") {
        ev.preventDefault();
        if (!modelComboOpen) renderModelComboList(els.selectModelo.value, true);
        modelComboActive = Math.min(items.length - 1, modelComboActive + 1);
        if (modelComboActive < 0 && items.length) modelComboActive = 0;
        renderModelComboList(filter, true);
      } else if (ev.key === "ArrowUp") {
        ev.preventDefault();
        if (!modelComboOpen) renderModelComboList(els.selectModelo.value, true);
        modelComboActive = Math.max(0, modelComboActive - 1);
        renderModelComboList(filter, true);
      } else if (ev.key === "Enter") {
        ev.preventDefault();
        if (modelComboOpen && modelComboActive >= 0 && items[modelComboActive]) {
          pickModeloFromCombo(items[modelComboActive].modelo);
        } else {
          const typed = String(els.selectModelo.value || "").trim();
          const hit = findModeloInCatalog(typed);
          if (hit) pickModeloFromCombo(hit.modelo);
          else clearModeloSelection(true);
        }
      } else if (ev.key === "Escape") {
        setModelComboOpen(false);
      }
    });
    els.selectModelo.addEventListener("blur", function () {
      window.setTimeout(function () {
        if (!els.modelCombo || els.modelCombo.contains(document.activeElement)) return;
        setModelComboOpen(false);
        const typed = String(els.selectModelo.value || "").trim();
        const hit = findModeloInCatalog(typed);
        if (hit && !syncingModelSelect) {
          pickModeloFromCombo(hit.modelo);
          return;
        }
        // Texto vacío o inválido (p. ej. "00"): vuelve a Sin selección.
        if (!syncingModelSelect) clearModeloSelection(true);
      }, 120);
    });
  }

  document.addEventListener("mousedown", function (ev) {
    if (!modelComboOpen || !els.modelCombo) return;
    if (els.modelCombo.contains(ev.target)) return;
    setModelComboOpen(false);
  });

  if (els.metricCantidad) {
    els.metricCantidad.addEventListener("change", function () {
      const n = parseInt(String(els.metricCantidad.value).trim(), 10);
      if (!Number.isFinite(n) || n < 1) {
        pushLocalEvent("El Kanban tiene que ser un entero de 1 o más.", "error");
        return;
      }
      saveCantidad(n);
    });
  }

  function onCutCompChange(side, el) {
    if (!el) return;
    el.addEventListener("change", function () {
      const mm = Number(String(el.value).trim());
      if (!Number.isFinite(mm)) {
        pushLocalEvent("La compensación de corte tiene que ser un número.", "error");
        return;
      }
      saveCutCompSide(side, mm);
    });
  }
  onCutCompChange("R", els.metricCutCompR);
  onCutCompChange("L", els.metricCutCompL);

  if (els.themeToggle) {
    els.themeToggle.addEventListener("click", toggleTheme);
  }

  function actionButton(action) {
    return document.querySelector('[data-action="' + action + '"]');
  }

  function setButtonOn(btn, on) {
    if (!btn) return;
    btn.classList.toggle("is-active", !!on);
    btn.setAttribute("aria-pressed", on ? "true" : "false");
    /* flashButton puede dejar is-vivid si is-active llega antes del timeout */
    btn.classList.remove("is-vivid");
  }

  function syncRoutine(state) {
    const rutina = state.rutina || "";
    const fase = state.rutina_fase || "";
    const purga = rutina === "purga";
    const enError = state.estado === "error" || !!state.ciclo_error;
    const postErrorPurga = !!state.post_error_purga;
    const cicloCorriendo = state.estado === "running";
    const purgaBtn = actionButton("purga");
    if (purgaBtn) {
      purgaBtn.disabled =
        cicloCorriendo || enError || (!!rutina && !purga);
      setButtonOn(purgaBtn, purga || (postErrorPurga && !enError && !purga));
      purgaBtn.title = enError
        ? "Abre el error y pulsa Reset antes de la purga."
        : postErrorPurga && !purga
          ? "La recuperación guiada hace la purga sola. También puedes hacerla aquí."
          : "";
      if (postErrorPurga && !enError && !purga && !purgaBtn.disabled) {
        purgaBtn.classList.add("is-active");
        purgaBtn.setAttribute("aria-pressed", "true");
      }
    }
    const home = actionButton("posicion_inicial");
    if (home) {
      home.disabled = !!state.rutina_manos || cicloCorriendo || (!!rutina && rutina !== "purga");
      setButtonOn(home, rutina === "posicion_inicial");
    }
    const pfIniciar = document.querySelector('[data-block="prealimentador"] [data-action="iniciar"]');
    if (pfIniciar) {
      const pf = state.prefeeder || {};
      if (pf.com === false) {
        pfIniciar.title = "Sin COM del maestro (estabilización omitida).";
      } else if (pf.ready) {
        pfIniciar.title = "Ya está listo y bloqueado.";
      } else if (pf.com && !pf.stable) {
        pfIniciar.title = "Estabiliza ambos lados y bloquea.";
      } else {
        pfIniciar.title = "Estabiliza y bloquea el PreAlimentador.";
      }
    }

    if (els.statusBannerActions) {
      const showPurge = purga && fase === "espera";
      const showTray = !!state.charola_llena;
      els.statusBannerActions.hidden = !(showPurge || showTray);
      els.statusBannerActions
        .querySelectorAll("[data-action], [data-plc-op]")
        .forEach(function (btn) {
          const trayBtn = btn.dataset.action === "charola_vacia";
          const show = trayBtn ? showTray : showPurge;
          btn.hidden = !show;
          btn.disabled = !show;
        });
      // Liberar sigue al espejo PLC (Alimentar/Continuar cambian válvulas sin /api/plc).
      const outs = state.plc_outputs;
      if (outs && outs.known) {
        lastPlcOutputs = {
          known: true,
          holder: outs.holder === true,
          encoder: outs.encoder === true,
          gripper: outs.gripper === true,
          cutter_r: outs.cutter_r === true,
          cutter_l: outs.cutter_l === true,
        };
        if (showPurge) syncPlcHolderEncoderButton(lastPlcOutputs);
      }
    }
    renderChecklist(state);
    if (els.mantCounterValue) {
      els.mantCounterValue.textContent = String(state.ciclos_mant || 0);
    }
    if (els.mantCounter) {
      els.mantCounter.textContent = "Ciclos desde el último reset";
    }
  }

  function checkButton(label, op, disabled) {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "btn btn-config";
    btn.textContent = label;
    btn.setAttribute("data-check", op);
    btn.disabled = !!disabled;
    return btn;
  }

  function checklistSection(title) {
    const section = document.createElement("section");
    section.className = "checklist-section";
    const h = document.createElement("h3");
    h.className = "checklist-section__title";
    h.textContent = title;
    section.appendChild(h);
    return section;
  }

  function renderChecklistLog(state) {
    const section = checklistSection("Registro (log)");
    const rows = Array.isArray(state.checklist_log) ? state.checklist_log.slice() : [];
    rows.reverse();
    if (!rows.length) {
      const empty = document.createElement("p");
      empty.className = "checklist-log-empty";
      empty.textContent = "Sin revisiones registradas todavía.";
      section.appendChild(empty);
      return section;
    }
    const list = document.createElement("ul");
    list.className = "checklist-log-list";
    rows.forEach(function (row) {
      const item = document.createElement("li");
      item.className = "checklist-log-item";
      const fecha = document.createElement("span");
      fecha.textContent = row && row.fecha ? String(row.fecha) : "—";
      const emp = document.createElement("span");
      emp.textContent = row && row.empleado ? String(row.empleado) : "—";
      const res = document.createElement("span");
      res.textContent = row && row.resultado ? String(row.resultado) : "—";
      item.appendChild(fecha);
      item.appendChild(emp);
      item.appendChild(res);
      list.appendChild(item);
    });
    section.appendChild(list);
    return section;
  }

  function renderChecklistSideCounts(state) {
    const section = checklistSection("Revisión de ciclos");
    const grid = document.createElement("div");
    grid.className = "checklist-side-counts";
    [
      { side: "R", label: "Derecho (R)", value: state.ciclos_mant_r },
      { side: "L", label: "Izquierdo (L)", value: state.ciclos_mant_l },
    ].forEach(function (item) {
      const card = document.createElement("div");
      card.className = "checklist-side-count";
      const sideEl = document.createElement("span");
      sideEl.className = "checklist-side-count__side";
      sideEl.textContent = item.label;
      const valueEl = document.createElement("span");
      valueEl.className = "checklist-side-count__value";
      const n = Number(item.value);
      valueEl.textContent = Number.isFinite(n) ? String(Math.max(0, Math.round(n))) : "0";
      card.appendChild(sideEl);
      card.appendChild(valueEl);
      grid.appendChild(card);
    });
    section.appendChild(grid);
    const note = document.createElement("p");
    note.className = "modal__hint";
    note.textContent =
      "Piezas buenas por lado desde el último reset de mantenimiento. No usan el lote actual ni reparte el contador global.";
    section.appendChild(note);
    return section;
  }

  function renderChecklist(state) {
    if (!els.checklistModal || els.checklistModal.hidden || !els.checklistBody) return;
    const ch = state.checklist || {};
    const logLen = Array.isArray(state.checklist_log) ? state.checklist_log.length : 0;
    const key = [
      ch.ok,
      state.checklist_bypass,
      ch.paso,
      ch.empleado,
      ch.eje_ok,
      ch.recorrido_ok,
      ch.alimentador_ok,
      ch.purga_ok,
      ch.piezas_cortadas,
      state.ciclos_mant,
      state.ciclos_mant_r,
      state.ciclos_mant_l,
      logLen,
      state.rutina,
      state.mensaje || "",
    ].join("|");
    const live = document.getElementById("checklistLive");
    if (key === checklistViewKey && live) {
      live.textContent = state.mensaje || "";
      return;
    }
    checklistViewKey = key;
    const body = els.checklistBody;
    body.innerHTML = "";
    body.appendChild(renderChecklistLog(state));

    const revision = renderChecklistSideCounts(state);
    const hint = document.createElement("p");
    hint.className = "modal__hint";
    const liveLine = document.createElement("p");
    liveLine.id = "checklistLive";
    liveLine.className = "modal__hint";
    liveLine.textContent = state.mensaje || "";
    const actions = document.createElement("div");
    actions.className = "checklist-actions";

    if (ch.ok || state.checklist_bypass) {
      hint.textContent = state.checklist_bypass
        ? "Resultado: bypass activo. El ciclo puede iniciar sin completar esta revisión."
        : ch.empleado
          ? "Resultado: revisión completa. Empleado " + ch.empleado + "."
          : "Resultado: revisión completa.";
      revision.appendChild(hint);
      revision.appendChild(liveLine);
      body.appendChild(revision);
      return;
    }

    const paso = ch.paso || 1;
    if (paso === 1) {
      const ciclos = state.ciclos_mant || 0;
      hint.textContent =
        "Paso 1 de 6. Ciclos desde mantenimiento: " +
        ciclos +
        ". El tope es 10 000.";
      const field = document.createElement("label");
      field.className = "modal-field";
      field.textContent = "Empleado (5 dígitos)";
      const input = document.createElement("input");
      input.id = "checkEmpleado";
      input.maxLength = 5;
      input.inputMode = "numeric";
      input.autocomplete = "off";
      field.appendChild(input);
      actions.appendChild(
        checkButton("Continuar", "checklist_empezar", ciclos >= 10000)
      );
      revision.appendChild(hint);
      revision.appendChild(field);
      revision.appendChild(liveLine);
      revision.appendChild(actions);
      body.appendChild(revision);
      return;
    }
    if (paso === 2) {
      hint.textContent = "Paso 2 de 6. Prueba de aire: se accionan holder y encoder y se leen las fallas del PLC.";
      actions.appendChild(checkButton("Probar aire", "checklist_aire"));
    } else if (paso === 3) {
      hint.textContent = "Paso 3 de 6. Revisa a ojo que el material no quede ni flojo ni tenso en ambos lados.";
      actions.appendChild(checkButton("Confirmo holgura", "checklist_holgura"));
    } else if (paso === 4) {
      hint.textContent = "Paso 4 de 6. Primero la referencia mecánica del eje. Después el recorrido y la confirmación de que regresó.";
      actions.appendChild(checkButton("Llevar a referencia", "checklist_home", !!state.rutina));
      actions.appendChild(checkButton("Hacer recorrido", "checklist_recorrido", !ch.eje_ok || !!state.rutina));
      actions.appendChild(checkButton("Regresó bien", "checklist_eje_listo", !ch.eje_ok || !ch.recorrido_ok));
    } else if (paso === 5) {
      hint.textContent =
        "Paso 5 de 6. Prepara los alimentadores CAN y, si hay COM del PreAlimentador, estabilízalo. Luego la purga. Alimentar, Expulsar, Continuar y Terminar quedan en el banner superior.";
      actions.appendChild(checkButton("Preparar alimentador", "checklist_preparar", !!state.rutina));
      actions.appendChild(checkButton("Hacer purga", "checklist_purga", !ch.alimentador_ok || !!state.rutina));
      actions.appendChild(checkButton("Purga lista", "checklist_purga_hecha", !ch.purga_ok));
    } else {
      hint.textContent = "Paso 6 de 6. Corta dos piezas del modelo cargado. Cada lado debe medir la longitud del modelo, dentro de la tolerancia del ciclo. La LM Guide recorre esa longitud menos 55 mm.";
      if (!ch.piezas_cortadas) {
        actions.appendChild(checkButton("Cortar 2 piezas", "checklist_piezas", !!state.rutina));
      } else {
        const left = document.createElement("label");
        left.className = "modal-field";
        left.textContent = "Longitud medida izquierda (mm)";
        const inL = document.createElement("input");
        inL.id = "checkMmL";
        inL.inputMode = "decimal";
        left.appendChild(inL);
        const right = document.createElement("label");
        right.className = "modal-field";
        right.textContent = "Longitud medida derecha (mm)";
        const inR = document.createElement("input");
        inR.id = "checkMmR";
        inR.inputMode = "decimal";
        right.appendChild(inR);
        revision.appendChild(hint);
        revision.appendChild(left);
        revision.appendChild(right);
        revision.appendChild(liveLine);
        actions.appendChild(checkButton("Guardar medición", "checklist_medicion"));
        revision.appendChild(actions);
        body.appendChild(revision);
        return;
      }
    }
    revision.appendChild(hint);
    revision.appendChild(liveLine);
    revision.appendChild(actions);
    body.appendChild(revision);
  }

  function openChecklist() {
    checklistViewKey = "";
    if (els.checklistModal) els.checklistModal.hidden = false;
    fetchState();
  }

  function closeChecklist() {
    if (els.checklistModal) els.checklistModal.hidden = true;
    checklistViewKey = "";
  }

  function openMantenimiento() {
    if (els.mantModal) els.mantModal.hidden = false;
    showMantResetForm(false);
    fetchState();
  }

  function closeMantenimiento() {
    // Cerrar/cancelar no ejecuta reset ni modifica el contador.
    showMantResetForm(false);
    if (els.mantModal) els.mantModal.hidden = true;
  }

  function askEmpleado(done) {
    empleadoWait = done;
    if (!els.empleadoModal) return;
    if (els.empleadoInput) els.empleadoInput.value = "";
    els.empleadoModal.hidden = false;
    // Tras Skip: quedar encima de Recuperación y enfocar el campo.
    requestAnimationFrame(function () {
      if (els.empleadoInput && els.empleadoModal && !els.empleadoModal.hidden) {
        els.empleadoInput.focus();
        try {
          els.empleadoInput.select();
        } catch (e) {}
      }
    });
  }

  function closeEmpleado() {
    // Cerrar con X cancela el pedido de empleado; no ejecuta Skip.
    empleadoWait = null;
    if (els.empleadoModal) els.empleadoModal.hidden = true;
  }

  function confirmEmpleado() {
    const empleado = els.empleadoInput ? els.empleadoInput.value.trim() : "";
    if (!/^\d{5}$/.test(empleado)) {
      pushLocalEvent("El número de empleado son cinco dígitos.", "error");
      return;
    }
    const done = empleadoWait;
    empleadoWait = null;
    if (els.empleadoModal) els.empleadoModal.hidden = true;
    if (done) done(empleado);
  }

  if (els.statusBannerActions) {
    els.statusBannerActions.addEventListener("click", function (e) {
      const btn = e.target.closest("[data-action]");
      if (!btn || btn.disabled) return;
      flashButton(btn);
      const extra = {};
      const side = String(btn.dataset.side || "").toUpperCase();
      if (side === "R" || side === "L") extra.side = side;
      sendAction("operacion_manual", btn.dataset.action, extra);
    });
  }

  document.querySelectorAll("[data-andon-call]").forEach(function (btn) {
    btn.addEventListener("click", function () {
      const call = btn.getAttribute("data-andon-call");
      if (!call) return;
      flashButton(btn);
      sendAction("andon", call);
    });
  });

  document.querySelectorAll("[data-pf-hold]").forEach(function (btn) {
    const side = btn.getAttribute("data-pf-hold");
    if (side !== "L" && side !== "R") return;
    let held = false;
    async function setForce(on) {
      if (held === on) return;
      held = on;
      setButtonOn(btn, on);
      try {
        await fetch("/api/prefeeder/cmd", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ op: "set_force_feed", side: side, on: on }),
        });
      } catch (err) {
        /* sin servidor */
      }
    }
    btn.addEventListener("pointerdown", function (ev) {
      ev.preventDefault();
      try {
        btn.setPointerCapture(ev.pointerId);
      } catch (e) {
        /* ignore */
      }
      setForce(true);
    });
    function release() {
      setForce(false);
    }
    btn.addEventListener("pointerup", release);
    btn.addEventListener("pointercancel", release);
    btn.addEventListener("lostpointercapture", release);
  });

  document.querySelectorAll("[data-block]").forEach(function (container) {
    const block = container.dataset.block;
    container.querySelectorAll("[data-action]").forEach(function (btn) {
      // Evita doble handler si hay data-block anidados (p. ej. purga dentro de proceso).
      if (btn.closest("[data-block]") !== container) return;
      btn.addEventListener("click", function () {
        if (btn.hasAttribute("data-pf-hold")) return;
        const action = btn.dataset.action;
        flashButton(btn);

        if (block === "configuracion" && action === "parametros") {
          sendAction(block, action);
          openParametros();
          return;
        }
        if (block === "configuracion" && action === "checklist") {
          openChecklist();
          closeSettings();
          return;
        }
        if (block === "configuracion" && action === "mantenimiento") {
          openMantenimiento();
          closeSettings();
          return;
        }
        sendAction(block, action);

        if (block === "configuracion") {
          closeSettings();
        }
      });
    });
  });

  loadThemePreference();
  loadEventsPreference();
  restorePlcBlowerMs();
  loadModelos()
    .then(fetchState)
    .then(function (state) {
      syncUiFromPersistedState(state);
      return maybeRestoreModelFromLocal();
    })
    .then(loadCycleConfig);
})();
