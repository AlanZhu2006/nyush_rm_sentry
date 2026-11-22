import type { ExtensionContext, PanelExtensionContext } from "@foxglove/extension";
import { ros2humble } from "@foxglove/rosmsg-msgs-common";

import { panelStyle } from "./style";

const topics = {
  command: "/sentry/operator/command",
  drive: "/sentry/operator/drive",
  status: "/sentry/operator/status",
  log: "/sentry/operator/log",
};
const stringSchema = "std_msgs/msg/String";
const stringDefinition = ros2humble["std_msgs/String"];
const datatypes = new Map([
  [stringSchema, stringDefinition],
  ["std_msgs/String", stringDefinition],
]);
const staleAfterMs = 1500;
const keepaliveMs = 250;

type Action =
  | "map_start"
  | "map_save"
  | "nav_start"
  | "goal_send"
  | "nav_cancel"
  | "stop"
  | "discard"
  | "manual_start"
  | "manual_stop"
  | "logger_start"
  | "logger_stop";
type MoveKey = "KeyW" | "KeyA" | "KeyS" | "KeyD" | "KeyQ" | "KeyE";
type Receipt = { id: string; ok: boolean; message: string };
type Status = {
  mode: "idle" | "map" | "nav";
  phase: string;
  map: string | null;
  maps: string[];
  goal: string | null;
  goalTarget: string | null;
  manual: boolean;
  manualOwner: string | null;
  logger: boolean;
  loggerAttached: boolean;
  serial: boolean;
  busy: boolean;
  message: string;
  receipts: Receipt[];
};
type Drive = { forward: number; left: number; turn: number };
const zeroDrive: Drive = { forward: 0, left: 0, turn: 0 };
const moveKeys = new Set<string>(["KeyW", "KeyA", "KeyS", "KeyD", "KeyQ", "KeyE"]);
const actionLabels: Record<Action, string> = {
  map_start: "启动建图",
  map_save: "保存地图",
  nav_start: "启动导航",
  goal_send: "执行目标",
  nav_cancel: "取消目标",
  stop: "停止任务",
  discard: "停止但不保存",
  manual_start: "取得手动控制",
  manual_stop: "释放手动控制",
  logger_start: "启动 RTT 日志",
  logger_stop: "停止 RTT 日志",
};
const modeLabels = { idle: "空闲", map: "建图", nav: "导航" };
const phaseLabels: Record<string, string> = {
  starting: "启动中", ready: "就绪", saving: "保存中", stopping: "停止中",
  fault: "故障", idle: "空闲", external: "由其他终端启动",
};

function record(value: unknown): Record<string, unknown> | undefined {
  return typeof value === "object" && value !== null
    ? (value as Record<string, unknown>)
    : undefined;
}

function jsonMessage(message: unknown): Record<string, unknown> | undefined {
  const data = record(message)?.data;
  if (typeof data !== "string") {
    return undefined;
  }
  try {
    return record(JSON.parse(data));
  } catch {
    return undefined;
  }
}

function asText(value: unknown): string | null {
  return typeof value === "string" && value.length > 0 ? value : null;
}

function readStatus(message: Record<string, unknown>): Status | undefined {
  const { mode, phase, maps, manual, logger, serial } = message;
  if (
    (mode !== "idle" && mode !== "map" && mode !== "nav") ||
    typeof phase !== "string" ||
    !Array.isArray(maps) ||
    typeof manual !== "boolean" ||
    typeof logger !== "boolean" ||
    typeof serial !== "boolean"
  ) {
    return undefined;
  }
  const receipts: Receipt[] = [];
  for (const entry of Array.isArray(message.receipts) ? message.receipts : []) {
    const receipt = record(entry);
    if (
      typeof receipt?.id === "string" &&
      typeof receipt.ok === "boolean" &&
      typeof receipt.message === "string"
    ) {
      receipts.push({ id: receipt.id, ok: receipt.ok, message: receipt.message });
    }
  }
  return {
    mode,
    phase,
    map: asText(message.map),
    maps: maps.filter((name): name is string => typeof name === "string"),
    goal: asText(message.goal),
    goalTarget: asText(message.goal_target),
    manual,
    manualOwner: asText(message.manual_owner),
    logger,
    loggerAttached: message.logger_attached === true,
    serial,
    busy: message.busy === true,
    message: asText(message.message) ?? "",
    receipts,
  };
}

function identifier(): string {
  return Array.from(globalThis.crypto.getRandomValues(new Uint8Array(16)))
    .map((byte) => byte.toString(16).padStart(2, "0"))
    .join("");
}

function initPanel(context: PanelExtensionContext): () => void {
  const owner = identifier();
  const root = document.createElement("main");
  root.className = "nyush-operator";
  const style = document.createElement("style");
  style.textContent = panelStyle;
  root.innerHTML = `
    <header>
      <h1>NYUSH Sentry 操作台</h1>
      <span class="health" data-ref="health" role="status">等待操作服务</span>
    </header>
    <p class="muted message" data-ref="state"></p>
    <p class="receipt message" data-ref="receipt" role="status"></p>
    <section>
      <h2>建图与导航</h2>
      <div class="map-actions">
        <div class="row">
          <label>新地图名称 <input data-ref="new-map" placeholder="例如 arena_run" maxlength="64" autocomplete="off"></label>
          <button data-ref="map-start" type="button">启动建图</button>
          <button data-ref="map-save" type="button">保存地图</button>
        </div>
        <div class="row">
          <label>已有地图 <select data-ref="maps"></select></label>
          <button data-ref="nav-start" type="button">启动导航</button>
          <button data-ref="stop" data-stop type="button">停止任务</button>
          <button data-ref="discard" data-stop type="button" aria-expanded="false" hidden>停止但不保存</button>
        </div>
        <div class="row" data-ref="discard-confirmation" hidden>
          <span>确认停止建图并放弃本次未保存的数据？</span>
          <button data-ref="discard-confirm" data-stop type="button">确认放弃</button>
          <button data-ref="discard-cancel" type="button">继续建图</button>
        </div>
        <div class="row">
          <button data-ref="goal-send" type="button">执行目标</button>
          <button data-ref="nav-cancel" type="button">取消目标</button>
          <span class="muted" data-ref="goal"></span>
        </div>
      </div>
    </section>
    <section>
      <h2>手动底盘</h2>
      <div class="row">
        <button data-ref="manual-start" type="button">取得控制</button>
        <button data-ref="manual-stop" data-release type="button">停止并释放</button>
        <span class="muted" data-ref="manual-status"></span>
      </div>
      <div class="drive">
        <div class="keys" role="group" aria-label="按住方向键移动">
          <button data-key="KeyQ" type="button" aria-label="逆时针旋转">Q ↶</button>
          <button data-key="KeyW" type="button" aria-label="前进">W ↑</button>
          <button data-key="KeyE" type="button" aria-label="顺时针旋转">E ↷</button>
          <button data-key="KeyA" type="button" aria-label="向左平移">A ←</button>
          <button data-key="KeyS" type="button" aria-label="后退">S ↓</button>
          <button data-key="KeyD" type="button" aria-label="向右平移">D →</button>
        </div>
        <div class="keyboard-help">
          <button data-ref="keyboard" type="button">键盘控制</button>
          <p class="muted message">点按后按住空格与 WASD 或 QE。松手即停。</p>
        </div>
      </div>
    </section>
    <section>
      <h2>进程日志</h2>
      <div class="row">
        <button data-ref="logger-start" type="button">启动 RTT 日志</button>
        <button data-ref="logger-stop" type="button">停止 RTT 日志</button>
        <span class="muted" data-ref="logger-status"></span>
      </div>
      <pre data-ref="logs" aria-label="最近的进程日志"></pre>
    </section>
  `;
  context.panelElement.append(style, root);

  function element<T extends HTMLElement>(ref: string): T {
    return root.querySelector<T>(`[data-ref="${ref}"]`)!;
  }

  const stateElement = element("state");
  const healthElement = element("health");
  const receiptElement = element("receipt");
  const goalElement = element("goal");
  const manualStatus = element("manual-status");
  const logElement = element<HTMLPreElement>("logs");
  const mapInput = element<HTMLInputElement>("new-map");
  const mapSelect = element<HTMLSelectElement>("maps");
  const mapStart = element<HTMLButtonElement>("map-start");
  const mapSave = element<HTMLButtonElement>("map-save");
  const navStart = element<HTMLButtonElement>("nav-start");
  const stop = element<HTMLButtonElement>("stop");
  const discard = element<HTMLButtonElement>("discard");
  const discardConfirmation = element("discard-confirmation");
  const discardConfirm = element<HTMLButtonElement>("discard-confirm");
  const discardCancel = element<HTMLButtonElement>("discard-cancel");
  const goalSend = element<HTMLButtonElement>("goal-send");
  const navCancel = element<HTMLButtonElement>("nav-cancel");
  const manualStart = element<HTMLButtonElement>("manual-start");
  const manualStop = element<HTMLButtonElement>("manual-stop");
  const keyboard = element<HTMLButtonElement>("keyboard");
  const loggerStart = element<HTMLButtonElement>("logger-start");
  const loggerStop = element<HTMLButtonElement>("logger-stop");
  const loggerStatus = element("logger-status");
  const movement = Array.from(root.querySelectorAll<HTMLButtonElement>("[data-key]"));
  const keys = new Set<string>();
  const pointers = new Map<number, MoveKey>();
  const pending = new Map<string, { action: Action; deadline: number }>();
  const logLines: string[] = [];
  let status: Status | undefined;
  let lastStatusAt = -Infinity;
  let mapsKey = "";
  let advertised = false;
  let space = false;
  let moving = false;
  let sequence = 0;
  let lastDriveAt = -Infinity;
  let manualRequested = false;
  let confirmDiscard = false;

  const fresh = () => performance.now() - lastStatusAt <= staleAfterMs;
  const visible = () => document.visibilityState !== "hidden";
  const ownsManual = () => status?.manual === true && status.manualOwner === owner;
  const canPublish = () =>
    context.dataSourceProfile === "ros2" &&
    context.advertise !== undefined && context.publish !== undefined;
  const keepaliveAllowed = () => fresh() && canPublish() && ownsManual() && visible();

  function report(message: string, error = false): void {
    receiptElement.textContent = message;
    receiptElement.dataset.error = String(error);
  }

  function publishers(): boolean {
    if (!canPublish()) {
      return false;
    }
    if (!advertised) {
      const options = { datatypes };
      context.advertise?.(topics.command, stringSchema, options);
      context.advertise?.(topics.drive, stringSchema, options);
      advertised = true;
    }
    return true;
  }

  function write(topic: string, value: object): boolean {
    try {
      if (!publishers()) {
        return false;
      }
      context.publish?.(topic, { data: JSON.stringify(value) });
      return true;
    } catch (error) {
      report(`发送失败：${String(error)}`, true);
      return false;
    }
  }

  function sendCommand(action: Action, name?: string, confirmHeartbeat = true): void {
    if (confirmHeartbeat && !fresh()) {
      return;
    }
    const id = identifier();
    const payload = { id, action, name, owner: undefined as string | undefined };
    if (action === "manual_start" || action === "manual_stop") {
      payload.owner = owner;
    }
    if (write(topics.command, payload)) {
      const timeout = action === "map_save" || action === "stop" ? 130_000 : 35_000;
      pending.set(id, { action, deadline: performance.now() + timeout });
      if (action === "manual_start") {
        manualRequested = true;
      }
      report(`${actionLabels[action]} · 等待确认`);
    }
  }

  function sendDrive(drive: Drive): boolean {
    if (write(topics.drive, { owner, seq: ++sequence, ...drive })) {
      lastDriveAt = performance.now();
      return true;
    }
    return false;
  }

  function clearInput(): void {
    space = false;
    keys.clear();
    pointers.clear();
  }

  function release(): void {
    clearInput();
    if (ownsManual() && advertised) {
      sendDrive(zeroDrive);
    }
    moving = false;
  }

  function held(key: MoveKey): boolean {
    return (space && keys.has(key)) || Array.from(pointers.values()).includes(key);
  }

  function driveAllowed(): boolean {
    return keepaliveAllowed() && status?.serial === true && document.hasFocus();
  }

  function sendCurrentDrive(): void {
    if (!driveAllowed()) {
      if (moving || space || pointers.size > 0) {
        release();
      }
      if (keepaliveAllowed() && performance.now() - lastDriveAt >= keepaliveMs) {
        sendDrive(zeroDrive);
      }
      return;
    }
    const forward = Number(held("KeyW")) - Number(held("KeyS"));
    const left = Number(held("KeyA")) - Number(held("KeyD"));
    const turn = Number(held("KeyQ")) - Number(held("KeyE"));
    if (forward === 0 && left === 0 && turn === 0) {
      if (moving || performance.now() - lastDriveAt >= keepaliveMs) {
        sendDrive(zeroDrive);
        moving = false;
      }
      return;
    }
    if (sendDrive({ forward, left, turn })) {
      moving = true;
    } else {
      clearInput();
      moving = false;
    }
  }

  function draw(): void {
    const online = fresh();
    const writable = online && canPublish() && advertised;
    const state = status;
    const phase = state?.phase;
    const idle = state?.mode === "idle" && (phase === "idle" || phase === "ready" || phase === "fault");
    const mapping = state?.mode === "map";
    const navigating = state?.mode === "nav";
    const manual = state?.manual === true;
    const serial = state?.serial === true;
    const owned = ownsManual();
    const processReady = writable && !state?.busy;
    const mapName = mapInput.value.trim();
    const validMap = /^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$/.test(mapName);
    healthElement.dataset.online = String(writable);
    healthElement.textContent = !online
      ? (state ? "操作服务心跳已过期" : "等待操作服务")
      : (writable ? "操作服务已连接" : "需要可写的 ROS 2 实时连接");
    stateElement.textContent = state
      ? [modeLabels[state.mode], phaseLabels[state.phase] ?? state.phase,
        state.map, serial ? "底盘串口在线" : "底盘串口离线", state.message]
        .filter(Boolean).join(" · ")
      : "";
    const list = state?.maps ?? [];
    const newKey = JSON.stringify(list);
    if (mapsKey !== newKey) {
      const selected = mapSelect.value;
      mapsKey = newKey;
      mapSelect.replaceChildren(...list.map((name) => {
        const option = document.createElement("option");
        option.value = option.textContent = name;
        return option;
      }));
      if (list.includes(selected)) {
        mapSelect.value = selected;
      }
    }
    goalElement.textContent = state?.goalTarget
      ? `待执行 · ${state.goalTarget}`
      : (state?.goal ?? (navigating ? "用 3D 面板的 2D Pose 选取目标" : ""));
    manualStatus.textContent = owned ? "本面板持有控制" : (manual ? "控制已被占用" : "待取得控制");
    mapStart.disabled = !processReady || !idle || manual || !validMap;
    mapSave.disabled = !processReady || !mapping || phase !== "ready";
    navStart.disabled = !processReady || !idle || manual || !mapSelect.value;
    const canEndSession = processReady && phase !== "external" && phase !== "stopping" &&
      phase !== "saving";
    stop.disabled = !canEndSession || !(navigating || (mapping && phase === "ready"));
    stop.textContent = mapping ? "保存并停止建图" : "停止导航";
    discard.hidden = !mapping;
    discard.disabled = !mapping || !canEndSession;
    if (discard.disabled) {
      confirmDiscard = false;
    }
    discard.setAttribute("aria-expanded", String(confirmDiscard));
    discardConfirmation.hidden = !confirmDiscard;
    discardConfirm.disabled = discard.disabled;
    goalSend.disabled = !processReady || !navigating || phase !== "ready" ||
      !serial || manual || !state?.goalTarget;
    navCancel.disabled = !writable || !navigating;
    const manualAvailable = idle || (serial && (phase === "starting" || phase === "ready"));
    manualStart.disabled = !processReady || manual || !manualAvailable;
    manualStop.disabled = !writable || !owned;
    const allowMovement = writable && owned && serial;
    keyboard.disabled = !allowMovement;
    for (const button of movement) {
      button.disabled = !allowMovement;
      button.dataset.held = String(held(button.dataset.key as MoveKey));
    }
    loggerStart.disabled = !processReady || state?.logger === true || state?.loggerAttached === true;
    loggerStop.disabled = !processReady || state?.logger !== true;
    loggerStatus.textContent = state?.loggerAttached ? "RTT 已连接" : "";
  }

  function tick(): void {
    sendCurrentDrive();
    for (const [id, command] of pending) {
      if (performance.now() > command.deadline) {
        pending.delete(id);
        report(`${actionLabels[command.action]} · 未收到确认`, true);
      }
    }
    draw();
  }

  function acceptStatus(message: Status): void {
    const wasOwner = ownsManual();
    status = message;
    lastStatusAt = performance.now();
    if (!advertised && canPublish()) {
      try {
        publishers();
      } catch (error) {
        report(`无法创建操作话题：${String(error)}`, true);
      }
    }
    if (wasOwner && !ownsManual()) {
      clearInput();
      moving = false;
      manualRequested = false;
    }
    for (const receipt of message.receipts) {
      const command = pending.get(receipt.id);
      if (command) {
        pending.delete(receipt.id);
        if ((command.action === "manual_start" && !receipt.ok) ||
          (command.action === "manual_stop" && receipt.ok)) {
          manualRequested = false;
        }
        report(`${actionLabels[command.action]} · ${receipt.message}`, !receipt.ok);
      }
    }
  }

  mapInput.addEventListener("input", draw);
  mapSelect.addEventListener("change", draw);
  mapStart.addEventListener("click", () => sendCommand("map_start", mapInput.value.trim()));
  mapSave.addEventListener("click", () => sendCommand("map_save"));
  navStart.addEventListener("click", () => sendCommand("nav_start", mapSelect.value));
  stop.addEventListener("click", () => {
    release();
    sendCommand("stop");
  });
  discard.addEventListener("click", () => {
    confirmDiscard = !confirmDiscard;
    draw();
    if (confirmDiscard) {
      discardCancel.focus();
    }
  });
  discardConfirm.addEventListener("click", () => {
    if (confirmDiscard) {
      confirmDiscard = false;
      release();
      sendCommand("discard");
      draw();
    }
  });
  discardCancel.addEventListener("click", () => {
    confirmDiscard = false;
    draw();
    discard.focus();
  });
  goalSend.addEventListener("click", () => sendCommand("goal_send"));
  navCancel.addEventListener("click", () => sendCommand("nav_cancel"));
  manualStart.addEventListener("click", () => sendCommand("manual_start"));
  manualStop.addEventListener("click", () => {
    release();
    sendCommand("manual_stop");
  });
  loggerStart.addEventListener("click", () => sendCommand("logger_start"));
  loggerStop.addEventListener("click", () => sendCommand("logger_stop"));

  for (const button of movement) {
    button.addEventListener("pointerdown", (event) => {
      if (event.button !== 0 || !driveAllowed()) {
        return;
      }
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      pointers.set(event.pointerId, button.dataset.key as MoveKey);
      tick();
    });
    const endPointer = (event: PointerEvent) => {
      if (pointers.delete(event.pointerId)) {
        tick();
      }
    };
    button.addEventListener("pointerup", endPointer);
    button.addEventListener("pointercancel", endPointer);
    button.addEventListener("lostpointercapture", endPointer);
  }

  keyboard.addEventListener("keydown", (event) => {
    if ((!moveKeys.has(event.code) && event.code !== "Space") ||
      event.altKey || event.ctrlKey || event.metaKey) {
      return;
    }
    event.preventDefault();
    event.stopPropagation();
    if (event.repeat || !driveAllowed()) {
      return;
    }
    if (event.code === "Space") {
      space = true;
    } else {
      keys.add(event.code);
    }
    tick();
  });
  keyboard.addEventListener("keyup", (event) => {
    if (!moveKeys.has(event.code) && event.code !== "Space") {
      return;
    }
    event.preventDefault();
    event.stopPropagation();
    if (event.code === "Space") {
      release();
    } else {
      keys.delete(event.code);
    }
    tick();
  });
  keyboard.addEventListener("blur", release);
  window.addEventListener("blur", release);
  const onVisibility = () => {
    if (!visible()) {
      release();
    }
  };
  document.addEventListener("visibilitychange", onVisibility);

  context.watch("currentFrame");
  context.subscribe([{ topic: topics.status }, { topic: topics.log }]);
  context.onRender = (renderState, done) => {
    try {
      let logged = false;
      for (const event of renderState.currentFrame ?? []) {
        const message = jsonMessage(event.message);
        if (!message) {
          continue;
        }
        if (event.topic === topics.status) {
          const incoming = readStatus(message);
          if (incoming) {
            acceptStatus(incoming);
          }
        } else if (event.topic === topics.log && typeof message.text === "string") {
          logLines.push(`[${asText(message.source) ?? "operator"}] ${message.text}`.slice(0, 500));
          if (logLines.length > 80) {
            logLines.shift();
          }
          logged = true;
        }
      }
      if (logged) {
        const follow = logElement.scrollHeight - logElement.scrollTop - logElement.clientHeight < 20;
        logElement.textContent = logLines.join("\n");
        if (follow) {
          logElement.scrollTop = logElement.scrollHeight;
        }
      }
      draw();
    } finally {
      done();
    }
  };
  const timer = window.setInterval(tick, 50);
  draw();

  return () => {
    window.clearInterval(timer);
    release();
    if (manualRequested || ownsManual()) {
      sendCommand("manual_stop", undefined, false);
    }
    window.removeEventListener("blur", release);
    document.removeEventListener("visibilitychange", onVisibility);
    context.onRender = undefined;
    context.unsubscribeAll();
    if (advertised) {
      context.unadvertise?.(topics.drive);
      context.unadvertise?.(topics.command);
    }
    root.remove();
    style.remove();
  };
}

export function activate(context: ExtensionContext): void {
  context.registerPanel({ name: "NYUSH Sentry 操作台", initPanel });
}
