export const panelStyle = `
  .nyush-operator {
    box-sizing: border-box;
    min-height: 100%;
    padding: 14px;
    color-scheme: dark;
    color: #edf6ff;
    background: #0a1420;
    font: 13px/1.5 system-ui, sans-serif;
    --outline: #39526d;
    --muted: #a4b7ca;
    --ready: #50dfa0;
    --attention: #ffd07b;
  }
  .nyush-operator * { box-sizing: border-box; }
  .nyush-operator [hidden] { display: none !important; }
  .nyush-operator header, .nyush-operator .row {
    display: flex;
    align-items: center;
    gap: 8px;
    flex-wrap: wrap;
  }
  .nyush-operator header { justify-content: space-between; margin-bottom: 8px; }
  .nyush-operator h1 { font-size: 17px; margin: 0; }
  .nyush-operator h2 { font-size: 14px; margin: 0 0 8px; }
  .nyush-operator .muted { color: var(--muted); }
  .nyush-operator .health { color: var(--attention); }
  .nyush-operator .health[data-online="true"] { color: var(--ready); }
  .nyush-operator section {
    padding: 11px;
    border: 1px solid #2a4057;
    border-radius: 8px;
    background: #111f30;
    margin: 10px 0;
  }
  .nyush-operator label {
    min-width: 120px;
    max-width: 220px;
    display: grid;
    gap: 3px;
    flex: 1;
  }
  .nyush-operator button, .nyush-operator input, .nyush-operator select {
    border: 1px solid var(--outline);
    border-radius: 5px;
    color: inherit;
    background: #17293d;
    min-height: 30px;
    font: inherit;
    padding: 4px 8px;
  }
  .nyush-operator button { cursor: pointer; }
  .nyush-operator button:disabled { color: #8394a5; cursor: default; }
  .nyush-operator button:not(:disabled):hover { border-color: #7cabc8; }
  .nyush-operator :focus-visible { outline: 2px solid #77d6ff; outline-offset: 2px; }
  .nyush-operator button[data-stop], .nyush-operator button[data-release] {
    border-color: #82566a;
  }
  .nyush-operator .map-actions { display: grid; gap: 8px; }
  .nyush-operator .map-actions .row { align-items: end; }
  .nyush-operator .drive {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 10px;
    margin-top: 10px;
  }
  .nyush-operator .keys { display: grid; grid-template-columns: repeat(3, 44px); gap: 4px; }
  .nyush-operator .keys button { min-height: 35px; touch-action: none; user-select: none; }
  .nyush-operator .keys button[data-held="true"] {
    background: #224d60;
    border-color: #77d6ff;
  }
  .nyush-operator .keyboard-help { max-width: 22ch; }
  .nyush-operator .message {
    min-height: 20px;
    overflow-wrap: anywhere;
    margin: 5px 0 0;
  }
  .nyush-operator .receipt[data-error="true"] { color: var(--attention); }
  .nyush-operator pre {
    min-height: 48px;
    max-height: 160px;
    margin: 8px 0 0;
    overflow: auto;
    white-space: pre-wrap;
    overflow-wrap: anywhere;
    color: #c3d8e9;
    font: 11px/1.45 ui-monospace, monospace;
  }
`;
