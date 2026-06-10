// Browser lifecycle helpers driven from C# via [JSImport] (see BrowserHostInterop.cs).
//   setTitle(string)        — set the document/tab title.
//   setUnsavedGuard(bool)   — arm/disarm a beforeunload prompt so the browser warns
//                             the user about unsaved changes before closing/reloading.
// Loaded by Program.Main via JSHost.ImportAsync("browserhost", "../browserhost.js").

let hasUnsavedChanges = false;

// Single persistent beforeunload listener. The browser shows its own generic
// confirmation dialog whenever preventDefault() is called / returnValue is set;
// custom text is ignored by modern browsers, so none is supplied.
window.addEventListener('beforeunload', (e) => {
    if (hasUnsavedChanges) {
        e.preventDefault();
        e.returnValue = '';
        return '';
    }
});

// True only when the PRIMARY pointing device is coarse — i.e. a touchscreen is the main
// input (iPad, tablet, phone). We deliberately do NOT use 'ontouchstart' in window (always
// present in Chromium, even on mouse-only desktops) nor navigator.maxTouchPoints (>0 on many
// non-touch desktops) — those falsely showed the touch bar on non-touch screens. On hybrid
// devices (touch laptop with a mouse) the bar still appears on the first real touch, via the
// viewport's TouchDetected fallback.
export function isTouchDevice() {
    try {
        return !!(window.matchMedia && window.matchMedia('(pointer: coarse)').matches);
    } catch { return false; }
}

export function setTitle(title) {
    if (typeof title === 'string' && title.length > 0) {
        document.title = title;
    }
}

export function setUnsavedGuard(unsaved) {
    hasUnsavedChanges = !!unsaved;
}

// Save a file by triggering a browser download (Blob + anchor click). Works in every
// browser — unlike StorageProvider.SaveFilePickerAsync, which needs the Chromium-only
// File System Access API and returns null elsewhere. `base64` is the file content.
export function downloadFile(fileName, base64) {
    const bin = atob(base64);
    const bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    const blob = new Blob([bytes], { type: 'application/octet-stream' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = fileName || 'project.cdb.json';
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 2000);
}
