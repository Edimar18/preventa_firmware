// Exercise the actual embedded AP JavaScript, including delayed HTTP responses.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../ESP32-MAIN-FINAL/PREVENTA_ESP32_Main/PREVENTA_ESP32_Main.ino'), 'utf8');
const start = source.indexOf('"<script>"');
const end = source.indexOf('"</script></body></html>"', start);
assert(start >= 0 && end > start);
const html = source.slice(start, end).split('\n').map(line => line.trim()).filter(line => line.startsWith('"')).map(line => JSON.parse(line)).join('');
const script = html.slice('<script>'.length);
assert(!source.includes("id='setup_photo'"), 'AP must not render a full-frame preview');
assert(!script.includes('/api/capture.jpg'), 'AP must not fetch a full frame as a model-input substitute');
const tick = () => new Promise(resolve => setImmediate(resolve));
async function settle() { await tick(); await tick(); }

function browser() {
  const elements = new Map(), requests = [], alerts = [];
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      dataset: {}, style: {}, parentElement: {hidden: true}, hidden: true,
      disabled: false, complete: true, naturalWidth: 96, innerText: '',
      removeAttribute(name) { delete this[name]; },
    });
    return elements.get(id);
  }
  const ctx = {
    document: {getElementById: element}, setInterval() {}, setTimeout() {},
    alert: message => alerts.push(message), confirm: () => true,
    URLSearchParams,
    fetch(url, options) {
      return new Promise((resolve, reject) => requests.push({url, options, resolve, reject}));
    },
  };
  vm.createContext(ctx); vm.runInContext(script, ctx);
  async function respond(request, data) {
    request.resolve({json: () => Promise.resolve(data)}); await settle();
  }
  const data = (revision, result = 'accepted', fire = 80) => ({
    mq7_ppm: 0, current_a: 0, temp_c: 25, humidity: 50, tier: 0,
    cam_link: true, baseline_samples: 30, baseline_target: 30,
    photo_revision: revision, capture_result: result, photo_ready: true, model_input_ready: true,
    preview_inference: {fire_pct: fire, no_fire_pct: 100 - fire, camera_state: 'HIGH', response_ms: 500},
    test_sms: 'idle', test_push: 'idle',
  });
  const loadImages = () => element('setup_model_input').onload?.();
  return {ctx, element, requests, alerts, respond, data, loadImages};
}

(async () => {
  const b = browser(), {ctx, element: el, requests: req, respond, data} = b;
  assert.equal(req[0].options.cache, 'no-store');
  await respond(req[0], data(7)); b.loadImages();
  assert.equal(el('setup_model_input').parentElement.hidden, false);
  ctx.updateTelemetry(); const staleBeforeClick = req.at(-1);
  ctx.captureImage(el('capture_button')); const post = req.at(-1);
  assert.equal(post.url, '/api/capture');
  assert.equal(el('capture_button').disabled, true);
  assert.equal(el('setup_model_input').parentElement.hidden, true);
  assert.equal(el('capture_inference').hidden, true);
  assert.equal(el('setup_model_input').src, undefined);
  const beforeDoubleClick = req.length;
  ctx.captureImage(el('capture_button'));
  assert.equal(req.length, beforeDoubleClick, 'double-click must not queue another capture');
  await respond(staleBeforeClick, data(7));
  assert.equal(el('setup_model_input').parentElement.hidden, true, 'old poll cannot redisplay old crop');
  ctx.updateTelemetry(); const duringPost = req.at(-1);
  await respond(duringPost, data(7));
  assert.equal(el('capture_button').disabled, true);
  assert.equal(el('capture_inference').hidden, true);
  await respond(post, {status: 'ok', previous_photo_revision: 7});
  const afterPost = req.at(-1);
  await respond(afterPost, data(7));
  assert.equal(el('setup_model_input').parentElement.hidden, true, 'prior revision must stay hidden even if marked accepted');
  ctx.updateTelemetry(); await respond(req.at(-1), data(7, 'pending'));
  assert.equal(el('capture_button').disabled, true);
  ctx.updateTelemetry(); const latePending = req.at(-1);
  ctx.updateTelemetry(); const fresh = req.at(-1);
  await respond(fresh, data(8, 'accepted', 75)); b.loadImages();
  assert.equal(el('capture_button').disabled, false);
  assert.equal(el('setup_model_input').src, '/api/model-input.bmp?v=8');
  assert.equal(el('setup_model_input').parentElement.hidden, false);
  assert.match(el('capture_inference').innerText, /Fire 75%/);
  assert.match(el('capture_stat').innerText, /Capture #8/);
  await respond(latePending, data(7, 'pending'));
  assert.equal(el('setup_model_input').parentElement.hidden, false, 'out-of-order poll cannot replace the new result');
  assert.equal(el('capture_button').disabled, false);

  ctx.captureImage(el('capture_button')); const secondPost = req.at(-1);
  await respond(secondPost, {status: 'ok', previous_photo_revision: 8});
  await respond(req.at(-1), data(8, 'failed'));
  assert.equal(el('setup_model_input').parentElement.hidden, true);
  assert.equal(el('capture_inference').hidden, true);
  assert.match(el('capture_stat').innerText, /failed/i);
  assert.equal(el('capture_button').disabled, false);

  ctx.captureImage(el('capture_button')); req.at(-1).reject(new Error('offline')); await settle();
  assert.equal(el('capture_button').disabled, false);
  assert.match(el('capture_stat').innerText, /retry/i);

  const missing = browser();
  await missing.respond(missing.requests[0], {...missing.data(12), model_input_ready: false, photo_ready: true});
  assert.equal(missing.element('setup_model_input').parentElement.hidden, true);
  assert.equal(missing.element('setup_model_input').src, undefined, 'a full photograph must never substitute for a missing crop');
  assert.match(missing.element('capture_stat').innerText, /model input preview unavailable/i);
  console.log('AP crop-only preview checks passed: stale poll, in-flight POST, old revision, double click, fresh crop/score pairing, out-of-order poll, failure, retry and missing-crop handling.');
})().catch(error => { console.error(error); process.exitCode = 1; });
