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
  const elements = new Map(), requests = [], alerts = [], timers = new Map();
  let timerId = 0, now = 1000;
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      dataset: {}, style: {}, parentElement: {hidden: true}, hidden: true,
      disabled: false, complete: true, naturalWidth: 96, innerText: '',
      removeAttribute(name) { delete this[name]; },
    });
    return elements.get(id);
  }
  const ctx = {
    document: {getElementById: element}, setInterval() {},
    setTimeout(fn, delay) { const id = ++timerId; timers.set(id, {fn, delay}); return id; },
    clearTimeout(id) { timers.delete(id); }, AbortController,
    Date: class extends Date { static now() { return now; } },
    alert: message => alerts.push(message), confirm: () => true,
    URLSearchParams,
    fetch(url, options) {
      return new Promise((resolve, reject) => {
        requests.push({url, options, resolve, reject});
        options?.signal?.addEventListener('abort', () => reject(new Error('aborted')));
      });
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
  const advance = ms => { now += ms; };
  const expireRequests = () => { for (const [id, timer] of [...timers]) if (timer.delay === 8000) {timers.delete(id);timer.fn();} };
  return {ctx, element, requests, alerts, respond, data, loadImages, advance, expireRequests};
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

  const brokenSensor = browser();
  await brokenSensor.respond(brokenSensor.requests[0], {...brokenSensor.data(2), temp_c: null, mq7_ppm: null});
  assert.equal(brokenSensor.element('v_temp').innerText, '--');
  assert.match(brokenSensor.element('capture_stat').innerText, /Capture #2/);

  const stalled = browser();
  await stalled.respond(stalled.requests[0], stalled.data(0, 'idle'));
  stalled.ctx.captureImage(stalled.element('capture_button'));
  stalled.expireRequests(); await settle();
  assert.equal(stalled.element('capture_button').disabled, false, 'a hung POST must release the button');
  assert.match(stalled.element('capture_stat').innerText, /retry/i);

  const waiting = browser();
  await waiting.respond(waiting.requests[0], waiting.data(0, 'idle'));
  waiting.ctx.captureImage(waiting.element('capture_button'));
  await waiting.respond(waiting.requests.at(-1), {status:'ok',previous_photo_revision:0});
  await waiting.respond(waiting.requests.at(-1), waiting.data(0,'pending'));
  waiting.advance(51000); waiting.ctx.updateTelemetry();
  await waiting.respond(waiting.requests.at(-1), waiting.data(0,'pending'));
  assert.equal(waiting.element('capture_button').disabled, false);
  assert.match(waiting.element('capture_stat').innerText, /timed out/i);

  const reset = browser();
  await reset.respond(reset.requests[0], reset.data(4));
  reset.ctx.captureImage(reset.element('capture_button')); const canceledPost = reset.requests.at(-1);
  reset.ctx.recalibrateCamera(reset.element('camera_calibrate_button'));
  const resetPost = reset.requests.at(-1);
  assert.equal(resetPost.url, '/api/camera/recalibrate');
  assert.equal(resetPost.options.headers['X-Preventa-Setup'], '1');
  await reset.respond(canceledPost, {status:'ok',previous_photo_revision:4});
  assert.equal(reset.requests.at(-1),resetPost,'canceled capture response must not interfere with reset');
  await reset.respond(resetPost, {status:'ok'});
  await reset.respond(reset.requests.at(-1), {...reset.data(4,'idle'),camera_calibration_result:'pending',camera_calibration_message:'Restarting sensor'});
  assert.equal(reset.element('capture_button').disabled,true);
  assert.equal(reset.element('camera_calibrate_button').disabled,true);
  reset.ctx.updateTelemetry();
  await reset.respond(reset.requests.at(-1), {...reset.data(4,'idle'),camera_calibration_result:'accepted',camera_calibration_message:'Camera recalibrated'});
  assert.equal(reset.element('capture_button').disabled,false);
  assert.equal(reset.element('camera_calibrate_button').disabled,false);
  assert.match(reset.element('camera_calibrate_stat').innerText,/recalibrated/);
  console.log('AP preview checks passed: fresh crop/score pairing, stale responses, null telemetry, request timeouts, retry, and camera recalibration.');
})().catch(error => { console.error(error); process.exitCode = 1; });
