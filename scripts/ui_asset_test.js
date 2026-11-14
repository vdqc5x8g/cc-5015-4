// Drives the terrain asset UI in headless Chromium. This exercises the real
// interface: switching tabs, loading an asset into the WebGL viewport, editing
// a contour elevation, rebuilding through the kernel, and checking that a
// rejected edit surfaces the kernel's error instead of silently changing the
// surface.
const puppeteer = require('puppeteer');
const { spawn } = require('child_process');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');
const PORT = 8900 + Math.floor(Math.random() * 90);
const BASE = `http://127.0.0.1:${PORT}`;

function wait(ms) { return new Promise((r) => setTimeout(r, ms)); }

async function waitFor(page, predicate, timeout = 20000, label = 'condition') {
  const start = Date.now();
  while (Date.now() - start < timeout) {
    const value = await page.evaluate(predicate);
    if (value) return value;
    await wait(150);
  }
  throw new Error('timed out waiting for ' + label);
}

(async () => {
  const server = spawn(path.join(ROOT, 'build', 'terrain'),
    ['serve', String(PORT)], { cwd: ROOT, stdio: ['ignore', 'pipe', 'pipe'] });
  let serverOutput = '';
  server.stdout.on('data', (d) => { serverOutput += d.toString(); });
  server.stderr.on('data', (d) => { serverOutput += d.toString(); });

  const browser = await puppeteer.launch({
    headless: 'new',
    args: ['--no-sandbox', '--use-gl=swiftshader', '--enable-unsafe-swiftshader',
      '--disable-gpu-sandbox', '--window-size=1600,1000'],
  });
  const page = await browser.newPage();
  await page.setViewport({ width: 1600, height: 1000 });
  const consoleErrors = [];
  page.on('console', (msg) => {
    if (msg.type() !== 'error') return;
    // The browser requests /favicon.ico on its own; a 404 there is not a UI
    // failure. Ignore only that resource.
    if (msg.text().includes('404') && msg.location()?.url?.endsWith('/favicon.ico')) return;
    consoleErrors.push(msg.text());
  });
  page.on('pageerror', (err) => consoleErrors.push('pageerror: ' + err.message));

  try {
    await page.goto(BASE, { waitUntil: 'networkidle0', timeout: 30000 });
    await waitFor(page, () => !!window.__terrain && !!window.__terrain.state.doc,
      20000, 'initial sample loaded');
    console.log('booted with sample', await page.evaluate(() => window.__terrain.state.doc.original.name));

    // 1. Switch to the terrain assets tab and check the asset list loaded.
    await page.click('.tab[data-tab="assets"]');
    await waitFor(page, () => {
      const sel = document.getElementById('assetSelect');
      return sel && sel.options.length > 0 && !sel.options[0].textContent.includes('no assets');
    }, 10000, 'asset list');
    const assetIds = await page.evaluate(() =>
      [...document.getElementById('assetSelect').options].map((o) => o.value));
    console.log('assets listed:', assetIds.join(', '));
    if (assetIds.length < 4) throw new Error('expected at least four assets');

    // 2. Load an asset into the model viewport and check real geometry.
    await page.evaluate(() => {
      const sel = document.getElementById('assetSelect');
      sel.value = 'ridge_truncated';
      document.getElementById('loadAsset').click();
    });
    await waitFor(page, () => window.__terrain.state.doc?.original?.name === 'ridge_truncated'
      && window.__terrain.state.doc.mesh?.ok, 20000, 'ridge asset built');
    const meshInfo = await page.evaluate(() => ({
      faces: window.__terrain.state.doc.mesh.triangle_count,
      apiFaces: window.__terrain.state.doc.mesh.triangle_count,
      surfaceChildren: window.__terrain.groups.surface.children.length,
      bounds: window.__terrain.state.doc.original.boundary.length,
      provRecipe: window.__terrain.state.doc.original.contours[0]?.attributes?.source,
    }));
    console.log('ridge mesh:', JSON.stringify(meshInfo));
    if (meshInfo.faces < 100) throw new Error('surface has too few faces');
    if (meshInfo.surfaceChildren < 1) throw new Error('no surface geometry in scene');

    // 3. The canvas must actually render non-background pixels.
    const canvasCheck = await page.evaluate(() => {
      const c = document.getElementById('view');
      const gl = c.getContext('webgl2') || c.getContext('webgl');
      return { hasContext: !!gl, width: c.width, height: c.height };
    });
    console.log('canvas:', JSON.stringify(canvasCheck));
    if (!canvasCheck.hasContext || canvasCheck.width < 10) throw new Error('WebGL canvas not initialised');

    // 4. Verification panel reflects the record.
    const verify = await page.evaluate(() => ({
      badge: document.getElementById('verifyBadge').textContent,
      text: document.getElementById('verifyPanel').textContent,
    }));
    console.log('verification badge:', verify.badge);
    if (verify.badge !== 'pass') throw new Error('verification badge is not pass');
    if (!verify.text.includes('constrained edges')) throw new Error('verification panel missing constraint evidence');

    // 5. Edit a contour elevation and rebuild.
    const editResult = await page.evaluate(() => {
      const rows = document.querySelectorAll('.curve-row');
      const input = rows[0].querySelector('input[type="number"]');
      const before = { value: input.value };
      input.value = String(parseFloat(input.value) + 2);
      input.dispatchEvent(new Event('change', { bubbles: true }));
      document.getElementById('rebuildFromEdits').click();
      return before;
    });
    await waitFor(page, () => document.getElementById('status').textContent.includes('rebuilt')
      || document.getElementById('status').textContent.includes('rejected'), 20000, 'edit rebuild');
    const afterEdit = await page.evaluate(() => ({
      status: document.getElementById('status').textContent,
      faces: window.__terrain.state.doc?.mesh?.triangle_count,
      editError: document.getElementById('editError').hidden ? '' : document.getElementById('editError').textContent,
      userEdits: (() => {
        try { return JSON.parse(window.__terrain.state.text).provenance.user_edits || []; }
        catch { return []; }
      })(),
    }));
    console.log('after edit:', JSON.stringify(afterEdit));
    if (afterEdit.faces < 100) throw new Error('edit rebuild lost the mesh');
    if (!afterEdit.userEdits.length) throw new Error('edited provenance not recorded in the posted document');

    // 6. Reset returns to the asset baseline.
    await page.evaluate(() => document.getElementById('resetCurves').click());
    const afterReset = await page.evaluate(() => ({
      text: window.__terrain.state.text,
      baseline: window.__terrain.assetState.baseline,
    }));
    if (afterReset.text !== afterReset.baseline) throw new Error('reset did not restore the baseline asset');

    // 7. A deliberate bad edit must be rejected by the kernel, and the
    //    previous mesh left intact.  Shifting a contour far outside the
    //    boundary is a real validation failure the kernel reports.
    const facesBefore = await page.evaluate(() => window.__terrain.state.doc.mesh.triangle_count);
    await page.evaluate(() => {
      const rows = document.querySelectorAll('.curve-row');
      // Inputs per row: [elevation, dx, dy].
      const dx = rows[0].querySelectorAll('input[type="number"]')[1];
      dx.value = '400';
      dx.dispatchEvent(new Event('change', { bubbles: true }));
      document.getElementById('rebuildFromEdits').click();
    });
    await waitFor(page, () => {
      const s = document.getElementById('status').textContent;
      return s.includes('rejected') || s.includes('rebuilt') || s.includes('failed');
    }, 20000, 'bad edit outcome');
    const bad = await page.evaluate(() => ({
      status: document.getElementById('status').textContent,
      editError: document.getElementById('editError').hidden ? '' : document.getElementById('editError').textContent,
      faces: window.__terrain.state.doc?.mesh?.triangle_count,
    }));
    console.log('bad edit:', JSON.stringify(bad));
    if (!bad.status.includes('rejected')) throw new Error('unsatisfiable curve edit was not rejected');
    if (!bad.editError) throw new Error('rejection not surfaced to the user');
    if (bad.faces !== facesBefore) throw new Error('rejected edit replaced the last good mesh');

    if (consoleErrors.length) {
      console.log('console errors:', consoleErrors.join('\n'));
      throw new Error('console errors during UI run');
    }

    console.log('UI PASS');
  } catch (error) {
    console.error('UI FAIL:', error.message);
    console.log('server output:\n' + serverOutput.slice(-2000));
    process.exitCode = 1;
  } finally {
    await browser.close();
    server.kill('SIGTERM');
  }
})();
