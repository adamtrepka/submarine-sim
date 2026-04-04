/**
 * Take a screenshot of the web simulator using Playwright.
 *
 * Usage (from repo root):
 *   node .github/scripts/screenshot-web.js
 *
 * Requires: npx playwright install --with-deps chromium
 */

const { chromium } = require("playwright");
const http = require("http");
const fs = require("fs");
const path = require("path");

const WEB_DIR = path.resolve(__dirname, "../../web");
const OUTPUT = path.resolve(__dirname, "../../assets/web.png");
const PORT = 8787;
const WAIT_MS = 4000; // let canvas animation render

// Minimal static file server
function startServer() {
  const MIME = {
    ".html": "text/html",
    ".css": "text/css",
    ".js": "application/javascript",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".svg": "image/svg+xml",
  };

  const server = http.createServer((req, res) => {
    const filePath = path.join(WEB_DIR, req.url === "/" ? "index.html" : req.url);
    const ext = path.extname(filePath);
    fs.readFile(filePath, (err, data) => {
      if (err) {
        res.writeHead(404);
        res.end("Not found");
        return;
      }
      res.writeHead(200, { "Content-Type": MIME[ext] || "application/octet-stream" });
      res.end(data);
    });
  });

  return new Promise((resolve) => {
    server.listen(PORT, () => resolve(server));
  });
}

async function main() {
  const server = await startServer();
  console.log(`Serving ${WEB_DIR} on http://localhost:${PORT}`);

  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });

  await page.goto(`http://localhost:${PORT}/`, { waitUntil: "load" });
  await page.waitForTimeout(WAIT_MS);

  fs.mkdirSync(path.dirname(OUTPUT), { recursive: true });
  await page.screenshot({ path: OUTPUT, fullPage: false });
  console.log(`Screenshot saved to ${OUTPUT}`);

  await browser.close();
  server.close();
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
