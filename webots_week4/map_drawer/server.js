const http = require("http");
const fs = require("fs");
const path = require("path");

const root = __dirname;
const generatedDir = path.join(root, "..", "worlds", "generated");
const port = Number(process.env.PORT || 3100);

const mime = {
  ".html": "text/html; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".js": "application/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".txt": "text/plain; charset=utf-8",
  ".wbt": "text/plain; charset=utf-8",
  ".proto": "text/plain; charset=utf-8",
};

function send(res, status, body, contentType = "text/plain; charset=utf-8") {
  res.writeHead(status, { "Content-Type": contentType });
  res.end(body);
}

function safeFileName(name) {
  const base = path.basename(String(name || ""));
  if (!/^[a-zA-Z0-9_. -]+$/.test(base)) {
    throw new Error(`Unsafe file name: ${name}`);
  }
  return base;
}

function serveFile(req, res) {
  const url = new URL(req.url, `http://localhost:${port}`);
  const requested = url.pathname === "/" ? "/index.html" : url.pathname;
  const filePath = path.normalize(path.join(root, requested));
  if (!filePath.startsWith(root)) {
    send(res, 403, "Forbidden");
    return;
  }
  fs.readFile(filePath, (err, data) => {
    if (err) {
      send(res, 404, "Not found");
      return;
    }
    send(res, 200, data, mime[path.extname(filePath)] || "application/octet-stream");
  });
}

function saveGenerated(req, res) {
  let body = "";
  req.on("data", (chunk) => {
    body += chunk;
    if (body.length > 8_000_000) {
      req.destroy();
    }
  });
  req.on("end", () => {
    try {
      const payload = JSON.parse(body);
      const files = payload.files || {};
      fs.mkdirSync(generatedDir, { recursive: true });
      const written = [];
      for (const [name, content] of Object.entries(files)) {
        const safe = safeFileName(name);
        const target = path.join(generatedDir, safe);
        fs.writeFileSync(target, String(content), "utf8");
        written.push(safe);
      }
      send(res, 200, JSON.stringify({ ok: true, written }, null, 2), "application/json; charset=utf-8");
    } catch (error) {
      send(res, 400, JSON.stringify({ ok: false, error: error.message }), "application/json; charset=utf-8");
    }
  });
}

http.createServer((req, res) => {
  if (req.method === "POST" && req.url === "/save") {
    saveGenerated(req, res);
    return;
  }
  if (req.method === "GET") {
    serveFile(req, res);
    return;
  }
  send(res, 405, "Method not allowed");
}).listen(port, "127.0.0.1", () => {
  console.log(`MTRN3100 map drawer: http://127.0.0.1:${port}`);
  console.log(`Generated files: ${generatedDir}`);
});
