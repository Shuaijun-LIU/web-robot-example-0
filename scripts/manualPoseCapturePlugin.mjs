import { mkdir, rename, writeFile } from 'node:fs/promises';
import path from 'node:path';

const ENDPOINT = '/__manual-pose-capture';
const MAX_REQUEST_BYTES = 1_000_000;

function sanitizePoseLabel(value) {
  const label = String(value ?? '').trim();
  if (label.length > 64) throw new Error('Pose names must not exceed 64 characters');
  if (!/^[A-Za-z0-9][A-Za-z0-9_-]*$/.test(label)) {
    throw new Error('Pose names may contain only letters, numbers, hyphens and underscores, and must start with a letter or number');
  }
  return label;
}

export async function persistManualPoseSnapshot(projectRoot, payload) {
  if (!payload || payload.scene !== 'frankaAssembly1') {
    throw new Error('Only Franka Assembly1 poses are accepted');
  }
  if (payload.schemaVersion !== 'franka-assembly1-manual-pose-v1') {
    throw new Error('Invalid pose schema version');
  }
  const label = sanitizePoseLabel(payload.label);
  const outputDirectory = path.resolve(projectRoot, 'artifacts', 'manual-poses');
  const outputPath = path.join(outputDirectory, `${label}.json`);
  const temporaryPath = path.join(
    outputDirectory,
    `.${label}.${process.pid}.${Date.now()}.tmp`,
  );
  await mkdir(outputDirectory, { recursive: true });
  await writeFile(temporaryPath, `${JSON.stringify(payload, null, 2)}\n`, 'utf8');
  await rename(temporaryPath, outputPath);
  return {
    outputPath,
    relativePath: path.relative(projectRoot, outputPath).split(path.sep).join('/'),
  };
}
async function readJsonRequest(request, maxBytes) {
  const chunks = [];
  let totalBytes = 0;
  for await (const chunk of request) {
    totalBytes += chunk.length;
    if (totalBytes > maxBytes) throw new Error('Pose data exceeds the 1 MB limit');
    chunks.push(chunk);
  }
  return JSON.parse(Buffer.concat(chunks).toString('utf8'));
}

function sendJson(response, statusCode, payload) {
  response.statusCode = statusCode;
  response.setHeader('Content-Type', 'application/json; charset=utf-8');
  response.end(JSON.stringify(payload));
}

export function manualPoseCapturePlugin({ projectRoot = process.cwd() } = {}) {
  return {
    name: 'manual-pose-capture',
    apply: 'serve',
    configureServer(server) {
      server.middlewares.use(async (request, response, next) => {
        const requestUrl = new URL(request.url ?? '/', 'http://127.0.0.1');
        if (requestUrl.pathname !== ENDPOINT) {
          next();
          return;
        }
        if (request.method !== 'POST') {
          sendJson(response, 405, { ok: false, error: 'Method not allowed' });
          return;
        }
        try {
          const payload = await readJsonRequest(request, MAX_REQUEST_BYTES);
          const saved = await persistManualPoseSnapshot(projectRoot, payload);
          sendJson(response, 200, { ok: true, path: saved.relativePath });
        } catch (error) {
          const message = error instanceof Error ? error.message : String(error);
          sendJson(response, 400, { ok: false, error: message });
        }
      });
    },
  };
}
