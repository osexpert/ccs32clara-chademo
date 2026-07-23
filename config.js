// Shared logic for the ccs32clara-chademo config editor.
// Works three ways, unmodified:
//   1. <script src="config.js"></script> in index.html — functions become globals.
//   2. const cfg = require('./config.js') from another Node script/module.
//   3. node config.js <firmware-in> <configs.json> <config-name> <firmware-out>  (CLI, see bottom)

// ---- struct layout ----
// #pragma pack(push, 1)
// struct ConfigBlock {
//   char start_marker[9];      "<config>\0"
//   char name[NS];             e.g. "max-amps\0"
//   char type[TNS];            e.g. "uint8_t\0"
//   uint8_t value_len;         sizeof(VT)
//   volatile VT org_value;     compile-time default, never modified by this tool
//   volatile VT config_value;  the value we read/patch
//   char end_marker[10];       "</config>\0"
// };
// #pragma pack(pop)

const LITTLE_ENDIAN = true; // STM32 is always LE

function matchesAt(bytes, offset, needle){
  if (offset + needle.length > bytes.length) return false;
  for (let k = 0; k < needle.length; k++){
    if (bytes[offset + k] !== needle[k]) return false;
  }
  return true;
}

function readCString(bytes, offset){
  let end = offset;
  while (end < bytes.length && bytes[end] !== 0) end++;
  return { str: new TextDecoder().decode(bytes.slice(offset, end)), len: end - offset + 1 };
}

function findVersionString(bytes){
  const prefix = new TextEncoder().encode("ccs32clara-chademo ");
  for (let i = 0; i <= bytes.length - prefix.length; i++){
    if (matchesAt(bytes, i, prefix)){
      return readCString(bytes, i).str; // read from the match to the null terminator
    }
  }
  return null;
}

function parseConfigBlocks(bytes){
  const START = new TextEncoder().encode("<config>");
  const END = "</config>";
  const found = [];
  let i = 0;
  while (i <= bytes.length - START.length){
    if (!matchesAt(bytes, i, START)){ i++; continue; }
    try {
      let pos = i + START.length + 1; // marker + its null terminator
      const name = readCString(bytes, pos); pos += name.len;
      const type = readCString(bytes, pos); pos += type.len;
      const valueLen = bytes[pos]; pos += 1;
      const orgValueOffset = pos;                 // org_value comes first now
      const valueOffset = orgValueOffset + valueLen; // config_value — the writable field
      const afterConfigValue = valueOffset + valueLen;
      const endStr = new TextDecoder().decode(bytes.slice(afterConfigValue, afterConfigValue + END.length));
      if (endStr !== END){ i++; continue; }
      const blockEnd = afterConfigValue + END.length + 1; // + its null terminator
      found.push({ name: name.str, type: type.str, valueLen, orgValueOffset, valueOffset });
      i = blockEnd;
    } catch(e){
      i++;
    }
  }
  return found;
}

function readValue(bytes, offset, len, type){
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (type === 'bool') return view.getUint8(offset) !== 0;
  const signed = type.startsWith('int');
  if (len === 1) return signed ? view.getInt8(offset) : view.getUint8(offset);
  if (len === 2) return signed ? view.getInt16(offset, LITTLE_ENDIAN) : view.getUint16(offset, LITTLE_ENDIAN);
  if (len === 4) return signed ? view.getInt32(offset, LITTLE_ENDIAN) : view.getUint32(offset, LITTLE_ENDIAN);
  return view.getUint8(offset); // fallback: first byte
}

function writeValue(bytes, offset, len, type, newVal){
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (type === 'bool'){ view.setUint8(offset, newVal ? 1 : 0); return; }
  const signed = type.startsWith('int');
  if (len === 1) signed ? view.setInt8(offset, newVal) : view.setUint8(offset, newVal);
  else if (len === 2) signed ? view.setInt16(offset, newVal, LITTLE_ENDIAN) : view.setUint16(offset, newVal, LITTLE_ENDIAN);
  else if (len === 4) signed ? view.setInt32(offset, newVal, LITTLE_ENDIAN) : view.setUint32(offset, newVal, LITTLE_ENDIAN);
  else view.setUint8(offset, newVal);
}

function bounds(type, len){
  if (type === 'bool') return [0, 1];
  const signed = type.startsWith('int');
  const bits = len * 8;
  return signed ? [-(2 ** (bits - 1)), 2 ** (bits - 1) - 1] : [0, 2 ** bits - 1];
}

// ---- config list (configs.json) ----
// { "configs": [ { "name": "taycan", "extends": "default", "description": "...", "values": {"max-volts": 750} } ] }
// "name" is the only identifier — also what "extends" references and what selects a config from the CLI.

function resolveConfig(raw, byName, seen){
  if (seen.has(raw.name)) return raw.values || {}; // guard against cycles
  seen.add(raw.name);
  let values = {};
  if (raw.extends && byName[raw.extends]){
    values = { ...resolveConfig(byName[raw.extends], byName, seen) };
  }
  return { ...values, ...(raw.values || {}) };
}

function loadConfigList(json){
  const list = (json && Array.isArray(json.configs)) ? json.configs : [];
  const byName = Object.fromEntries(list.map(c => [c.name, c]));
  return list.map(c => ({
    name: c.name,
    description: c.description || '',
    values: resolveConfig(c, byName, new Set())
  }));
}

// Applies a resolved config to bytes. Strict: if ANY field in config.values doesn't
// exist in this firmware's blocks, nothing is written at all and an Error is thrown —
// a config JSON that's out of sync with the firmware must fail loudly, never silently
// produce a partially-patched file.
function applyConfig(bytes, blocks, config){
  const byName = Object.fromEntries(blocks.map(b => [b.name, b]));
  const missing = Object.keys(config.values).filter(name => !byName[name]);
  if (missing.length){
    throw new Error(`Config "${config.name}" references fields not present in this firmware: ${missing.join(', ')}`);
  }
  Object.entries(config.values).forEach(([name, value]) => {
    const b = byName[name];
    const v = b.type === 'bool' ? (value ? 1 : 0) : value;
    writeValue(bytes, b.valueOffset, b.valueLen, b.type, v);
  });
}

const ConfigEditor = {
  LITTLE_ENDIAN, matchesAt, readCString, findVersionString,
  parseConfigBlocks, readValue, writeValue, bounds,
  resolveConfig, loadConfigList, applyConfig
};

if (typeof module !== 'undefined' && module.exports){
  module.exports = ConfigEditor;
} else if (typeof window !== 'undefined'){
  window.ConfigEditor = ConfigEditor;
}

// ---- CLI: node config.js <firmware-in> <configs.json> <config-name> <firmware-out> ----
if (typeof require !== 'undefined' && typeof module !== 'undefined' && require.main === module){
  const fs = require('fs');
  const path = require('path');
  const [, , fwIn, configsPath, configName, fwOut] = process.argv;
  if (!fwIn || !configsPath || !configName || !fwOut){
    console.error('Usage: node config.js <firmware-in> <configs.json> <config-name> <firmware-out>');
    process.exit(1);
  }
  try {
    const bytes = new Uint8Array(fs.readFileSync(fwIn));
    const blocks = parseConfigBlocks(bytes);
    if (!blocks.length) throw new Error(`No <config> blocks found in ${fwIn}`);
    const json = JSON.parse(fs.readFileSync(configsPath, 'utf8'));
    const configs = loadConfigList(json);
    const config = configs.find(c => c.name === configName);
    if (!config) throw new Error(`Config "${configName}" not found in ${configsPath}`);
    applyConfig(bytes, blocks, config); // throws on any field missing from this firmware
    const outDir = path.dirname(fwOut);
    if (outDir && outDir !== '.') fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(fwOut, Buffer.from(bytes));
    const version = findVersionString(bytes);
    console.log(`Wrote ${fwOut}${version ? ` (${version})` : ''} with config "${configName}" applied.`);
  } catch (e){
    console.error('Error: ' + e.message);
    process.exit(1);
  }
}