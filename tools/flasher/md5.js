// MD5 of a binary string (one char per byte) -> lowercase hex. Used to let the
// flasher stub verify what was written.
function md5bstr(s) {
  const K = new Uint32Array(64), S = [7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21];
  for (let i = 0; i < 64; i++) K[i] = Math.floor(Math.abs(Math.sin(i + 1)) * 4294967296) >>> 0;
  const n = s.length, total = ((n + 8) >>> 6) + 1, words = new Uint32Array(total * 16);
  for (let i = 0; i < n; i++) words[i >> 2] |= (s.charCodeAt(i) & 0xff) << ((i % 4) * 8);
  words[n >> 2] |= 0x80 << ((n % 4) * 8);
  words[total * 16 - 2] = (n * 8) >>> 0;
  words[total * 16 - 1] = Math.floor(n / 536870912) >>> 0;
  let a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
  for (let blk = 0; blk < total * 16; blk += 16) {
    let a = a0, b = b0, c = c0, d = d0;
    for (let i = 0; i < 64; i++) {
      let f, g;
      if (i < 16) { f = (b & c) | (~b & d); g = i; }
      else if (i < 32) { f = (d & b) | (~d & c); g = (5 * i + 1) % 16; }
      else if (i < 48) { f = b ^ c ^ d; g = (3 * i + 5) % 16; }
      else { f = c ^ (b | ~d); g = (7 * i) % 16; }
      const tmp = d;
      d = c;
      c = b;
      const x = (a + f + K[i] + words[blk + g]) >>> 0;
      const r = S[(i >> 4) * 4 + (i % 4)];
      b = (b + ((x << r) | (x >>> (32 - r)))) >>> 0;
      a = tmp;
    }
    a0 = (a0 + a) >>> 0; b0 = (b0 + b) >>> 0; c0 = (c0 + c) >>> 0; d0 = (d0 + d) >>> 0;
  }
  let hex = "";
  for (const v of [a0, b0, c0, d0]) for (let i = 0; i < 4; i++) hex += ((v >>> (i * 8)) & 0xff).toString(16).padStart(2, "0");
  return hex;
}
if (typeof module !== "undefined") module.exports = md5bstr;
