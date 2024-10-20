const date = new Date();
const isoString = date.toISOString();

function decodeUplink(input) {
  return {
    data: {
      bytes:input.bytes.map(function (x) {
 return ('00' + x.toString(16)).slice(-2);
 }).join(''),  
      temperature: (input.bytes[0]),
      light: ((input.bytes[1] >> 8) | input.bytes[2]),
      deviceId: 'cc4950-jc221492-1',
      count: ((input.bytes[3] >> 8) | input.bytes[4]),
      voltage: toFloat(input.bytes.slice(5, 9)), 
      receivedAt: isoString
 },
 warnings: [],
 errors: []
 };
}

function toFloat(bytes) {
 const buffer = new ArrayBuffer(4);
 const f32 = new Float32Array(buffer);
 const ui8 = new Uint8Array(buffer);
 bytes.forEach(function (b, i) {
 ui8[i] = b;
 });
 return f32[0];
}
