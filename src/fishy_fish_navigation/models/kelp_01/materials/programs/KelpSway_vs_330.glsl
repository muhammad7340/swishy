#version 330
// Swaying kelp (meshes/Kelp0N_sway.obj: metres, Z-up, base at z = 0). The base
// stays put and the bend grows with height squared, so the tips move most.
// Each plant gets its own phase from its world position, so repeated models
// placed side by side never move in sync.

in vec4 vertex;
in vec3 normal;
in vec4 uv0;
uniform mat4 worldviewproj_matrix;
uniform mat4 world_matrix;
uniform float t;
uniform float swayAmp;      // tip displacement in mesh units (m before SDF scale)
uniform float swayFreq;     // rad/s
uniform float plantHeight;  // mesh height in m before SDF scale

out gl_PerVertex
{
  vec4 gl_Position;
};

out block
{
  vec2 uv;
  vec3 n;
} outVs;

void main()
{
  vec2 base = world_matrix[3].xy;
  vec2 phase = 6.2832 * fract(sin(vec2(dot(base, vec2(12.9898, 78.233)),
                                       dot(base, vec2(39.3468, 11.1351)))) * 43758.5453);
  float f = swayFreq * (0.85 + 0.3 * fract(phase.x * 1.7));   // slightly different speeds

  vec4 P = vertex;
  float h = clamp(P.z / plantHeight, 0.0, 1.0);
  float bend = swayAmp * h * h;
  P.x += bend * sin(f * t + phase.x + P.z * 2.0);
  P.y += 0.6 * bend * cos(0.8 * f * t + phase.y + P.z * 1.7);

  gl_Position = worldviewproj_matrix * P;
  outVs.uv = uv0.xy;
  outVs.n = normal;
}
