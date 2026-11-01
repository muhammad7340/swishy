#version 330
// Ambient fish (models/fish_*): the mesh faces +x, centred on the model
// origin, which is the centre of its swimming path. Each frame the fish
// moves along a closed path (ellipse or figure-eight) in the model's xy
// plane, turns to face where it is going, bobs gently and bends its body
// in a wave travelling from head to tail. The model's world position seeds
// the direction, speed, path size and phase, so copies of one species swim
// out of step with each other.

in vec4 vertex;
in vec3 normal;
in vec4 uv0;

uniform mat4 worldviewproj_matrix;
uniform mat4 world_matrix;
uniform float t;
uniform float bodyLength;   // m, along +x
uniform float waveAmp;      // tail amplitude as a fraction of bodyLength
uniform float headAmp;      // share of waveAmp the head still moves (0..1)
uniform float waveFreq;     // tail beats per second
uniform float waveCount;    // body waves along the fish (eels > 1)
uniform float pattern;      // 0 = ellipse, 1 = figure-eight
uniform vec2 radii;         // path half-axes, m
uniform float speed;        // path rate, rad/s
uniform float bob;          // vertical bob, m

out gl_PerVertex
{
  vec4 gl_Position;
};

out block
{
  vec2 uv;
  vec3 n;
} outVs;

float hash(vec2 p)
{
  return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);
}

void main()
{
  vec2 base = world_matrix[3].xy;
  float h1 = hash(base);
  float h2 = hash(base + 17.1);
  float h3 = hash(base + 3.7);
  float dir = h1 < 0.5 ? -1.0 : 1.0;
  vec2 R = radii * (0.85 + 0.3 * h3);
  float th = 6.2832 * h2 + dir * speed * (0.85 + 0.3 * h2) * t;

  vec2 P;
  vec2 T;
  if (pattern < 0.5) {
    P = vec2(R.x * cos(th), R.y * sin(th));
    T = vec2(-R.x * sin(th), R.y * cos(th));
  } else {
    P = vec2(R.x * sin(th), 0.5 * R.y * sin(2.0 * th));
    T = vec2(R.x * cos(th), R.y * cos(2.0 * th));
  }
  T *= dir;
  float yaw = atan(T.y, T.x);

  // travelling body wave: s = 0 at the head, 1 at the tail
  vec3 v = vertex.xyz;
  float s = clamp((0.5 * bodyLength - v.x) / bodyLength, 0.0, 1.0);
  float phase = 6.2832 * (waveFreq * t - waveCount * s + h1);
  v.y += waveAmp * bodyLength * mix(headAmp, 1.0, s * s) * sin(phase);

  float c = cos(yaw);
  float sn = sin(yaw);
  vec3 w = vec3(c * v.x - sn * v.y, sn * v.x + c * v.y, v.z);
  w.xy += P;
  w.z += bob * sin(0.5 * th + 6.2832 * h3);

  gl_Position = worldviewproj_matrix * vec4(w, 1.0);
  outVs.uv = uv0.xy;
  outVs.n = vec3(c * normal.x - sn * normal.y, sn * normal.x + c * normal.y, normal.z);
}
