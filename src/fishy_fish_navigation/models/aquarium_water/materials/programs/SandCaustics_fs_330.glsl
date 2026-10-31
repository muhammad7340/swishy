#version 330
// Refracted sunlight gathers into a bright, wavy web on the sand. Each layer is
// a cell pattern whose feature points drift in small circles; the light sits
// where two cells meet (second-nearest minus nearest distance is small). Two
// layers at different scales and speeds, plus a gentle domain warp, keep the
// web from ever looking like a repeating texture.

uniform float t;
uniform vec4 lightColor;
uniform float strength;   // peak opacity of the light
uniform float scale;      // cells per metre
uniform float speed;
uniform float width;      // line width in cell units

in block
{
  vec2 xy;
} inPs;

out vec4 fragColor;

vec2 hash2(vec2 p)
{
  p = vec2(dot(p, vec2(127.1, 311.7)), dot(p, vec2(269.5, 183.3)));
  return fract(sin(p) * 43758.5453);
}

float web(vec2 x, float time)
{
  vec2 n = floor(x);
  vec2 f = fract(x);
  float f1 = 8.0;
  float f2 = 8.0;
  for (int j = -1; j <= 1; ++j)
  {
    for (int i = -1; i <= 1; ++i)
    {
      vec2 g = vec2(float(i), float(j));
      vec2 o = 0.5 + 0.4 * sin(time + 6.2831 * hash2(n + g));
      float d = length(g + o - f);
      if (d < f1) { f2 = f1; f1 = d; }
      else if (d < f2) { f2 = d; }
    }
  }
  float l = 1.0 - smoothstep(0.0, width, f2 - f1);
  return l * l;
}

void main()
{
  float time = t * speed;
  vec2 p = inPs.xy * scale;
  p += 0.35 * vec2(sin(0.9 * p.y + 0.7 * time), cos(0.8 * p.x - 0.6 * time));
  float l1 = web(p, time);
  float l2 = web(p * 1.37 + vec2(3.1, 7.7), -1.3 * time);
  float light = clamp(0.65 * l1 + 0.5 * l2 + 0.6 * l1 * l2, 0.0, 1.0);
  fragColor = vec4(lightColor.rgb, strength * light);
}
