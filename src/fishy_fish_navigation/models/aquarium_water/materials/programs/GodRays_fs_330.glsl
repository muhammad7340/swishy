#version 330

uniform vec4 rayColor;
uniform float intensity;
uniform float waterZ;
uniform float t;

in block
{
  vec2 uv;
  vec3 eyeVec;
  vec3 normal;
  float camZ;
} inPs;

out vec4 fragColor;

void main()
{
  if (inPs.camZ > waterZ)
    discard;                                             // only seen from under water
  float seed = floor(inPs.uv.x);
  float u = fract(inPs.uv.x);
  float v = inPs.uv.y;
  float across = sin(3.14159 * u);                       // soft edges
  float along = pow(1.0 - v, 1.3) * smoothstep(0.0, 0.06, v);  // fade to the sand
  float facing = abs(dot(normalize(inPs.eyeVec), normalize(inPs.normal)));
  float edgeOn = smoothstep(0.05, 0.45, facing);         // hide strips seen edge-on
  float flicker = 0.75 + 0.25 * sin(1.3 * t + 4.1 * seed) * sin(0.71 * t + 2.0 * seed);
  fragColor = vec4(rayColor.rgb, intensity * across * across * along * edgeOn * flicker);
}
