#version 330
// Clear centre, bright rim and a small highlight: reads as an air bubble.

in block
{
  vec3 eyeVec;
  vec3 normal;
  float phase;
} inPs;

out vec4 fragColor;

void main()
{
  vec3 E = normalize(inPs.eyeVec);
  vec3 N = normalize(inPs.normal);
  float rim = 1.0 - abs(dot(E, N));
  float highlight = pow(max(dot(N, normalize(vec3(0.4, 0.3, 1.0))), 0.0), 24.0);
  float a = 0.12 + 0.75 * pow(rim, 2.0) + highlight;
  a *= smoothstep(0.0, 0.04, inPs.phase) * (1.0 - smoothstep(0.92, 1.0, inPs.phase));
  vec3 col = mix(vec3(0.75, 0.92, 1.0), vec3(1.0), highlight + rim * 0.5);
  fragColor = vec4(col, clamp(a, 0.0, 1.0));
}
