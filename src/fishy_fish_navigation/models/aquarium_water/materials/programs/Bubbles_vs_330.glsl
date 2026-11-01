#version 330
// Rising bubbles (meshes/bubbles.obj). Every vertex of one bubble shares its
// uv: u = phase offset, v = speed factor. The centre is vertex - normal*radius.

in vec4 vertex;
in vec3 normal;
in vec4 uv0;
uniform mat4 worldviewproj_matrix;
uniform vec3 camera_position_object_space;
uniform float t;
uniform float radius;     // radius the mesh was built with
uniform float topZ;       // bubbles rise to this height (model frame) and restart
uniform float speed;      // m/s

out gl_PerVertex
{
  vec4 gl_Position;
};

out block
{
  vec3 eyeVec;
  vec3 normal;
  float phase;
} outVs;

void main()
{
  vec3 c = vertex.xyz - normal * radius;
  // each bubble rises from its own start (sand or pump outlet) to topZ
  float rise = max(topZ - c.z, 0.05);
  float phase = fract(t * speed * uv0.y / rise + uv0.x);
  float seed = uv0.x * 37.0;
  c.z += phase * rise;
  c.x += 0.012 * sin(9.0 * phase + seed) * phase;        // wobble as they rise
  c.y += 0.012 * cos(7.0 * phase + 1.3 * seed) * phase;
  float r = radius * (0.6 + 0.8 * phase) * (0.7 + 0.6 * fract(seed));  // grow as pressure drops
  vec3 n = normal;
  n.z *= 0.85;                                            // slightly flattened
  vec4 P = vec4(c + n * r, 1.0);
  gl_Position = worldviewproj_matrix * P;
  outVs.eyeVec = P.xyz - camera_position_object_space;
  outVs.normal = normal;
  outVs.phase = phase;
}
