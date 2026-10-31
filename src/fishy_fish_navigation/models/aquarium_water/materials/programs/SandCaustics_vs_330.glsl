#version 330
// Animated caustics on the sand (meshes/sand_caustics.obj).

in vec4 vertex;
uniform mat4 worldviewproj_matrix;

out gl_PerVertex
{
  vec4 gl_Position;
};

out block
{
  vec2 xy;
} outVs;

void main()
{
  gl_Position = worldviewproj_matrix * vertex;
  outVs.xy = vertex.xy;
}
