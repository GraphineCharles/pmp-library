// Copyright 2011-2020 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

// clang-format off

// mat-cap shader: assume view=(0,0,-1), then the tex-coord for
// spherical environment mapping is just the normal's XY
// scaled by 0.5 and shifted by 0.5.
// scale by 0.49 to avoid artifacts at gracing angles
static const char* bake_vshader =
#ifndef __EMSCRIPTEN__
    "#version 330"
#else
    "#version 300 es"
#endif
R"glsl(
layout (location=0) in vec4 v_position;
layout (location=1) in vec3 v_normal;
layout (location=2) in vec2 v_tex;
layout (location=3) in vec3 v_color;
layout (location=4) in vec4 v_bake;

out vec3 v2f_normal;
out vec2 v2f_tex;
out vec3 v2f_color;
out vec4 v2f_bake;
uniform mat4 modelview_projection_matrix;
uniform mat3 normal_matrix;

void main()
{
    v2f_normal = normalize(normal_matrix * v_normal);
    v2f_tex = v_tex;
    v2f_color = v_color;
	v2f_bake = v_bake;
    gl_Position = modelview_projection_matrix * vec4(v_position.x, v_position.y, 0.0f, 1.0f);
}
)glsl";


static const char* bake_fshader =
#ifndef __EMSCRIPTEN__
    "#version 330"
#else
    "#version 300 es"
#endif
R"glsl(
precision mediump float;

in vec3 v2f_normal;
in vec3 v2f_color;
in vec2 v2f_tex;
in vec4 v2f_bake;
uniform sampler2D textureToBake;
uniform vec4 scale;
uniform bool bakeTex;
out vec4 f_color;

vec2 uv;
vec4 rgba;

void main()
{
	if ( bakeTex )
	{
		vec2 tex = v2f_tex;
		tex.y = 1.0f - tex.y;
		f_color =  vec4(v2f_tex,0.0f,1.0f);
		f_color =  texture(textureToBake, tex) * scale;
	}
	else
	{
		f_color = v2f_bake * scale;
	}
	//f_color.xy = v2f_tex;
	//f_color.z = 0.0f;	
	//f_color.w = 1.0f;	
}
)glsl";

// clang-format on
