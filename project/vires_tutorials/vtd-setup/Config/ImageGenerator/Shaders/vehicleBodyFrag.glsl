#version 130
in vec4 v_colorAmbientEmissive;
in vec3 v_normalBCS;
in vec3 v_normal;
in float v_fresnelCoeff;
in vec3 v_esPosition;

void main(void)
{
  gl_FragData[0].a = 1.0;
  gl_FragData[0].rgb = vec3( 1.0, 0.0, 0.0 );
}




