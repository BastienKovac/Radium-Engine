#version 130
#extension GL_ARB_draw_buffers : enable
#extension GL_ARB_compatibility : enable
#extension GL_EXT_gpu_shader4 : enable

uniform sampler2D grad;
uniform float     sw;
uniform float     sh;
uniform int       type;

// type = 0: nothing
// type = 1: suggestive contours (SC)
// type = 2: suggestive highlights (SH)
// type = 3: SC + SH
// type = 4: apparent ridges (AR)
// type = 5: apparent valleys (AV)
// type = 6: AR + AV
// type = 7: demarcating curves

vec4 vals[8];

void loadValues() {
  // |v[0]|v[1]|v[2]|
  // |v[3]|v[8]|v[4]|
  // |v[5]|v[6]|v[7]|

  // coordinates
  float xc = gl_TexCoord[0].s;
  float xm = gl_TexCoord[0].s-sw;
  float xp = gl_TexCoord[0].s+sw;

  float yc = gl_TexCoord[0].t;
  float ym = gl_TexCoord[0].t-sh;
  float yp = gl_TexCoord[0].t+sh;

  // automatically load view values (for depth discontinuities)
  vals[0] = texture2D(grad,vec2(xm,yp));
  vals[1] = texture2D(grad,vec2(xc,yp));
  vals[2] = texture2D(grad,vec2(xp,yp));
  vals[3] = texture2D(grad,vec2(xm,yc));
  vals[4] = texture2D(grad,vec2(xp,yc));
  vals[5] = texture2D(grad,vec2(xm,ym));
  vals[6] = texture2D(grad,vec2(xc,ym));
  vals[7] = texture2D(grad,vec2(xp,ym));
}

float mtanh(float c) {
  // remapping [-inf,inf] in [-1,1]
  const float tanhmax = 3.11622;
  float x = (c/tanhmax);
  float e = exp(-2.0*x);
  return clamp((1.0-e)/(1.0+e),-1.0,1.0);
}

vec4 depthDiscontinuities() {
  vec2 g = vec2(2.0*vals[4].w-2.0*vals[3].w+vals[2].w-vals[0].w+vals[7].w-vals[5].w,
		2.0*vals[1].w-2.0*vals[6].w+vals[0].w-vals[5].w+vals[2].w-vals[7].w);

  // return tangent of gradient and magnitude of gradient
  return g==vec2(0.0) ? vec4(0.0) : vec4(normalize(vec2(-g.y,g.x)),length(g),0.0);
}

vec3 gradient() {
  vec2 g = texture2D(grad,gl_TexCoord[0].st).xy;

  return g==vec2(0.0) ? vec3(0.0) : vec3(normalize(vec2(-g.y,g.x)),mtanh(length(g)));
}

vec3 hessian() {

  float xx = 2.0*vals[4].x-2.0*vals[3].x+vals[2].x-vals[0].x+vals[7].x-vals[5].x;
  float xy = 2.0*vals[4].y-2.0*vals[3].y+vals[2].y-vals[0].y+vals[7].y-vals[5].y;
  float yx = 2.0*vals[1].x-2.0*vals[6].x+vals[0].x-vals[5].x+vals[2].x-vals[7].x;
  float yy = 2.0*vals[1].y-2.0*vals[6].y+vals[0].y-vals[5].y+vals[2].y-vals[7].y;

  return vec3(xx,yy,((xy+yx)/2.0));
}

vec4 curvature(in vec3 h,int type) {

  // singularities
  if(h.z==0.0) {
    return vec4(0.0);
  }

  // eigen values
  float tm = sqrt(h.x*h.x+4.0*h.z*h.z-2.0*h.x*h.y+h.y*h.y);
  float k1 = mtanh(-0.5*(h.x+h.y+tm));
  float k2 = mtanh(-0.5*(h.x+h.y-tm));

  // eigen vectors
  vec2  d1 = vec2( ( h.x-h.y+tm)/(2.0*h.z),1.0);
  vec2  d2 = vec2(-(-h.x+h.y+tm)/(2.0*h.z),1.0);
  d1 = d1==vec2(0.0) ? vec2(0.0) : normalize(d1);
  d2 = d2==vec2(0.0) ? vec2(0.0) : normalize(d2);

  vec2 dmin = abs(k1)>abs(k2) ? d1 : d2;
  vec2 dmax = abs(k1)>abs(k2) ? d2 : d1;
  float km  = 0.5*(k1+k2);

  // kmin/kmax and min/max dir
  if(type==4) {
    return k1>=k2 ? vec4(d1,k2,k1) : vec4(d2,k1,k2); // ridges
  } else if(type==5)
    return k2>=k1 ? vec4(d1,k2,k1) : vec4(d2,k1,k2); // valleys
  else
    return abs(k1)>abs(k2) ? vec4(d2,k1,k2) : vec4(d1,k2,k1); // ridges & valleys
}

float meanCurvature(in vec3 h) {
  float tm = sqrt(h.x*h.x+4.0*h.z*h.z-2.0*h.x*h.y+h.y*h.y);

  float k1 = -0.5*(h.x+h.y+tm);
  float k2 = -0.5*(h.x+h.y-tm);

  return 0.5*(k1+k2);
}

void main(void) {
  // return tangent of gradient and magnitude of gradient for
  // silhouettes and user lines

/*   if(texture2D(grad,gl_TexCoord[0].st)==vec4(0.0)) { */
/*     gl_FragData[0] = vec4(0.0); */
/*     gl_FragData[1] = vec4(0.0); */
/*     gl_FragData[2] = vec4(1.0); */
/*     return; */
/*   } */

  loadValues();
  vec3 h = hessian();
  vec4 d = depthDiscontinuities();

  gl_FragData[0]   = d;
  //gl_FragData[0].w = meanCurvature(h);
  gl_FragData[0].w = texture2D(grad,gl_TexCoord[0].st).x;

  if(type<=0) {
    // nothing
    gl_FragData[1] = vec4(0.0);

  } else if(type<=3) {
    // first order: SC, SH, SC+SH
    vec4 g = vec4(gradient(),0.0);
    gl_FragData[1] = g;

  } else if(type<=6) {
    // second order: AR, AV, AR+AV
    vec4 c = curvature(h,type);
    gl_FragData[1] = c;

  } else {
    // third order: demarcating curves
    gl_FragData[1] = vec4(0.0);

  }
}

