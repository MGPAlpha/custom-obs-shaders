#define PI 3.14159

#define POSTERIZE(v, steps) (floor((v) * steps) / (steps))

float invLerp(float from, float to, float value){
  return (value - from) / (to - from);
}

float remap(float iMin, float iMax, float oMin, float oMax, float value) {
    return lerp(oMin, oMax, invLerp(iMin,iMax,value));
}

float3 Barycentric(float3 p, float3 a, float3 b, float3 c)
{
    float3 v0 = b - a, v1 = c - a, v2 = p - a;
    float den = v0.x * v1.y - v1.x * v0.y;
    float v = (v2.x * v1.y - v1.x * v2.y) / den;
    float w = (v0.x * v2.y - v2.x * v0.y) / den;
    float u = 1.0f - v - w;
    return float3(u,v,w);
}

float sdOctahedron( float3 p, float s)
{
  p = abs(p);
  return (p.x+p.y+p.z-s)*0.57735027;
}

float3 rotateX(float3 p, float angle) {
    return float3x3(1,0,0,0,cos(angle),sin(-angle),0,sin(angle),cos(angle)) * p;
}
float3 rotateY(float3 p, float angle) {
    return float3x3(cos(angle),0,sin(angle),0,1,0,sin(-angle),0,cos(angle)) * p;
}
float3 rotateZ(float3 p, float angle) {
    return float3x3(cos(angle),sin(-angle),0,sin(angle),cos(angle),0,0,0,1) * p;
}

float sdShape(float3 p, out float3 octaPos) {
    p -= float3(0,.1,3.2);
    p = rotateX(p,.25);
    p = rotateY(p,builtin_elapsed_time/4);
    octaPos = p;
    return sdOctahedron(p,1);
}

float2 pointToUv(float3 p) {
    
    
    p = abs(p);

    p = p - dot(p,float3(1,1,1))/3*float3(1,1,1);

    float2 uv = float2(dot(normalize(float3(-1,0,1)),p), dot(normalize(float3(-.5,1,-.5)),p));

    float yMax = dot(float3(0,1,0),normalize(float3(-.5,1,-.5)));
    float yMin = dot(float3(.5,0,.5),normalize(float3(-.5,1,-.5)));

    float xMin = dot(float3(1,0,0),normalize(float3(-1,0,1)));
    float xMax = dot(float3(0,0,1),normalize(float3(-1,0,1)));

    uv.x = remap(xMin, xMax, 0, 1, uv.x);
    uv.y = remap(yMin, yMax, 1, 0, uv.y);

    return uv;
}

#define STEP_LIMIT 100
#define SURFACE_DIST .001
#define MAX_DIST 100
float4 raymarch(float3 ro, float3 rd) {
    float3 p = ro;
    float3 octaPos;
    for (int i = 0; i < STEP_LIMIT; i++) {
        float dist = sdShape(p,octaPos);
        if (dist < SURFACE_DIST) break;
        if (dist > MAX_DIST) return float4(0,0,0,0);
        p += rd * dist;
    }

    float3 bary = Barycentric(abs(octaPos), float3(1,0,0),float3(0,1,0),float3(0,0,1));

    float minEdge = min(bary.x,min(bary.y,bary.z));

    float3 albedo = minEdge > .03 ? image.Sample(builtin_texture_sampler, pointToUv(octaPos)).xyz : float3(.5,1,1);

    return float4(albedo,1);
    // return float4(pointToUv(p),0,1);
}

float4 render(float2 uv) {
    // sample the source texture and return its color to be displayed
    float3 ro = float3(0);
    float3 rd = normalize(float3((uv-float2(.5))*2/3, 1));
    float4 col = raymarch(ro,rd);
    return col;
}
