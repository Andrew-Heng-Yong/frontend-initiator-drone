#include <metal_stdlib>
using namespace metal;

struct Vertex { packed_float3 position; packed_float3 color; };
struct RasterVertex {
    float4 position [[position]];
    float3 color;
    float size [[point_size]];
};

vertex RasterVertex sceneVertex(uint id [[vertex_id]],
                                const device Vertex *vertices [[buffer(0)]],
                                constant float4x4 &matrix [[buffer(1)]]) {
    RasterVertex out;
    out.position = matrix * float4(float3(vertices[id].position), 1);
    out.color = float3(vertices[id].color);
    out.size = clamp(18.0 / max(out.position.w, 0.1), 2.0, 12.0);
    return out;
}

fragment float4 sceneLine(RasterVertex in [[stage_in]]) { return float4(in.color, 1); }
fragment float4 scenePoint(RasterVertex in [[stage_in]], float2 point [[point_coord]]) {
    if (distance(point, float2(0.5)) > 0.5) discard_fragment();
    return float4(in.color, 1);
}


struct CameraVertex { float4 position [[position]]; float2 uv; };
vertex CameraVertex arCameraVertex(uint id [[vertex_id]],constant float3x3 &uvTransform [[buffer(0)]]) {
    float2 p=float2(id&1,id>>1);CameraVertex out;out.position=float4(p.x*2-1,1-p.y*2,1,1);
    out.uv=(uvTransform*float3(p,1)).xy;return out;
}
fragment float4 arCameraFragment(CameraVertex in [[stage_in]],texture2d<float> y [[texture(0)]],texture2d<float> uv [[texture(1)]]) {
    constexpr sampler s(filter::linear,address::clamp_to_edge);
    float l=y.sample(s,in.uv).r;float2 c=uv.sample(s,in.uv).rg-float2(.5);
    return float4(l+1.402*c.y,l-.344136*c.x-.714136*c.y,l+1.772*c.x,1);
}
struct ThermalVertex { float4 position [[position]];float size [[point_size]];float temperature;float depth;float2 uv; };
vertex ThermalVertex thermalVertex(uint id [[vertex_id]],const device float4 *points [[buffer(0)]],
                                   constant float4x4 &matrix [[buffer(1)]],constant float4x4 &optical [[buffer(2)]],
                                   constant float3x3 &k [[buffer(3)]],constant float4 &sizes [[buffer(4)]]) {
    float4 p=float4(points[id].xyz,1);float3 camera=(optical*p).xyz;float3 pixel=k*camera;
    ThermalVertex out;out.position=matrix*p;out.depth=camera.z;out.uv=pixel.xy/max(pixel.z,.001)/sizes.xy;
    out.temperature=points[id].w;out.size=clamp(16/max(out.position.w,.1f),2.f,12.f);return out;
}
fragment float4 thermalFragment(ThermalVertex in [[stage_in]],float2 point [[point_coord]],texture2d<float> depth [[texture(0)]],constant float2 &range [[buffer(0)]],constant float2 &options [[buffer(1)]]) {
    if(distance(point,float2(.5))>.5||in.depth<=0)discard_fragment();
    constexpr sampler s(filter::nearest,address::clamp_to_edge);
    float measured=depth.sample(s,in.uv).r;
    if(options.y<.5&&measured>0&&isfinite(measured)&&in.depth>measured+.08)discard_fragment();
    float t=clamp((in.temperature-range.x)/(range.y-range.x),0.f,1.f);
    float3 c=mix(float3(.1,.05,.4),float3(1,.2,0),min(1.f,t*2));
    c=mix(c,float3(1,1,.5),max(0.f,t*2-1));return float4(c,1);
}

fragment float4 thermalHeatFragment(ThermalVertex in [[stage_in]],texture2d<float> depth [[texture(0)]],constant float2 &range [[buffer(0)]],constant float2 &options [[buffer(1)]]) {
    if(in.depth<=0 || in.temperature<options.x)discard_fragment();
    constexpr sampler s(filter::nearest,address::clamp_to_edge);
    float measured=depth.sample(s,in.uv).r;
    if(options.y<.5 && measured>0 && isfinite(measured) && in.depth>measured+.08)discard_fragment();
    float t=clamp((in.temperature-options.x)/max(range.y-options.x,1.f),0.f,1.f);
    return float4(mix(float3(1,.35,0),float3(1,1,.2),t),.65);
}
