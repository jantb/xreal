import Metal
import QuartzCore
import XrealCore

private let shaderSource = """
    #include <metal_stdlib>
    using namespace metal;

    struct VertexOut {
        float4 position [[position]];
        float2 uv;
        float2 ndc;
    };

    // One triangle covering the screen. `p` runs from 0 to 2, so its visible
    // part maps output (0,0)-(1,1) onto the viewport `rect` (x, y, w, h) in
    // normalized source coordinates.
    vertex VertexOut viewportVertex(uint id [[vertex_id]], constant float4 &rect [[buffer(0)]]) {
        float2 p = float2((id << 1) & 2, id & 2);
        VertexOut out;
        out.ndc = float2(p.x * 2 - 1, 1 - p.y * 2);
        out.position = float4(out.ndc, 0, 1);
        out.uv = rect.xy + p * rect.zw;
        return out;
    }

    static float4 sampleOrBlack(texture2d<float> source, sampler linear, float2 uv) {
        if (any(uv < 0) || any(uv > 1)) {
            return float4(0, 0, 0, 1);
        }
        return float4(source.sample(linear, uv).rgb, 1);
    }

    fragment float4 cropFragment(VertexOut in [[stage_in]],
                                 texture2d<float> source [[texture(0)]],
                                 sampler linear [[sampler(0)]]) {
        return sampleOrBlack(source, linear, in.uv);
    }

    // Mirrors `SpatialView.sourcePoint(atOutput:)`.
    struct Spatial {
        float4 rotation0, rotation1, rotation2;
        float4 fovScreen;         // tan half fov xy, screen size zw
        float4 sourcePan;         // source size xy, pan zw
        float4 pivotScaleCurved;  // pivot xy, scale, curved
    };

    fragment float4 spatialFragment(VertexOut in [[stage_in]],
                                    constant Spatial &u [[buffer(0)]],
                                    texture2d<float> source [[texture(0)]],
                                    sampler linear [[sampler(0)]]) {
        float3x3 rotation = float3x3(u.rotation0.xyz, u.rotation1.xyz, u.rotation2.xyz);
        float3 room = rotation * float3(in.ndc * u.fovScreen.xy, -1);
        float2 onScreen;
        if (u.pivotScaleCurved.w > 0.5) {
            float distance = length(room.xz);
            if (distance < 1e-6) {
                return float4(0, 0, 0, 1);
            }
            onScreen = float2(atan2(room.x, -room.z), room.y / distance);
        } else {
            if (room.z > -1e-6) {
                return float4(0, 0, 0, 1);
            }
            onScreen = room.xy / -room.z;
        }
        float2 unit = float2(onScreen.x / u.fovScreen.z + 0.5, 0.5 - onScreen.y / u.fovScreen.w);
        float2 pixel = unit * u.sourcePan.xy + u.sourcePan.zw;
        float2 pivot = u.pivotScaleCurved.xy;
        pixel = pivot + (pixel - pivot) / u.pivotScaleCurved.z;
        return sampleOrBlack(source, linear, pixel / u.sourcePan.xy);
    }
    """

enum RendererError: Error {
    case noMetalDevice
}

/// Draws the latest captured frame as seen through the viewport. The frame
/// stays on the GPU the whole way: the captured IOSurface is sampled
/// directly. Used from the render thread only, apart from `device`.
final class Renderer: @unchecked Sendable {
    let device: MTLDevice
    private let queue: MTLCommandQueue
    private let cropPipeline: MTLRenderPipelineState
    private let spatialPipeline: MTLRenderPipelineState
    private let sampler: MTLSamplerState

    init() throws {
        guard let device = MTLCreateSystemDefaultDevice(), let queue = device.makeCommandQueue() else {
            throw RendererError.noMetalDevice
        }
        self.device = device
        self.queue = queue

        let library = try device.makeLibrary(source: shaderSource, options: nil)
        func pipeline(fragment: String) throws -> MTLRenderPipelineState {
            let descriptor = MTLRenderPipelineDescriptor()
            descriptor.vertexFunction = library.makeFunction(name: "viewportVertex")
            descriptor.fragmentFunction = library.makeFunction(name: fragment)
            descriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
            return try device.makeRenderPipelineState(descriptor: descriptor)
        }
        cropPipeline = try pipeline(fragment: "cropFragment")
        spatialPipeline = try pipeline(fragment: "spatialFragment")

        let samplerDescriptor = MTLSamplerDescriptor()
        samplerDescriptor.minFilter = .linear
        samplerDescriptor.magFilter = .linear
        samplerDescriptor.sAddressMode = .clampToEdge
        samplerDescriptor.tAddressMode = .clampToEdge
        guard let sampler = device.makeSamplerState(descriptor: samplerDescriptor) else {
            throw RendererError.noMetalDevice
        }
        self.sampler = sampler
    }

    /// Clears to black when there is nothing to show yet.
    func draw(to drawable: CAMetalDrawable, frame: CapturedFrame?, geometry: ViewGeometry?) {
        guard let commandBuffer = queue.makeCommandBuffer() else { return }
        let pass = MTLRenderPassDescriptor()
        pass.colorAttachments[0].texture = drawable.texture
        pass.colorAttachments[0].loadAction = .clear
        pass.colorAttachments[0].clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        pass.colorAttachments[0].storeAction = .store
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: pass) else { return }

        if let frame, let geometry {
            var rect = SIMD4<Float>(0, 0, 1, 1)
            switch geometry {
            case .crop(let crop):
                let width = Float(frame.width)
                let height = Float(frame.height)
                rect = SIMD4(crop.x / width, crop.y / height, crop.width / width, crop.height / height)
                encoder.setRenderPipelineState(cropPipeline)
            case .spatial(let view):
                var uniforms = [
                    SIMD4(view.rotation.columns.0, 0),
                    SIMD4(view.rotation.columns.1, 0),
                    SIMD4(view.rotation.columns.2, 0),
                    SIMD4(lowHalf: view.tanHalfFov, highHalf: view.screenSize),
                    SIMD4(lowHalf: view.sourceSize, highHalf: view.pan),
                    SIMD4(lowHalf: view.pivot, highHalf: SIMD2(view.scale, view.curved ? 1 : 0)),
                ]
                encoder.setRenderPipelineState(spatialPipeline)
                encoder.setFragmentBytes(
                    &uniforms, length: MemoryLayout<SIMD4<Float>>.stride * uniforms.count, index: 0)
            }
            encoder.setVertexBytes(&rect, length: MemoryLayout<SIMD4<Float>>.size, index: 0)
            encoder.setFragmentTexture(frame.texture, index: 0)
            encoder.setFragmentSamplerState(sampler, index: 0)
            encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)
        }
        encoder.endEncoding()
        // The capture surface must not be recycled while the GPU still reads it.
        commandBuffer.addCompletedHandler { _ in withExtendedLifetime(frame) {} }
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }
}
