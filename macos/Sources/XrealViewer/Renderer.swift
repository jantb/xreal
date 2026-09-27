import Metal
import simd
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

    // One capture of a virtual screen hanging in the room: the part of the
    // screen from span.x to span.y (-1 to 1 across), flat or curved.
    struct Panel {
        float4x4 viewProjection;
        float4 center, right, up;  // room coordinates; half extents
        float4 shape;              // x: half arc (0 flat), yz: span, w: segments
        float4 outline;            // x: 1 to outline the screen
    };

    struct PanelOut {
        float4 position [[position]];
        float2 uv;        // in the capture
        float2 screenUV;  // in the whole screen, for the outline
    };

    // Mirrors `ScreenSurface.point(at:)`.
    vertex PanelOut panelVertex(uint id [[vertex_id]], constant Panel &panel [[buffer(0)]]) {
        // Triangle strip down the columns: top, bottom, next top, ...
        float t = float(id >> 1) / panel.shape.w;
        float x = mix(panel.shape.y, panel.shape.z, t);
        float y = (id & 1) ? -1 : 1;
        float3 room;
        if (panel.shape.x > 0) {
            float angle = x * panel.shape.x;
            float radius = length(panel.center.xyz);
            room = cos(angle) * panel.center.xyz + sin(angle) * radius * normalize(panel.right.xyz)
                + y * panel.up.xyz;
        } else {
            room = panel.center.xyz + x * panel.right.xyz + y * panel.up.xyz;
        }
        PanelOut out;
        out.position = panel.viewProjection * float4(room, 1);
        out.uv = float2(t, 0.5 - y * 0.5);
        out.screenUV = float2(x * 0.5 + 0.5, 0.5 - y * 0.5);
        return out;
    }

    fragment float4 panelFragment(PanelOut in [[stage_in]],
                                  constant Panel &panel [[buffer(0)]],
                                  texture2d<float> source [[texture(0)]],
                                  sampler linear [[sampler(0)]]) {
        // Derivatives first, while every pixel of the quad still runs.
        float2 edgeWidth = fwidth(in.screenUV);
        float2 dx = dfdx(in.uv);
        float2 dy = dfdy(in.uv);
        if (panel.outline.x > 0.5) {
            // About three glasses pixels wide, however far away the screen is.
            float2 fromEdge = min(in.screenUV, 1 - in.screenUV) / edgeWidth;
            if (min(fromEdge.x, fromEdge.y) < 3) {
                return float4(0.35, 0.75, 1, 1);
            }
        }
        // A screen pushed away covers several source pixels per glasses
        // pixel; averaging four samples across that keeps text from
        // shimmering.
        float2 size = float2(source.get_width(), source.get_height());
        if (max(length(dx * size), length(dy * size)) < 1.25) {
            return float4(source.sample(linear, in.uv).rgb, 1);
        }
        float3 sum = source.sample(linear, in.uv + 0.25 * (dx + dy)).rgb
            + source.sample(linear, in.uv + 0.25 * (dx - dy)).rgb
            + source.sample(linear, in.uv - 0.25 * (dx + dy)).rgb
            + source.sample(linear, in.uv - 0.25 * (dx - dy)).rgb;
        return float4(sum * 0.25, 1);
    }
    """

// Clip distances for the room's depth range, in room units (1 is where a
// screen shows at the glasses' pixel density).
private let nearClip: Float = 0.05
private let farClip: Float = 100
// A curved screen is drawn as flat strips about this wide.
private let curveSegmentAngle: Float = 0.02  // rad, about 1°

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
    private let panelPipeline: MTLRenderPipelineState
    private let depthState: MTLDepthStencilState
    private let sampler: MTLSamplerState
    // Matches the drawable size; recreated when that changes.
    private var depthTexture: MTLTexture?

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

        let panelDescriptor = MTLRenderPipelineDescriptor()
        panelDescriptor.vertexFunction = library.makeFunction(name: "panelVertex")
        panelDescriptor.fragmentFunction = library.makeFunction(name: "panelFragment")
        panelDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
        panelDescriptor.depthAttachmentPixelFormat = .depth32Float
        panelPipeline = try device.makeRenderPipelineState(descriptor: panelDescriptor)
        let depthDescriptor = MTLDepthStencilDescriptor()
        depthDescriptor.depthCompareFunction = .less
        depthDescriptor.isDepthWriteEnabled = true
        guard let depthState = device.makeDepthStencilState(descriptor: depthDescriptor) else {
            throw RendererError.noMetalDevice
        }
        self.depthState = depthState

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

    /// Draws `geometry` from `frames`, one per captured display: the crop and
    /// spatial views show the first, the room shows each panel's own. Clears
    /// to black when there is nothing to show yet.
    func draw(to drawable: CAMetalDrawable, frames: [CapturedFrame?], geometry: ViewGeometry?) {
        guard let commandBuffer = queue.makeCommandBuffer() else { return }
        let pass = MTLRenderPassDescriptor()
        pass.colorAttachments[0].texture = drawable.texture
        pass.colorAttachments[0].loadAction = .clear
        pass.colorAttachments[0].clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        pass.colorAttachments[0].storeAction = .store
        if case .room = geometry {
            pass.depthAttachment.texture = depthTexture(matching: drawable.texture)
            pass.depthAttachment.loadAction = .clear
            pass.depthAttachment.clearDepth = 1
            pass.depthAttachment.storeAction = .dontCare
        }
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: pass) else { return }
        encoder.setFragmentSamplerState(sampler, index: 0)

        switch geometry {
        case nil:
            break
        case .crop(let crop):
            if let frame = frames.first ?? nil {
                let width = Float(frame.width)
                let height = Float(frame.height)
                var rect = SIMD4(crop.x / width, crop.y / height, crop.width / width, crop.height / height)
                encoder.setRenderPipelineState(cropPipeline)
                encoder.setVertexBytes(&rect, length: MemoryLayout<SIMD4<Float>>.size, index: 0)
                encoder.setFragmentTexture(frame.texture, index: 0)
                encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)
            }
        case .spatial(let view):
            if let frame = frames.first ?? nil {
                var rect = SIMD4<Float>(0, 0, 1, 1)
                var uniforms = [
                    SIMD4(view.rotation.columns.0, 0),
                    SIMD4(view.rotation.columns.1, 0),
                    SIMD4(view.rotation.columns.2, 0),
                    SIMD4(lowHalf: view.tanHalfFov, highHalf: view.screenSize),
                    SIMD4(lowHalf: view.sourceSize, highHalf: view.pan),
                    SIMD4(lowHalf: view.pivot, highHalf: SIMD2(view.scale, view.curved ? 1 : 0)),
                ]
                encoder.setRenderPipelineState(spatialPipeline)
                encoder.setVertexBytes(&rect, length: MemoryLayout<SIMD4<Float>>.size, index: 0)
                encoder.setFragmentBytes(
                    &uniforms, length: MemoryLayout<SIMD4<Float>>.stride * uniforms.count, index: 0)
                encoder.setFragmentTexture(frame.texture, index: 0)
                encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)
            }
        case .room(let room):
            encoder.setRenderPipelineState(panelPipeline)
            encoder.setDepthStencilState(depthState)
            let viewProjection = Self.viewProjection(room)
            for panel in room.panels {
                guard panel.source < frames.count, let frame = frames[panel.source] else { continue }
                let surface = panel.surface
                let arc = surface.halfArc * (panel.span.y - panel.span.x)
                let segments = surface.halfArc > 0 ? max(Int((arc / curveSegmentAngle).rounded(.up)), 1) : 1
                var uniforms = PanelUniforms(
                    viewProjection: viewProjection, center: SIMD4(surface.center, 1), right: SIMD4(surface.right, 0),
                    up: SIMD4(surface.up, 0),
                    shape: SIMD4(surface.halfArc, panel.span.x, panel.span.y, Float(segments)),
                    outline: SIMD4(panel.highlighted ? 1 : 0, 0, 0, 0))
                encoder.setVertexBytes(&uniforms, length: MemoryLayout<PanelUniforms>.stride, index: 0)
                encoder.setFragmentBytes(&uniforms, length: MemoryLayout<PanelUniforms>.stride, index: 0)
                encoder.setFragmentTexture(frame.texture, index: 0)
                encoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 2 * (segments + 1))
            }
        }
        encoder.endEncoding()
        // The capture surfaces must not be recycled while the GPU still reads them.
        commandBuffer.addCompletedHandler { _ in withExtendedLifetime(frames) {} }
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    private struct PanelUniforms {
        var viewProjection: simd_float4x4
        var center: SIMD4<Float>
        var right: SIMD4<Float>
        var up: SIMD4<Float>
        var shape: SIMD4<Float>
        var outline: SIMD4<Float>
    }

    /// Room coordinates to clip space: into the head frame, then a
    /// perspective projection over the view's field of view with depth
    /// from 0 at `nearClip` to 1 at `farClip`.
    private static func viewProjection(_ room: RoomView) -> simd_float4x4 {
        let r = room.headRotation.transpose
        let view = simd_float4x4(
            SIMD4(r.columns.0, 0), SIMD4(r.columns.1, 0), SIMD4(r.columns.2, 0), SIMD4(0, 0, 0, 1))
        let depthScale = farClip / (nearClip - farClip)
        let projection = simd_float4x4(
            SIMD4(1 / room.tanHalfFov.x, 0, 0, 0), SIMD4(0, 1 / room.tanHalfFov.y, 0, 0),
            SIMD4(0, 0, depthScale, -1), SIMD4(0, 0, nearClip * depthScale, 0))
        return projection * view
    }

    private func depthTexture(matching target: MTLTexture) -> MTLTexture? {
        if let depthTexture, depthTexture.width == target.width, depthTexture.height == target.height {
            return depthTexture
        }
        let descriptor = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .depth32Float, width: target.width, height: target.height, mipmapped: false)
        descriptor.usage = .renderTarget
        descriptor.storageMode = .private
        depthTexture = device.makeTexture(descriptor: descriptor)
        return depthTexture
    }
}
