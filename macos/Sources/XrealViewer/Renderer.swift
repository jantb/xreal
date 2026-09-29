import Metal
import simd
import QuartzCore
import XrealCore

private let shaderSource = """
    #include <metal_stdlib>
    using namespace metal;

    // One image on the canvas's surface hanging in the room: the part of it
    // from shape.y to shape.z across and rect.x to rect.y down (-1 to 1 over
    // the canvas, beyond it above), flat or curved.
    struct Panel {
        float4x4 viewProjection;
        float4x4 scanEndViewProjection;  // as seen when the bottom row lights up
        float4 center, right, up;  // room coordinates; half extents
        float4 shape;              // x: half arc (0 flat), yz: left and right, w: segments
        float4 rect;               // x: top, y: bottom
        float4 outline;            // x: 1 to outline the canvas; y: rows it is drawn in; z: 1 to sharpen
        float4 spin0, spin1, spin2;  // turns the upright screen to its tilt
        float4 lens;               // xy: where the lens lookup starts, z: its step, w: 1 to use it
        float4 lensGrid;           // xy: the lookup's columns and rows, zw: the eye's size in pixels
    };

    struct PanelOut {
        float4 position [[position]];
        float2 uv;        // in the capture
        float2 screenUV;  // in the whole canvas, for the outline
    };

    // Mirrors `ScreenSurface.point(at:)` and `EyeOptics.displayPixel(of:)`.
    vertex PanelOut panelVertex(uint id [[vertex_id]], uint band [[instance_id]],
                                constant Panel &panel [[buffer(0)]],
                                texture2d<float> lens [[texture(0)]],
                                sampler linear [[sampler(0)]]) {
        // One triangle strip per row of the panel, across its columns:
        // top, bottom, next top, ...
        float t = float(id >> 1) / panel.shape.w;
        float v = float(band + (id & 1)) / panel.outline.y;
        float x = mix(panel.shape.y, panel.shape.z, t);
        float y = mix(panel.rect.x, panel.rect.y, v);
        float3 room;
        if (panel.shape.x > 0) {
            // Curved: the row's arc plus the straight column.
            float rowRadius = length(panel.right.xyz) / panel.shape.x;
            float3 ahead = normalize(float3(panel.center.x, 0, panel.center.z));
            float across = x * panel.shape.x;
            room = panel.center.xyz + rowRadius * (sin(across) * normalize(panel.right.xyz) - (1 - cos(across)) * ahead)
                + y * panel.up.xyz;
        } else {
            room = panel.center.xyz + x * panel.right.xyz + y * panel.up.xyz;
        }
        room = float3x3(panel.spin0.xyz, panel.spin1.xyz, panel.spin2.xyz) * room;
        PanelOut out;
        // The glasses light their rows top to bottom; each vertex is placed
        // as seen when its row lights up. The shift across the view is
        // about linear in height, so the vertices of a strip suffice.
        float4 top = panel.viewProjection * float4(room, 1);
        float4 bottom = panel.scanEndViewProjection * float4(room, 1);
        float row = top.w > 1e-4 ? 0.5 - 0.5 * top.y / top.w : 0;
        out.position = mix(top, bottom, row);
        if (panel.lens.w > 0.5 && out.position.w > 1e-4) {
            // Drawn where the lens shows it: from the pixel it would be at
            // without the lens, looked up in the lens's offsets.
            float2 size = panel.lensGrid.zw;
            float2 ndc = out.position.xy / out.position.w;
            float2 pixel = float2(ndc.x + 1, 1 - ndc.y) * 0.5 * size;
            float2 cell = (pixel - panel.lens.xy) / panel.lens.z + 0.5;
            pixel += lens.sample(linear, cell / panel.lensGrid.xy, level(0)).xy;
            ndc = float2(pixel.x / size.x * 2 - 1, 1 - pixel.y / size.y * 2);
            out.position.xy = ndc * out.position.w;
        }
        out.uv = float2(t, v);
        out.screenUV = float2(x * 0.5 + 0.5, 0.5 - y * 0.5);
        return out;
    }

    // Catmull-Rom from nine bilinear samples: sharper than one bilinear
    // sample, which blurs by up to half a pixel between source pixels.
    float4 sampleSharp(texture2d<float> source, sampler linear, float2 uv, float2 size) {
        float2 position = uv * size;
        float2 first = floor(position - 0.5) + 0.5;
        float2 f = position - first;
        float2 w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
        float2 w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
        float2 w2 = f * (0.5 + f * (2.0 - 1.5 * f));
        float2 w3 = f * f * (-0.5 + 0.5 * f);
        float2 w12 = w1 + w2;
        float2 at0 = (first - 1) / size;
        float2 at12 = (first + w2 / w12) / size;
        float2 at3 = (first + 2) / size;
        float4 sum = source.sample(linear, float2(at0.x, at0.y)) * w0.x * w0.y
            + source.sample(linear, float2(at12.x, at0.y)) * w12.x * w0.y
            + source.sample(linear, float2(at3.x, at0.y)) * w3.x * w0.y
            + source.sample(linear, float2(at0.x, at12.y)) * w0.x * w12.y
            + source.sample(linear, float2(at12.x, at12.y)) * w12.x * w12.y
            + source.sample(linear, float2(at3.x, at12.y)) * w3.x * w12.y
            + source.sample(linear, float2(at0.x, at3.y)) * w0.x * w3.y
            + source.sample(linear, float2(at12.x, at3.y)) * w12.x * w3.y
            + source.sample(linear, float2(at3.x, at3.y)) * w3.x * w3.y;
        return clamp(sum, 0.0, 1.0);
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
                // Light blue, in linear light.
                return float4(0.1, 0.52, 1, 1);
            }
        }
        // A canvas pushed even slightly away covers more than one source
        // pixel per glasses pixel, so some source pixels would fall between
        // glasses pixels and pop in and out as the head moves. Averaging
        // four samples across the footprint blends them in instead. At one
        // to one a single sample stays sharpest. Colours are premultiplied,
        // so the pointer's see-through parts blend over the canvas.
        float2 size = float2(source.get_width(), source.get_height());
        if (max(length(dx * size), length(dy * size)) < 1.02) {
            return panel.outline.z > 0.5 ? sampleSharp(source, linear, in.uv, size) : source.sample(linear, in.uv);
        }
        return (source.sample(linear, in.uv + 0.25 * (dx + dy)) + source.sample(linear, in.uv + 0.25 * (dx - dy))
            + source.sample(linear, in.uv - 0.25 * (dx + dy)) + source.sample(linear, in.uv - 0.25 * (dx - dy)))
            * 0.25;
    }
    """

// Clip distances for the room's depth range, in room units (1 is where the
// canvas shows at the glasses' pixel density).
private let nearClip: Float = 0.05
private let farClip: Float = 100
// The canvas is drawn as flat pieces about this wide and this tall, as seen
// from the viewer, so the lens correction and each row's timing bend it
// smoothly and a curved canvas looks round.
private let curveSegmentAngle: Float = 0.02  // rad, about 1°
private let rowAngle: Float = 0.03  // rad, about 1.7°

enum RendererError: Error {
    case noMetalDevice
}

/// The images a frame's panels show, as each panel's source names them.
struct PanelImages: Sendable {
    /// One per capture tile of the canvas.
    var canvas: [CapturedFrame?] = []
    var status: CapturedFrame?
    var pinned: CapturedFrame?
    var pointer: CapturedFrame?

    subscript(source: RoomView.Panel.Source) -> CapturedFrame? {
        switch source {
        case .canvas(let tile): tile < canvas.count ? canvas[tile] : nil
        case .status: status
        case .pinned: pinned
        case .pointer: pointer
        }
    }
}

/// Draws the canvas's latest captured frames as each eye sees them. The
/// frames stay on the GPU the whole way: the captured IOSurfaces are sampled
/// directly. Both the captures and the drawable are sRGB, so filtering
/// blends in linear light and thin text keeps its weight wherever it lands.
/// Used from the render thread only, apart from `device`.
final class Renderer: @unchecked Sendable {
    let device: MTLDevice
    private let queue: MTLCommandQueue
    private let panelPipeline: MTLRenderPipelineState
    private let depthState: MTLDepthStencilState
    // The pointer is drawn over the canvas it lies on, whatever the depth.
    private let overDepthState: MTLDepthStencilState
    private let sampler: MTLSamplerState
    // Matches the drawable size; recreated when that changes.
    private var depthTexture: MTLTexture?
    // Each eye's lens offsets, for the vertex shader, and the distortion
    // they were made from.
    private var lensTextures: [(distortion: LensDistortion, texture: MTLTexture)?] = []
    // Stands in for the lens offsets when an eye is drawn straight.
    private let noLens: MTLTexture

    init() throws {
        guard let device = MTLCreateSystemDefaultDevice(), let queue = device.makeCommandQueue() else {
            throw RendererError.noMetalDevice
        }
        self.device = device
        self.queue = queue

        let library = try device.makeLibrary(source: shaderSource, options: nil)
        let panelDescriptor = MTLRenderPipelineDescriptor()
        panelDescriptor.vertexFunction = library.makeFunction(name: "panelVertex")
        panelDescriptor.fragmentFunction = library.makeFunction(name: "panelFragment")
        panelDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm_srgb
        // Premultiplied: opaque images cover what is behind, the pointer's
        // see-through parts let it show.
        let blend = panelDescriptor.colorAttachments[0]!
        blend.isBlendingEnabled = true
        blend.sourceRGBBlendFactor = .one
        blend.sourceAlphaBlendFactor = .one
        blend.destinationRGBBlendFactor = .oneMinusSourceAlpha
        blend.destinationAlphaBlendFactor = .oneMinusSourceAlpha
        panelDescriptor.depthAttachmentPixelFormat = .depth32Float
        panelPipeline = try device.makeRenderPipelineState(descriptor: panelDescriptor)
        let depthDescriptor = MTLDepthStencilDescriptor()
        depthDescriptor.depthCompareFunction = .less
        depthDescriptor.isDepthWriteEnabled = true
        guard let depthState = device.makeDepthStencilState(descriptor: depthDescriptor) else {
            throw RendererError.noMetalDevice
        }
        self.depthState = depthState
        let overDescriptor = MTLDepthStencilDescriptor()
        overDescriptor.depthCompareFunction = .always
        overDescriptor.isDepthWriteEnabled = false
        guard let overDepthState = device.makeDepthStencilState(descriptor: overDescriptor) else {
            throw RendererError.noMetalDevice
        }
        self.overDepthState = overDepthState

        let samplerDescriptor = MTLSamplerDescriptor()
        samplerDescriptor.minFilter = .linear
        samplerDescriptor.magFilter = .linear
        samplerDescriptor.sAddressMode = .clampToEdge
        samplerDescriptor.tAddressMode = .clampToEdge
        guard let sampler = device.makeSamplerState(descriptor: samplerDescriptor) else {
            throw RendererError.noMetalDevice
        }
        self.sampler = sampler

        let noLensDescriptor = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rg16Float, width: 1, height: 1, mipmapped: false)
        noLensDescriptor.usage = .shaderRead
        guard let noLens = device.makeTexture(descriptor: noLensDescriptor) else {
            throw RendererError.noMetalDevice
        }
        let zero: [Float16] = [0, 0]
        zero.withUnsafeBytes { bytes in
            noLens.replace(region: MTLRegionMake2D(0, 0, 1, 1), mipmapLevel: 0, withBytes: bytes.baseAddress!, bytesPerRow: 4)
        }
        self.noLens = noLens
    }

    /// Draws `room` from `images` as one view per eye side by side, the
    /// canvas sharpened where it shows one to one if `sharpen`. Clears to
    /// black when there is nothing to show. `onFinished` is called with the
    /// time the GPU finished the frame.
    func draw(
        to drawable: CAMetalDrawable, images: PanelImages, room: RoomView?, sharpen: Bool,
        onFinished: @escaping @Sendable (_ gpuEnd: Double) -> Void = { _ in }
    ) {
        guard let commandBuffer = queue.makeCommandBuffer() else { return }
        let pass = MTLRenderPassDescriptor()
        pass.colorAttachments[0].texture = drawable.texture
        pass.colorAttachments[0].loadAction = .clear
        pass.colorAttachments[0].clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        pass.colorAttachments[0].storeAction = .store
        pass.depthAttachment.texture = depthTexture(matching: drawable.texture)
        pass.depthAttachment.loadAction = .clear
        pass.depthAttachment.clearDepth = 1
        pass.depthAttachment.storeAction = .dontCare
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: pass) else { return }
        encoder.setFragmentSamplerState(sampler, index: 0)

        if let room {
            encoder.setRenderPipelineState(panelPipeline)
            // One view per eye, side by side, each from where that eye is.
            let width = Double(drawable.texture.width) / Double(max(room.eyes.count, 1))
            let height = Double(drawable.texture.height)
            encoder.setVertexSamplerState(sampler, index: 0)
            for (index, eye) in room.eyes.enumerated() {
                encoder.setViewport(
                    MTLViewport(originX: width * Double(index), originY: 0, width: width, height: height, znear: 0, zfar: 1))
                encoder.setVertexTexture(lensTexture(for: eye.distortion, eye: index) ?? noLens, index: 0)
                let viewProjection = Self.viewProjection(eye, headRotation: room.headRotation)
                let scanEnd = room.scanEndRotation.map { Self.viewProjection(eye, headRotation: $0) }
                drawPanels(
                    of: room, eye: eye, viewProjection: viewProjection, scanEndViewProjection: scanEnd ?? viewProjection,
                    images: images, sharpen: sharpen, encoder: encoder)
            }
        }
        encoder.endEncoding()
        // The capture surfaces must not be recycled while the GPU still reads them.
        commandBuffer.addCompletedHandler { buffer in
            withExtendedLifetime(images) {}
            onFinished(buffer.gpuEndTime)
        }
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    private func drawPanels(
        of room: RoomView, eye: EyeOptics, viewProjection: simd_float4x4, scanEndViewProjection: simd_float4x4,
        images: PanelImages, sharpen: Bool, encoder: MTLRenderCommandEncoder
    ) {
        let lens = eye.distortion.map { SIMD4<Float>($0.origin.x, $0.origin.y, $0.step, 1) } ?? .zero
        let lensGrid = SIMD4(
            Float(eye.distortion?.columns ?? 1), Float(eye.distortion?.rows ?? 1), eye.size.x, eye.size.y)
        for panel in room.panels {
            guard let image = images[panel.source] else { continue }
            let surface = panel.surface
            let rect = panel.rect
            // How wide and tall the piece looks, roughly, from its distance.
            let distance = max(length(surface.center), 1e-3)
            let arc =
                surface.halfArc > 0
                ? surface.halfArc * (rect.right - rect.left)
                : length(surface.right) * (rect.right - rect.left) / distance
            let segments = max(Int((arc / curveSegmentAngle).rounded(.up)), 1)
            let rows = max(Int((length(surface.up) * (rect.top - rect.bottom) / distance / rowAngle).rounded(.up)), 1)
            let spin = simd_float3x3(surface.spin)
            let isCanvas = panel.tile != nil
            var uniforms = PanelUniforms(
                viewProjection: viewProjection, scanEndViewProjection: scanEndViewProjection,
                center: SIMD4(surface.center, 1), right: SIMD4(surface.right, 0), up: SIMD4(surface.up, 0),
                shape: SIMD4(surface.halfArc, rect.left, rect.right, Float(segments)),
                rect: SIMD4(rect.top, rect.bottom, 0, 0),
                outline: SIMD4(panel.highlighted ? 1 : 0, Float(rows), isCanvas && sharpen ? 1 : 0, 0),
                spin0: SIMD4(spin.columns.0, 0), spin1: SIMD4(spin.columns.1, 0), spin2: SIMD4(spin.columns.2, 0),
                lens: lens, lensGrid: lensGrid)
            encoder.setDepthStencilState(panel.source == .pointer ? overDepthState : depthState)
            encoder.setVertexBytes(&uniforms, length: MemoryLayout<PanelUniforms>.stride, index: 0)
            encoder.setFragmentBytes(&uniforms, length: MemoryLayout<PanelUniforms>.stride, index: 0)
            encoder.setFragmentTexture(image.texture, index: 0)
            encoder.drawPrimitives(
                type: .triangleStrip, vertexStart: 0, vertexCount: 2 * (segments + 1), instanceCount: rows)
        }
    }

    private struct PanelUniforms {
        var viewProjection: simd_float4x4
        var scanEndViewProjection: simd_float4x4
        var center: SIMD4<Float>
        var right: SIMD4<Float>
        var up: SIMD4<Float>
        var shape: SIMD4<Float>
        var rect: SIMD4<Float>
        var outline: SIMD4<Float>
        var spin0: SIMD4<Float>
        var spin1: SIMD4<Float>
        var spin2: SIMD4<Float>
        var lens: SIMD4<Float>
        var lensGrid: SIMD4<Float>
    }

    /// `distortion`'s offsets as a texture the vertex shader looks up in,
    /// made again only when the eye's distortion changes; nil to draw the
    /// eye straight.
    private func lensTexture(for distortion: LensDistortion?, eye: Int) -> MTLTexture? {
        guard let distortion else { return nil }
        while lensTextures.count <= eye {
            lensTextures.append(nil)
        }
        if let cached = lensTextures[eye], cached.distortion == distortion {
            return cached.texture
        }
        // Offsets from each lookup point to the display pixel that shows
        // it: a few pixels at most, so half floats keep them to a hundredth
        // of a pixel.
        var offsets: [Float16] = []
        offsets.reserveCapacity(distortion.displayPixels.count * 2)
        for (index, display) in distortion.displayPixels.enumerated() {
            let point = SIMD2(Float(index % distortion.columns), Float(index / distortion.columns))
            let offset = display - (distortion.origin + point * distortion.step)
            offsets.append(Float16(offset.x))
            offsets.append(Float16(offset.y))
        }
        let descriptor = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rg16Float, width: distortion.columns, height: distortion.rows, mipmapped: false)
        descriptor.usage = .shaderRead
        guard let texture = device.makeTexture(descriptor: descriptor) else { return nil }
        offsets.withUnsafeBytes { bytes in
            texture.replace(
                region: MTLRegionMake2D(0, 0, distortion.columns, distortion.rows), mipmapLevel: 0,
                withBytes: bytes.baseAddress!, bytesPerRow: distortion.columns * 4)
        }
        lensTextures[eye] = (distortion, texture)
        return texture
    }

    /// Room coordinates to clip space for `eye`: into the head frame, then
    /// into the eye's own frame, from where it sits and as it is turned, then
    /// the eye's projection from its calibrated focal length and optical
    /// centre, with depth from 0 at `nearClip` to 1 at `farClip`.
    private static func viewProjection(_ eye: EyeOptics, headRotation: simd_float3x3) -> simd_float4x4 {
        let r = eye.rotation.transpose * headRotation.transpose
        let t = -(eye.rotation.transpose * eye.position)
        let view = simd_float4x4(SIMD4(r.columns.0, 0), SIMD4(r.columns.1, 0), SIMD4(r.columns.2, 0), SIMD4(t, 1))
        let depthScale = farClip / (nearClip - farClip)
        // Pixels run right and down from the top-left corner; clip space
        // runs right and up from the middle.
        let scale = 2 * eye.focal / eye.size
        let offset = SIMD2(1 - 2 * eye.center.x / eye.size.x, 2 * eye.center.y / eye.size.y - 1)
        let projection = simd_float4x4(
            SIMD4(scale.x, 0, 0, 0), SIMD4(0, scale.y, 0, 0),
            SIMD4(offset.x, offset.y, depthScale, -1), SIMD4(0, 0, nearClip * depthScale, 0))
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
