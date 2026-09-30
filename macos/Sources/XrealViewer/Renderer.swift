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
        float4 rect;               // x: top, y: bottom, z: lean towards the viewer, w: how far columns wrap
        float4 outline;            // x: 1 to outline the canvas; y: rows it is drawn in; z: 1 to sharpen;
                                   // w: soften the edges of 1 the whole canvas, 2 this panel
        float4 spin0, spin1, spin2;  // turns the upright screen to its tilt
        float4 lens;               // xy: where the lens lookup starts, z: its step, w: 1 to use it
        float4 lensGrid;           // xy: the lookup's columns and rows, zw: the eye's size in pixels
        float4 halo;               // the glow: xy the canvas's size in points, z how far it reaches, w how bright
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
        float3 ahead = normalize(float3(panel.center.x, 0, panel.center.z));
        // How far towards the viewer the row at this height has come.
        float inward = panel.rect.z * (y + 1) * length(panel.up.xyz);
        if (panel.shape.x > 0 && panel.rect.w > 0) {
            // Wrapped: the column's arc, spaced as a Mercator map, sets the
            // row's height and circle, so pixels keep their shape.
            float rowRadius = length(panel.right.xyz) / panel.shape.x;
            float columnRadius = rowRadius / panel.rect.w;
            float rise = atan(sinh(y * length(panel.up.xyz) / columnRadius));
            float ring = rowRadius - columnRadius * (1 - cos(rise));
            float across = x * panel.shape.x;
            float3 level = sin(across) * normalize(panel.right.xyz) + cos(across) * ahead;
            room = panel.center.xyz - rowRadius * ahead + ring * level + columnRadius * sin(rise) * normalize(panel.up.xyz);
        } else if (panel.shape.x > 0) {
            // Curved: the row's arc, narrowed by the lean, plus the column.
            float rowRadius = length(panel.right.xyz) / panel.shape.x;
            float across = x * panel.shape.x;
            room = panel.center.xyz + y * panel.up.xyz - rowRadius * ahead
                + (rowRadius - inward) * (sin(across) * normalize(panel.right.xyz) + cos(across) * ahead);
        } else {
            room = panel.center.xyz + x * panel.right.xyz + y * panel.up.xyz;
            if (panel.rect.z != 0) {
                room -= inward * ahead;
            }
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

    // The glow round the canvas: the colour of the canvas's nearest edge, as
    // the small blurred copy of it has it, brightest at the edge and fading
    // out over `halo.z` points. Light only, added to what is behind.
    fragment float4 haloFragment(PanelOut in [[stage_in]],
                                 constant Panel &panel [[buffer(0)]],
                                 texture2d<float> ambient [[texture(0)]],
                                 sampler linear [[sampler(0)]]) {
        float2 inside = clamp(in.screenUV, 0.0, 1.0);
        float reach = length((in.screenUV - inside) * panel.halo.xy) / panel.halo.z;
        if (reach >= 1) {
            return float4(0);
        }
        float fall = (1 - reach) * (1 - reach) * exp(-2 * reach);
        return float4(ambient.sample(linear, inside).rgb * fall * panel.halo.w, 0);
    }

    // The small blurred copy of the canvas the glow takes its colours from:
    // one quad per capture tile, span.xy its part of the canvas's width and
    // span.zw the copy's size in pixels.
    struct AmbientOut {
        float4 position [[position]];
    };

    vertex AmbientOut ambientVertex(uint id [[vertex_id]], constant float4 &span [[buffer(0)]]) {
        float2 corner = float2(id & 1, id >> 1);
        AmbientOut out;
        out.position = float4(mix(span.x, span.y, corner.x) * 2 - 1, 1 - corner.y * 2, 0, 1);
        return out;
    }

    // Each pixel of the copy averages the tile over about three of its own
    // pixels each way, so the glow is smooth.
    fragment float4 ambientFragment(AmbientOut in [[stage_in]],
                                    constant float4 &span [[buffer(0)]],
                                    texture2d<float> source [[texture(0)]],
                                    sampler linear [[sampler(0)]]) {
        float2 onCanvas = in.position.xy / span.zw;
        float width = max(span.y - span.x, 1e-4);
        float2 uv = float2((onCanvas.x - span.x) / width, onCanvas.y);
        float2 texel = float2(1 / (span.z * width), 1 / span.w);
        float3 sum = 0;
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                sum += source.sample(linear, uv + (float2(i, j) / 7 - 0.5) * 3 * texel).rgb;
            }
        }
        return float4(sum / 64, 1);
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
        float2 panelWidth = fwidth(in.uv);
        float2 dx = dfdx(in.uv);
        float2 dy = dfdy(in.uv);
        // Fades out over the last few glasses pixels of the canvas or the
        // panel, so it ends softly against the room instead of in a hard cut.
        float fade = 1;
        if (panel.outline.w > 0.5) {
            bool whole = panel.outline.w < 1.5;
            float2 at = whole ? in.screenUV : in.uv;
            float2 fromEdge = min(at, 1 - at) / max(whole ? edgeWidth : panelWidth, float2(1e-6));
            fade = saturate(min(fromEdge.x, fromEdge.y) / 6);
        }
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
        float footprint = max(length(dx * size), length(dy * size));
        float blend = smoothstep(1.0, 1.25, footprint);
        if (blend <= 0) {
            return fade * (panel.outline.z > 0.5 ? sampleSharp(source, linear, in.uv, size) : source.sample(linear, in.uv));
        }
        float4 averaged = 0.25
            * (source.sample(linear, in.uv + 0.25 * (dx + dy)) + source.sample(linear, in.uv + 0.25 * (dx - dy))
                + source.sample(linear, in.uv - 0.25 * (dx + dy)) + source.sample(linear, in.uv - 0.25 * (dx - dy)));
        // Continuous across the magnification boundary: resizing or
        // panning must not suddenly change the shape of thin text strokes.
        if (blend >= 1) { return fade * averaged; }
        float4 sharp = panel.outline.z > 0.5 ? sampleSharp(source, linear, in.uv, size) : source.sample(linear, in.uv);
        return fade * mix(sharp, averaged, blend);
    }
    """

// The small blurred copy of the canvas the glow round it takes its colours
// from, in pixels.
private let ambientSize = SIMD2<Float>(128, 48)
// Samples a pixel, for smooth edges.
private let sampleCount = 4
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
        case .ambient: nil
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
    // The glow round the canvas, and the small blurred copy of the canvas it
    // takes its colours from, eased from frame to frame.
    private let haloPipeline: MTLRenderPipelineState
    private let ambientPipeline: MTLRenderPipelineState
    private var ambient: MTLTexture?
    private var ambientStarted = false
    private let sampler: MTLSamplerState
    // Match the drawable size; recreated when that changes.
    private var sampled: (color: MTLTexture, depth: MTLTexture)?
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
        panelDescriptor.rasterSampleCount = sampleCount
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
        panelDescriptor.fragmentFunction = library.makeFunction(name: "haloFragment")
        haloPipeline = try device.makeRenderPipelineState(descriptor: panelDescriptor)
        let ambientDescriptor = MTLRenderPipelineDescriptor()
        ambientDescriptor.vertexFunction = library.makeFunction(name: "ambientVertex")
        ambientDescriptor.fragmentFunction = library.makeFunction(name: "ambientFragment")
        ambientDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm_srgb
        // Each frame moves the copy a fifth of the way to the canvas, so the
        // glow drifts rather than flickers.
        let ease = ambientDescriptor.colorAttachments[0]!
        ease.isBlendingEnabled = true
        ease.sourceRGBBlendFactor = .blendColor
        ease.destinationRGBBlendFactor = .oneMinusBlendColor
        ease.sourceAlphaBlendFactor = .one
        ease.destinationAlphaBlendFactor = .zero
        ambientPipeline = try device.makeRenderPipelineState(descriptor: ambientDescriptor)
        let ambientTexture = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .bgra8Unorm_srgb, width: Int(ambientSize.x), height: Int(ambientSize.y), mipmapped: false)
        ambientTexture.usage = [.renderTarget, .shaderRead]
        ambientTexture.storageMode = .private
        ambient = device.makeTexture(descriptor: ambientTexture)
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
        to drawable: CAMetalDrawable, images: PanelImages, room: RoomView?, sharpen: Bool, softEdges: Bool = true,
        onFinished: @escaping @Sendable (_ gpuEnd: Double) -> Void = { _ in }
    ) {
        guard let commandBuffer = queue.makeCommandBuffer() else { return }
        if let room, room.panels.contains(where: { $0.source == .ambient }) {
            drawAmbient(from: room, images: images, into: commandBuffer)
        }
        let pass = MTLRenderPassDescriptor()
        // Drawn with several samples a pixel, so edges come out smooth
        // rather than stepped and crawling as the head moves, then resolved
        // into the drawable. The samples never leave the GPU's tile memory.
        guard let targets = sampleTargets(matching: drawable.texture) else { return }
        pass.colorAttachments[0].texture = targets.color
        pass.colorAttachments[0].resolveTexture = drawable.texture
        pass.colorAttachments[0].loadAction = .clear
        pass.colorAttachments[0].clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        pass.colorAttachments[0].storeAction = .multisampleResolve
        pass.depthAttachment.texture = targets.depth
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
                    images: images, sharpen: sharpen, softEdges: softEdges, encoder: encoder)
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

    /// Brings the small blurred copy of the canvas a step closer to what the
    /// canvas shows now.
    private func drawAmbient(from room: RoomView, images: PanelImages, into commandBuffer: MTLCommandBuffer) {
        guard let ambient else { return }
        let pass = MTLRenderPassDescriptor()
        pass.colorAttachments[0].texture = ambient
        pass.colorAttachments[0].loadAction = ambientStarted ? .load : .clear
        pass.colorAttachments[0].clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        pass.colorAttachments[0].storeAction = .store
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: pass) else { return }
        encoder.setRenderPipelineState(ambientPipeline)
        encoder.setFragmentSamplerState(sampler, index: 0)
        // The first frame fills it straight away.
        let step: Float = ambientStarted ? 0.2 : 1
        encoder.setBlendColor(red: step, green: step, blue: step, alpha: 1)
        for panel in room.panels {
            guard let tile = panel.tile, tile < images.canvas.count, let image = images.canvas[tile] else { continue }
            var span = SIMD4<Float>((panel.rect.left + 1) / 2, (panel.rect.right + 1) / 2, ambientSize.x, ambientSize.y)
            encoder.setVertexBytes(&span, length: MemoryLayout<SIMD4<Float>>.stride, index: 0)
            encoder.setFragmentBytes(&span, length: MemoryLayout<SIMD4<Float>>.stride, index: 0)
            encoder.setFragmentTexture(image.texture, index: 0)
            encoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
        }
        encoder.endEncoding()
        ambientStarted = true
    }

    private func drawPanels(
        of room: RoomView, eye: EyeOptics, viewProjection: simd_float4x4, scanEndViewProjection: simd_float4x4,
        images: PanelImages, sharpen: Bool, softEdges: Bool, encoder: MTLRenderCommandEncoder
    ) {
        let lens = eye.distortion.map { SIMD4<Float>($0.origin.x, $0.origin.y, $0.step, 1) } ?? .zero
        let lensGrid = SIMD4(
            Float(eye.distortion?.columns ?? 1), Float(eye.distortion?.rows ?? 1), eye.size.x, eye.size.y)
        for panel in room.panels {
            let isAmbient = panel.source == .ambient
            guard let texture = isAmbient ? ambient : images[panel.source]?.texture else { continue }
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
                rect: SIMD4(rect.top, rect.bottom, surface.lean, surface.wrap),
                outline: SIMD4(
                    panel.highlighted ? 1 : 0, Float(rows), isCanvas && sharpen ? 1 : 0,
                    !softEdges || panel.source == .pointer ? 0 : isCanvas ? 1 : 2),
                spin0: SIMD4(spin.columns.0, 0), spin1: SIMD4(spin.columns.1, 0), spin2: SIMD4(spin.columns.2, 0),
                lens: lens, lensGrid: lensGrid, halo: panel.halo)
            encoder.setRenderPipelineState(isAmbient ? haloPipeline : panelPipeline)
            encoder.setDepthStencilState(panel.source == .pointer || isAmbient ? overDepthState : depthState)
            encoder.setVertexBytes(&uniforms, length: MemoryLayout<PanelUniforms>.stride, index: 0)
            encoder.setFragmentBytes(&uniforms, length: MemoryLayout<PanelUniforms>.stride, index: 0)
            encoder.setFragmentTexture(texture, index: 0)
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
        var halo: SIMD4<Float>
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

    /// The multisampled colour and depth to draw into, the size of `target`.
    private func sampleTargets(matching target: MTLTexture) -> (color: MTLTexture, depth: MTLTexture)? {
        if let sampled, sampled.color.width == target.width, sampled.color.height == target.height {
            return sampled
        }
        func texture(_ format: MTLPixelFormat) -> MTLTexture? {
            let descriptor = MTLTextureDescriptor.texture2DDescriptor(
                pixelFormat: format, width: target.width, height: target.height, mipmapped: false)
            descriptor.textureType = .type2DMultisample
            descriptor.sampleCount = sampleCount
            descriptor.usage = .renderTarget
            descriptor.storageMode = .memoryless
            return device.makeTexture(descriptor: descriptor)
        }
        guard let color = texture(.bgra8Unorm_srgb), let depth = texture(.depth32Float) else { return nil }
        sampled = (color, depth)
        return sampled
    }
}
