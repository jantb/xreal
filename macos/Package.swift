// swift-tools-version: 6.2
import PackageDescription

let package = Package(
    name: "XrealViewer",
    platforms: [.macOS("27.0")],
    targets: [
        .target(name: "CDcmImu"),
        .target(name: "CVirtualDisplay"),
        .target(
            name: "XrealCore",
            dependencies: ["CDcmImu"],
            linkerSettings: [.linkedFramework("IOKit")]
        ),
        .executableTarget(
            name: "XrealViewer",
            dependencies: ["XrealCore", "CVirtualDisplay"],
            resources: [.copy("Resources/AppIcon.png")],
            linkerSettings: [
                .linkedFramework("AppKit"),
                .linkedFramework("Carbon"),
                .linkedFramework("CoreGraphics"),
                .linkedFramework("Metal"),
                .linkedFramework("ScreenCaptureKit"),
            ]
        ),
        .testTarget(name: "XrealCoreTests", dependencies: ["XrealCore"]),
        .testTarget(name: "XrealViewerTests", dependencies: ["XrealViewer", "XrealCore"]),
    ]
)
