import Carbon.HIToolbox
import XrealCore

/// A system-wide keyboard shortcut, delivered even while another app has
/// focus. Carbon's hot key API needs no accessibility permission.
@MainActor final class GlobalHotKey {
    private static var nextID: UInt32 = 1

    private let id: UInt32
    private let onPress: () -> Void
    private let onRelease: () -> Void
    private var hotKey: EventHotKeyRef?
    private var handler: EventHandlerRef?

    /// `onRelease` runs when the key is let go, so a shortcut can be held.
    init(keyCode: Int, modifiers: Int, onPress: @escaping () -> Void, onRelease: @escaping () -> Void = {}) {
        id = Self.nextID
        Self.nextID += 1
        self.onPress = onPress
        self.onRelease = onRelease
        var kinds = [
            EventTypeSpec(eventClass: OSType(kEventClassKeyboard), eventKind: UInt32(kEventHotKeyPressed)),
            EventTypeSpec(eventClass: OSType(kEventClassKeyboard), eventKind: UInt32(kEventHotKeyReleased)),
        ]
        let context = Unmanaged.passUnretained(self).toOpaque()
        let installed = InstallEventHandler(
            GetApplicationEventTarget(),
            { _, event, context in
                guard let event, let context else { return OSStatus(eventNotHandledErr) }
                var pressed = EventHotKeyID()
                GetEventParameter(
                    event, EventParamName(kEventParamDirectObject), EventParamType(typeEventHotKeyID), nil,
                    MemoryLayout<EventHotKeyID>.size, nil, &pressed)
                let hotKey = Unmanaged<GlobalHotKey>.fromOpaque(context).takeUnretainedValue()
                // Every shortcut's handler sees every shortcut; pass on the others.
                guard pressed.signature == signature, pressed.id == hotKey.id else {
                    return OSStatus(eventNotHandledErr)
                }
                let released = GetEventKind(event) == UInt32(kEventHotKeyReleased)
                MainActor.assumeIsolated { released ? hotKey.onRelease() : hotKey.onPress() }
                return noErr
            }, kinds.count, &kinds, context, &handler)
        let registered = RegisterEventHotKey(
            UInt32(keyCode), UInt32(modifiers), EventHotKeyID(signature: signature, id: id), GetApplicationEventTarget(),
            0, &hotKey)
        if installed != noErr || registered != noErr {
            eprint("Could not register a global shortcut (\(installed), \(registered))")
        }
    }

    isolated deinit {
        if let hotKey { UnregisterEventHotKey(hotKey) }
        if let handler { RemoveEventHandler(handler) }
    }
}

private let signature: OSType = 0x5852_564c  // "XRVL"
