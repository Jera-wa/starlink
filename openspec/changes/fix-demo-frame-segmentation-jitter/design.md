## Context

The current demo bridge moves raw bytes from UART RX into a ring buffer, drains contiguous slices into the SLE send path, and forwards received SLE bytes back to UART with another ring-buffer-to-DMA path. This keeps the bridge simple, but it also means fragmentation boundaries depend on UART callback chunking, ring-buffer wrap position, main-loop scheduling, and SLE busy retries instead of logical frame boundaries.

The captured image-refresh traces show the consequence of that design:
- send-side control bursts align almost one-to-one with receive-side logical frame starts,
- 525-byte logical frames are usually emitted as 2 or 3 UART bursts rather than one UART frame,
- intra-frame gaps cluster around roughly 7-20 ms,
- send-to-receive delay clusters around roughly 13-26 ms, and
- the current connection interval target in demo config is below the documented legal minimum, so the bridge cannot rely on the configured value as proof of actual link cadence.

## Goals / Non-Goals

**Goals:**
- Preserve logical frame boundaries end-to-end across the demo bridge.
- Make oversized frame fragmentation deterministic and independent of ring-buffer wrap position.
- Reassemble internal SLE fragments before UART egress so one logical frame maps to one UART output operation.
- Use negotiated runtime transport limits and legally valid connection settings as the basis for sizing and diagnostics.
- Keep the implementation local to the demo sample and compatible with the current image-refresh workload.

**Non-Goals:**
- Redesigning the SLE stack or controller scheduler.
- Guaranteeing that every logical frame fits in one wireless payload regardless of negotiated limits.
- Replacing the user protocol carried on UART with a different application protocol.
- Solving RF coexistence or peer-controller behavior outside the demo code path.

## Decisions

1. The bridge will operate on logical frames, not raw contiguous ring slices.
Rationale: The observed `[9,516]`, `[213,312]`, and similar splits are artifacts of buffer layout and scheduler timing, not meaningful transport decisions. A frame queue provides stable units for transport and validation.
Alternatives considered:
- Keep byte-stream rings and enlarge buffers: improves loss tolerance but does not remove variable split points.
- Tune only batch sizes and retry delays: may change timing but still leaks internal chunking to UART egress.

2. Internal SLE fragmentation will carry explicit fragment metadata and require receiver-side reassembly before UART egress.
Rationale: Oversized logical frames must sometimes be split over the air, but that split should remain internal to the bridge. Metadata such as frame identifier, fragment index, total fragment count, and total frame length allows deterministic reassembly without depending on UART burst timing.
Alternatives considered:
- Expose raw SLE fragment boundaries to UART: matches the current failure mode and keeps random intra-frame gaps visible to the application.
- Switch to indication-only transport: improves acknowledgement semantics but does not by itself restore end-to-end frame boundaries.

2a. The bridge will treat SLE delivery as complete only after the peer confirms full-frame reassembly.
Rationale: The current notify-only path can still lose a fragment silently even after the sender has advanced its state. A frame-level acknowledgement after successful peer reassembly provides a concrete success point without extending the success boundary all the way to remote UART DMA completion.
Alternatives considered:
- Keep notify and rely only on local API return codes: preserves lower latency but cannot prevent silent frame loss.
- Move business traffic back to `client -> server write_req`: improves reliability but abandons the chosen unified `server -> client` hybrid data path.

3. UART ingress will use an explicit frame extractor instead of assuming callback boundaries are frame boundaries.
Rationale: The demo currently receives framed application traffic, but UART callbacks are triggered by threshold and idle conditions. A frame extractor lets the bridge identify complete logical frames before enqueueing them for transport.
Alternatives considered:
- Idle-only framing: simple fallback, but unstable for sustained high-rate traffic and insufficient for deterministic large-frame handling.
- Fixed-size slicing: easy to implement, but it preserves the current problem under a different rule.

4. UART egress will issue one DMA write per complete logical frame and track completion/error explicitly.
Rationale: The capture shows that a complete logical frame can arrive at the receiver and still be emitted as multiple UART bursts. One DMA submission per complete reassembled frame removes that leak and gives clearer timing diagnostics.
Alternatives considered:
- Continue draining the UART TX ring by contiguous slices: keeps scheduler-driven burst fragmentation in the user-visible output path.

4a. Business fragments will use an indication-confirmed transport path with a separate frame acknowledgement control message.
Rationale: The user-selected success boundary for the next phase is "peer SLE complete reassembly", not "remote UART already transmitted". Indication confirm gives per-fragment protocol feedback, while a small ACK control message gives per-frame application feedback.
Alternatives considered:
- Notify plus custom retry only: duplicates protocol-level reliability work in application space.
- Indication with no frame ACK: confirms each fragment but still cannot distinguish "all fragments confirmed" from "peer fully reassembled the logical frame".

5. Transport sizing will use negotiated runtime limits and valid configured ranges.
Rationale: The current demo hard-codes aggressive payload and interval targets, but the observed traces and SDK headers show that configured values are not sufficient evidence of the actual link cadence. The bridge must size fragments from negotiated runtime limits and request only documented legal connection parameters.
Alternatives considered:
- Continue using compile-time constants as the transport truth: simple but not portable and easy to misread during debugging.

## Risks / Trade-offs

- [Risk] Frame-aware buffering increases memory pressure compared with pure byte-stream rings.
  -> Mitigation: Bound the number and size of queued frames, and expose queue pressure counters.

- [Risk] Reassembly adds latency if fragments are lost or delayed.
  -> Mitigation: Track per-frame age and error counters, and bound reassembly wait behavior.

- [Risk] Indication-based reliable transport will lower peak throughput compared with notify-only fire-and-forget transport.
  -> Mitigation: Start with stop-and-wait semantics for correctness, keep the reliable transport logic isolated, and defer any pipelining optimization until after lossless behavior is confirmed.

- [Risk] Choosing the wrong UART frame extractor could misclassify traffic.
  -> Mitigation: Isolate framing strategy behind a small module and validate it against the captured image-refresh traces before enabling it broadly.

- [Risk] One-DMA-write-per-frame may lower peak throughput for some stream-oriented traffic.
  -> Mitigation: Keep the scope limited to the demo sample and size DMA buffers for the frame lengths actually used by the image-refresh workload.

## Migration Plan

1. Add regression targets and diagnostics from the current capture so before/after behavior is comparable.
2. Introduce logical frame ingress parsing and a bounded frame queue on the UART-to-SLE path.
3. Add deterministic fragment metadata, negotiated-limit sizing, and receiver-side reassembly on the SLE path.
4. Update UART egress to emit one DMA write per complete logical frame.
5. Introduce reliable `server -> client` transport semantics using indication confirm, frame ACK, timeout handling, and duplicate suppression.
6. Re-run the image-refresh capture and compare send-to-receive delay, intra-frame gap, retry behavior, and large-frame split patterns against the baseline.
7. Roll back by disabling reliable frame-aware mode in the demo sample if the new path causes unacceptable regressions.

## Open Questions

- What exact framing rule from the current image-refresh UART protocol should be treated as authoritative for logical frame extraction?
- What memory ceiling is acceptable for queued frames and reassembly state on both roles?
- Should the bridge keep a pure byte-stream compatibility mode alongside the frame-aware mode, or is the image-refresh framing mode sufficient for this demo target?
