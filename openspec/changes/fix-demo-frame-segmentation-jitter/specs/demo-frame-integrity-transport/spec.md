## ADDED Requirements

### Requirement: Logical frames are preserved end-to-end
The demo bridge SHALL detect logical UART frames at ingress and SHALL emit exactly one complete copy of each logical frame at peer UART egress, even when internal SLE transport fragmentation occurs.

#### Scenario: Control frame crosses the bridge
- **WHEN** a complete logical control frame enters the bridge
- **THEN** the bridge SHALL preserve its byte order and frame boundary across transport
- **THEN** the peer UART SHALL emit that control frame once, not as multiple UART bursts

#### Scenario: 525-byte image-refresh frame crosses the bridge
- **WHEN** a 525-byte logical frame enters the bridge and transport limits require one or more internal fragments
- **THEN** the receiver SHALL reassemble the fragments before UART transmission
- **THEN** the peer UART SHALL emit one 525-byte logical frame rather than 2 or 3 UART bursts

### Requirement: Oversized frames fragment deterministically
The demo bridge SHALL fragment any logical frame larger than the effective SLE payload using a fixed configured fragment payload size for every non-final fragment, and SHALL attach metadata sufficient to reassemble the original frame without using UART callback timing.

#### Scenario: 2151-byte frame is segmented
- **WHEN** a 2151-byte logical frame exceeds the effective SLE payload limit
- **THEN** every non-final fragment SHALL use the same payload size
- **THEN** only the final fragment MAY be shorter
- **THEN** fragment sizes SHALL NOT depend on ring-buffer wrap position or contiguous-slice length

### Requirement: Transport limits and delay metrics are observable
The demo bridge SHALL expose the effective payload limit, requested versus negotiated connection parameters, and per-frame delay metrics needed to validate image-refresh transport behavior after the change.

#### Scenario: Image-refresh regression capture is reviewed
- **WHEN** image-refresh traffic is captured after implementation
- **THEN** the demo SHALL report the effective payload limit used for fragmentation
- **THEN** the demo SHALL report the requested and negotiated connection parameters for that run
- **THEN** the resulting trace and logs SHALL allow validation of send-to-receive delay and intra-frame stall against the defined regression targets

### Requirement: Configured transport tuning uses valid ranges
The demo bridge SHALL reject or clamp connection-related configuration values that fall outside the documented SDK limits before applying them to the SLE stack.

#### Scenario: Configured connection interval is below the documented minimum
- **WHEN** the configured connection interval falls below the documented legal range
- **THEN** the demo SHALL reject or clamp that value before requesting the update
- **THEN** the logs SHALL show the effective requested value and the negotiated value returned by the stack
