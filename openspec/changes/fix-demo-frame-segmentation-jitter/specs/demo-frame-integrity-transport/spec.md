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

### Requirement: Server-to-client transport is reliable up to peer SLE reassembly
The demo bridge SHALL NOT consider a logical frame delivered merely because a local send API returned success. A frame SHALL be considered delivered only after the peer confirms successful full-frame reassembly.

#### Scenario: Fragment indication succeeds but frame is not yet acknowledged
- **WHEN** one or more internal wireless fragments for a logical frame have been accepted locally for transmission
- **THEN** the sender SHALL retain the logical frame in its reliable transmit state
- **THEN** the sender SHALL NOT release that frame until a peer acknowledgement for the completed logical frame is received

#### Scenario: Peer fully reassembles a frame
- **WHEN** the receiver completes reassembly of a logical frame
- **THEN** it SHALL emit a frame acknowledgement back to the sender
- **THEN** the sender SHALL treat receipt of that acknowledgement as the success point for SLE delivery

### Requirement: Transport retries and duplicate suppression are explicit
The demo bridge SHALL retry incomplete server-to-client delivery after indication or acknowledgement failure, and SHALL suppress duplicate logical frame delivery at the receiver.

#### Scenario: Indication confirm or frame acknowledgement is missing
- **WHEN** the sender does not receive the expected fragment confirm or frame acknowledgement within the configured timeout
- **THEN** it SHALL retry the logical frame according to the configured retry budget
- **THEN** it SHALL log retry and hard-failure events instead of silently dropping the frame

#### Scenario: Receiver observes a retransmitted completed frame
- **WHEN** the receiver gets a duplicate logical frame that it has already fully reassembled and acknowledged
- **THEN** it SHALL NOT enqueue that logical frame to UART a second time
- **THEN** it SHALL re-send the frame acknowledgement so the sender can converge
