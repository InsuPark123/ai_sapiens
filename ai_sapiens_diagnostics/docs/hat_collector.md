# Independent H5 fault evidence collector

This opt-in collector queries the H562/W5300 H5 service on UDP5008 with an explicit board ID (K1-22: 200). It is a separate ROS process with one bounded worker for network and disk work; DHI/SDK read/write code, build selection and communication settings are unchanged. It does not start controllers, change motor torque, or issue baud/reset/clear/reboot commands.

## Running as an independent service

Build `ai_sapiens_diagnostics` and source the intended workspace. Start the collector independently before bringup so it can remain alive after bringup exits:

```bash
ROS_DOMAIN_ID=0 ros2 launch ai_sapiens_diagnostics hat_collector.launch.py \
  hat_host:=192.168.200.200 hat_service_id:=200 \
  hat_output:=/workspace/hat-evidence
```

This launch is deliberately not included in normal bringup until firmware and collector 1 kHz comparisons are accepted. The parameter defaults map channels 0..4 to k1_right_arm/bus, k1_left_arm/bus, k1_left_leg/bus, k1_right_leg/bus, k1_imu/bus. The collector validates applied UDP ports 5001..5005 and profile v6 before treating evidence as supported. Do not repoint it at REV06 or an E2D2 device based solely on an IP match.

One-shot read-only export (no RAM freeze or journal selector writes):

```bash
ros2 run ai_sapiens_diagnostics hat_collect_once \
  --host 192.168.200.200 --id 200 --read-only --output /workspace/hat-evidence
```

Legacy FW28 returns identity, profile and journal status and is explicitly marked extended-diagnostics unsupported; the one-shot command returns exit 2 for incomplete extended capture. A legacy profile read uses the existing firmware's profile handler, which polls RX; it is not a zero-cost measurement. Ordinary one-shot mode freezes diagnostic RAM after verifying the extended ABI. It never clears counters or logs.

## Triggers and bounds

Normal query period is 10 s (minimum 5 s); timer/heartbeat use a steady clock. New read-failure onsets, newly increased failures between otherwise successful publications, and startup failure can queue a capture from the existing DHI schema2 `/diagnostics`. Stale/duplicate and retired-session frames are rejected. At most five onset records are pending, one per bus; repeated failed frames do not cause repeated captures. The first fault capture is not delayed by the normal poll interval; subsequent fault captures are separated by 5 s (minimum configurable 1 s). One worker/query can run at a time. Each query has a 3 s network budget, 100 ms transaction timeout and at most 64 source ports; no protocol retry loop runs indefinitely.

A fresh UDP source port for each request remains reserved until collection closes. This prevents a late status packet from satisfying a later equal-length CT read in the same query. CRC, exact packet length, ID, model, profile schema, cookie, frozen generation, MCU boot, channel and port mapping are checked. The frozen RAM view is read in 256-byte chunks. Live fault queries now collect RAM only. Flash journal selection reproduced active IMU failures on K1-22 on2026-10-13, so the live node passes journal_limit=0. Explicit offline export can read up to eight records with CRC and selected-sequence verification; do this only with hardware traffic idle. Deferred records may not yet appear; the result retains the full RAM evidence even if journal readback is incomplete. Disk saving follows the bounded query.

JSON files preserve host UTC and monotonic receipt/capture times, originating DHI session/sequence, H5 boot_id/uptime/event sequence, raw slot hex, applied configuration and journal queue/loss/error status. They do not equate host wall clock with MCU uptime. Initial state, changed evidence and host-triggered captures are saved atomically. Default retention is 128 files, max 256 KiB per file; only `hat-evidence-*.json` in the configured collector directory is rotated, and eviction/error counters are exposed. Use a dedicated directory owned by one collector. RAM evidence and queued Flash are not power-loss guarantees.

## Standard diagnostics and app mapping

The collector publishes `hat_bridge/port/5001` through `5005` to `/diagnostics`, with a separate HATBridge GenericAnalyzer section in `k1_aggregator.yaml`. The health adapter maps these to `dhi/hat_bridge/port/*` components with type `hat_bridge_port`, retaining the existing history, evidence, freshness and `/ai_sapiens/health` contract. They are additive, optional components rather than new expected motor IDs. They do not replace the existing DHI bus/motor diagnostic states.

Messages distinguish query failure/unsupported firmware, captured failure evidence, bidirectional progress observed, and evidence expired. Stage observation strings explicitly say “in capture interval.” Counters do not identify a DXL response boundary, prove host receipt, or locate the physical fault. A fresh first snapshot remains history even after progress resumes. File-save failures are exposed through capture_drops and an empty evidence-file path; callers should inspect this separately from bridge traffic state.

The collector publishes heartbeat receipt age rather than pretending to acquire a new MCU sample each heartbeat. On a failed query it emits UNKNOWN/MISSING instead of reusing the last good reading. If a worker stalls, age expiry also changes the raw DiagnosticStatus to STALE; stopping the collector lets aggregator and health publisher expiry take effect.

The local health contract is connected and tested. The deployed K1 status web/client may use its own older adapter or a fixed component list: deployment must verify that all five HAT port components, evidence paths and stale states are actually displayed. Publishing a new DiagnosticArray alone does not establish web integration. K1-22 deployment on 2026-10-13 uses an independent Docker collector and a dedicated HAT file-backed panel in the existing status web. Existing bringup retains ownership of health_node and the standard aggregator. The HAT panel remains available without either process or the ROS router. It does not add HAT ports to the expected motor inventory.

## Explicit remaining checks

K1-22 currently reports ID200 whereas the operational SDK bridge default is ID1. The collector's explicit `service_id` resolves its own control-service access. It does not change the SDK default or its `setBaudRate()` behavior, which still reports local-timing success after remote-write failure. A subsequent SDK change should use a per-board service-ID setting and report remote-applied versus local-only outcomes while preserving E2D2 ID1 compatibility; do not silently change all devices to ID200. This is not established as the cause of the dance failure.

The deployed FW28 source commit/build identity is not recoverable from its version byte. The review manifest records it as unknown and records the exact local candidate source hashes/ELF/bin hashes separately. K1-22 was flashed with verified app CRC/size on 2026-10-13; the exact image SHA256 is c6cbd93d81b1dc6b5025c8db05c448c31e79becd1b2fa0b8e907ccc9f87f3dc0. Original-image readback and a matching rollback binary remain unavailable.

Firmware instrumentation, journal writes and collector traffic require same-condition physical 1 kHz comparison. Report raw start interval mean/p99/max/jitter, total execution, per-bus read/write, >1 ms counts and excess, timeout/fallback, split by startup/normal/fault-logging/recovery. Neither unit tests, localhost ROS tests, image sizes nor an average 1000 Hz complete this acceptance step. A regression requires investigation or rollback, not relaxed criteria. Motor tests and flashing require current operational state and explicit approval.

## K1-22 deployed operation (2026-10-13)

`k1-hat-diagnostics` uses `/data/k1-hat-monitoring/compose.yaml`, explicit H5 ID200, poll10s/cooldown5s, and host directory `/data/ai_sapiens/docker/workspace/hat-evidence`. `latest.json` is atomically updated every query; immutable history remains bounded at128 files. Compact ROS status includes stage counters, while full raw snapshots remain in JSON. Do not run a second writer against this directory.

The status URL is http://k1-snpr50a1022.local:8088/. The dedicated HAT panel distinguishes query availability, query failure and expired evidence, and links saved JSON through `/api/hat-evidence`. Host triggers and H5 first snapshots are preserved separately. Overall web receipt can be driven by collector traffic; inspect DHI component freshness independently.

`ZENOH_ROUTER_CHECK_ATTEMPTS=-1` lets the collector initialize without a router; 0 means wait indefinitely. This was corrected during deployment and router-down file collection was verified. See the primary [rmw_zenoh configuration documentation](https://github.com/ros2/rmw_zenoh). The ROS router is still needed for DHI-trigger receipt and standard ROS forwarding.

For read-only export run `python3 /data/k1-hat-monitoring/save-evidence.py` on the Orin host. It combines HAT JSON, a SQLite backup, startup journals and bounded process logs under `/data/ai_sapiens/docker/workspace/hat-incident-bundles`. No log clear, torque action or reboot is issued.

Deployment checks passed for real Ethernet ABI1/5ports6Mbps, router-down collection, raw→health adapter and standard aggregator on ARM64, persistent files after collector restart and the actual web panel. Physical fault injection, motion reproduction, power-loss durability and same-condition real1kHz comparisons remain unverified.

### 2026-10-13 live journal interference correction

A43.5s router/web readback interval after stopping the collector showed43,446 additional IMU reads and zero added failures. A RAM-only capture added zero failures; two one-record journal queries added2 and3 failures. The existing H5 journal selector executes a synchronous record/CRC scan outside the ServiceSock timing probe. The live collector now requests no Flash record selection, including on host failures. RAM snapshots, raw hex, journal status counters and host trigger metadata are preserved. Saved pre-change first snapshots and journal exports are archived on the PC. This mitigates demonstrated query interference; the initiating four-bus failure and full real1kHz acceptance remain open.

The collector keeps up to eight preceding RAM captures in a worker-owned bounded deque and attaches them to saved/latest evidence. Each preceding capture contains only time, completeness, errors and live slots, so histories do not recursively contain earlier histories. This requires no additional H5 transaction and provides pre-onset traffic counters for the next controlled run.
