# RobOps Platform Integration Notes

## Open-Source Platforms

### Google Cloud Robotics Core
- **GitHub**: https://github.com/googlecloudrobotics/core
- **Integration**: K8s-native, ROS2 compatible
- **Key APIs**:
  - Deploy containerized workloads on robots and cloud
  - Service discovery via K8s mechanisms
  - Secure comms via cloud IAM
- **MOAI Integration**: Deploy moai-service as K8s workload, edge agent on robots

### Open-RMF
- **GitHub**: https://github.com/open-rmf
- **Integration**: ROS2 messages + fleet adapters
- **Key APIs**:
  - `rmf_task`: Task planning and dispatch
  - `rmf_traffic`: Traffic scheduling
  - `rmf_fleet_adapter`: Fleet integration
- **MOAI Integration**: Subscribe to RMF task/event topics, return decisions as constraints

### free_fleet
- **GitHub**: https://github.com/open-rmf/free_fleet
- **Integration**: ROS2 + Zenoh communication
- **Key APIs**:
  - Zenoh-based robot ↔ fleet adapter comms
  - ROS1/ROS2 message bridging
- **MOAI Integration**: Same as Open-RMF, via fleet adapter

### openTCS
- **GitHub**: https://github.com/openTCS/opentcs
- **Integration**: Web API (HTTP) or Java kernel
- **Key APIs**:
  - `TransportOrderService`: Create/query orders
  - Web API: Preferred for non-Java integrations
  - Order states: RAW → ACTIVE → DISPATCHABLE → BEING_PROCESSED → FINISHED
- **MOAI Integration**: Poll/subscribe via Web API, annotate orders with decisions

### FogROS2
- **GitHub**: https://github.com/BerkeleyAutomation/FogROS2
- **Integration**: ROS2 launch file based cloud deployment
- **Key APIs**:
  - `FogROSLaunchDescription`: Define cloud vs edge nodes
  - VPN-secured communication
- **MOAI Integration**: Deploy moai-service as FogROS2 cloud node
- **Caution**: Ensure secret key stays on trusted edge node

### Eclipse hawkBit
- **GitHub**: https://github.com/eclipse/hawkbit
- **Integration**: REST API + DMF (Device Management Federation)
- **Key APIs**:
  - Software module management
  - Rollout campaigns
  - Device registration
- **MOAI Integration**: Use for OTA updates of edge agent + model configs

### webrtc_ros
- **GitHub**: https://github.com/RobotWebTools/webrtc_ros
- **Integration**: ROS2 + WebRTC for teleop
- **Key APIs**:
  - Video streaming
  - Operator UI integration
- **MOAI Integration**: Overlay decisions on operator video feeds (future)

## Proprietary Platforms (Generic Adapters)

### Formant
- **Docs**: https://formant.io
- **Integration**: REST API + webhooks
- **Event Sources**: Incidents, alerts, telemetry
- **Writeback**: Update incident tags, add annotations

### InOrbit
- **Docs**: https://inorbit.ai
- **Integration**: REST API + webhooks + MQTT
- **Event Sources**: Robot events, tasks
- **Writeback**: Mission annotations, status updates

### AWS IoT RoboRunner
- **Docs**: https://aws.amazon.com/roborunner/
- **Integration**: AWS APIs + Step Functions
- **Event Sources**: Work orders, fleet status
- **Writeback**: Order annotations, dispatch decisions

## Generic Webhook Adapter

All proprietary platforms can use the generic webhook adapter:
- **Inbound**: HTTP webhook receiver (JSON events)
- **Outbound**: HTTP POST callbacks (JSON decisions)
- **Config**: YAML mapping of event fields → RobotOpsEvent
