//
// Collision Warning Message Definition
// VANET Collision Warning & Traffic Management - Phase 2
//

namespace five_vehicle_network;

message CollisionWarningMsg
{
    int senderId;           // Original sender vehicle ID (e.g. 1 for V1)
    int targetId;           // Target destination vehicle ID (e.g. 5 for V5, or -1 for broadcast)
    int hopCount = 0;       // Multi-hop counter, incremented at each forwarder
    string warningType;     // "EMERGENCY_BRAKE", "FORWARD_COLLISION_WARNING"
    double eventTime;       // Timestamp when incident occurred in seconds
    double posX;            // Longitudinal position X of alerting vehicle (meters)
    double posY;            // Lateral position Y (meters)
    double speed;           // Vehicle speed at time of warning (m/s)
}
