Header header
uint8 levelFlag
uint16 commVersion                  # Old version Aliengo does not have
uint16 robotID                      # Old version Aliengo does not have
uint32 SN                           # Old version Aliengo does not have
uint8 bandWidth                     # Old version Aliengo does not have
uint8 mode
uint8 gaitType                      # 0.idle  1.trot  2.trot running  3.climb stair  4.trot obstacle
IMU imu
MotorState[20] motorState
int16[4] footForce                  # Old version Aliengo is different
int16[4] footForceEst               # Old version Aliengo does not have
float32 progress
float32[3] position                 # (unit: m), from own odometry in inertial frame, usually drift
float32[3] velocity                 # (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
float32 bodyHeight                  # (unit: m, default: 0.28m)
float32 footRaiseHeight             # (unit: m, default: 0.08m), foot up height while walking
float32 yawSpeed                    # (unit: rad/s), rotateSpeed in body frame   
Cartesian[4] footPosition2Body      # foot position relative to body
Cartesian[4] footSpeed2Body         # foot speed relative to body
uint8[40] wirelessRemote
uint32 reserve                      # Old version Aliengo does not have
uint32 crc

# Under are not defined in SDK yet. # Old version Aliengo does not have
Cartesian[4] eeForce            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.
float32[12] jointP              # for visualization