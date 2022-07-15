## definition of each leg and joint
constexpr int FR_ = 0;       // leg index
constexpr int FL_ = 1;
constexpr int RR_ = 2;
constexpr int RL_ = 3;

constexpr int FR_0 = 0;      // joint index
constexpr int FR_1 = 1;      
constexpr int FR_2 = 2;

constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;

constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;

constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;

---
## Highstate

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

Under are not defined in SDK yet. # Old version Aliengo does not have
Cartesian[4] eeForce            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.
float32[12] jointP              # for visualization


## imu
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
float32[3] rpy
int8 temperature

## motorstate
uint8 mode           # motor current mode 
float32 q            # motor current position（rad）
float32 dq           # motor current speed（rad/s）
float32 ddq          # motor current speed（rad/s）
float32 tauEst       # current estimated output torque（N*m）
float32 q_raw        # motor current position（rad）
float32 dq_raw       # motor current speed（rad/s）
float32 ddq_raw      # motor current speed（rad/s）
int8 temperature     # motor temperature（slow conduction of temperature leads to lag）     ##  Over 60C, the motor will be force to shut
uint32[2] reserve


## HighCmd
uint8 levelFlag
uint16 commVersion              # Old version Aliengo does not have
uint16 robotID                  # Old version Aliengo does not have
uint32 SN                       # Old version Aliengo does not have
uint8 bandWidth                 # Old version Aliengo does not have
uint8 mode                      # 0. idle, default stand 
                                # 1. force stand (controlled by dBodyHeight + ypr)
                                # 2. target velocity walking (controlled by velocity + yawSpeed)
                                # 3. target position walking (controlled by position + ypr[0])
                                # 4. path mode walking (reserve for future release)
                                # 5. position stand down. 
                                # 6. position stand up 
                                # 7. damping mode 
                                # 8. recovery stand
                                # 9. backflip
                                # 10. jumpYaw
                                # 11. straightHand
                                # 12. dance1
                                # 13. dance2
uint8 gaitType                  # 0.idle  1.trot  2.trot running  3.climb stair
uint8 speedLevel                # 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
float32 footRaiseHeight         # (unit: m, default: 0.08m), foot up height while walking
float32 bodyHeight              # (unit: m, default: 0.28m),
float32[2] postion              # (unit: m), desired position in inertial frame
float32[2] velocity             # (unit: m/s), forwardSpeed, sideSpeed in body frame
float32[3] euler                # (unit: rad), roll pitch yaw in stand mode
float32 yawSpeed	            # (unit: rad/s), rotateSpeed in body frame
LED[4] led
uint8[40] wirelessRemote
uint8[40] AppRemote             # Old version Aliengo does not have
uint32 reserve                  # Old version Aliengo does not have
int32 crc



##  TO DO 

1. calibrate the footForce sensor regularly
2. add overtemp protection of motor
3. add highcmd topic 