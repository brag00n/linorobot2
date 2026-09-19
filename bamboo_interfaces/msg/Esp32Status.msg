# Telemetrie brute agregee de la carte ESP32 (diagnostic), reprise du snapshot() hote.
# Les grandeurs "propres" (odom, imu, batterie) sont publiees a part en messages standard ;
# ce message sert au debug et a l'observation directe via le MCP.
std_msgs/Header header

# Batterie (SYS_STATUS #1)
float32 battery_voltage   # V

# Attitude (ATTITUDE #30), DEGRES (reflet brut du snapshot hote ; imu/data la publie en rad)
float32 roll
float32 pitch
float32 yaw

# Vitesses chassis remontees par la carte (BAMBOO_WHEEL_STATE 42001), SI
float32 vx                # m/s
float32 vy                # m/s
float32 vz                # rad/s (lacet ; wz cote firmware)

# Comptage encodeur cumulatif par moteur (BAMBOO_ENCODERS 42003), M1..M4
int32[] encoders
