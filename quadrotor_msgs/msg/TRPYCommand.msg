# Collective thrust and body-rate reference for the AcroMode rate loop.
std_msgs/Header header
float64 thrust                               # N, total, along body z
geometry_msgs/Vector3 angular_velocity       # rad/s, body frame