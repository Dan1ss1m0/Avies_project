#include <iostream>
#include <string>
#include <cmath>

static float old_time = 16813716;
static float old_roll = 145.798221, old_pitch = 0.020531, old_yaw = 115.477508;

float filter(float old_v, float new_v, float a = 0.8)
{
	return a*new_v + (1-a)*old_v;
}

float acc_roll(float x, float y, float z)
{
	return 180*atan2(y, z)/M_PI;
}

float acc_pitch(float x, float y, float z)
{
	return 180*atan2(-x, sqrt(y*y+z*z))/M_PI;
}

float gyro_roll(float x, float y, long time)
{
	return old_roll+(time - old_time)*pow(10, -6)*y;
}

float gyro_pitch(float x, float y, long time)
{
	return old_pitch + (time - old_time)*x*pow(10, -6);
}

float get_roll(float g_x, float g_y, float a_x, float a_y, float a_z, long time, bool filt=false)
{
	float roll = gyro_roll(g_x, g_y, time)*0.9+acc_roll(a_x, a_y, a_z)*0.1;
	if (filt)
		return filter(old_roll, roll);
	return roll;
}

float get_pitch(float g_x, float g_y, float a_x, float a_y, float a_z, long time, bool filt=false)
{
	float pitch = gyro_pitch(g_x, g_y, time)*0.9+acc_pitch(a_x, a_y, a_z)*0.1;
	if (filt)
		return filter(old_pitch, pitch);
	return pitch;
}

float get_yaw(float x, float y, float z, float roll, float pitch, bool filt=false)
{
	float XH = x*cos(pitch)+y*sin(pitch)*sin(roll) + z*sin(pitch)*cos(roll);
	float YH = y*cos(roll)+z*sin(roll);
	float yaw = 180*atan2(-YH, XH)/M_PI;
	if (filt)
		return filter(old_yaw, yaw);
	return yaw;

}

float T = 273+25, R = 8.31, M = 0.029, g = 9.8;
float P0, h0;
float get_hight(float bar)
{
	return (R*T)/(M*g)*log(P0/bar);
}


int main()
{
	float g_x = -0.04, g_y = -0.14, g_z = -0.02, a_x = 0.00, a_y = -0.00, a_z = 1.01, m_x = -42.41, m_y = -18.63, m_z = -185.56, time = 16813716;
	float roll = get_roll(g_x, g_y, a_x, a_y, a_z, time, true);
	float pitch = get_pitch(g_x, g_y, a_x, a_y, a_z, time, true);
	float yaw = get_yaw(m_x, m_y, m_z, roll, pitch, true);
        std::cout << roll << ", " << pitch << ", " << yaw << std::endl;
	old_yaw = yaw;
	old_roll = roll;
	old_pitch = pitch;
	old_time = time;
}
