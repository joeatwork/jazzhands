//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

// JRB: Altered slightly (and uncomprehendingly) by JRB, see link above for
// JRB: original sources.

#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

struct MadgwickState {
  float sampleFreq; // sample frequency in Hz
  float beta;
  float q0;
  float q1;
  float q2;
  float q3;
};

//---------------------------------------------------------------------------------------------------

void MadgwickAHRSupdate(struct MadgwickState *state, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(struct MadgwickState *state, float gx, float gy, float gz, float ax, float ay, float az);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
