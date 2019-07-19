
enum cart { X,Y,Z };

#define GYRO_SCALE ((1998 * PI)/(32767.0f * 180.0f * 1000.0f)) // MPU6050
#define F_GYRO_SCALE 0.001 * GYRO_SCALE // in usec

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#define GYR_CMPF_FACTOR 90
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))

typedef struct fp_vector 
{		
  float X,Y,Z;		
} t_fp_vector_def;

typedef union 
{		
  float A[3];		
  t_fp_vector_def V;		
} t_fp_vector;


float InvSqrt (float x)
{ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) 
{
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}

void normalizeV(struct fp_vector *vec)
{
    float length;
    length = sqrtf(vec->X * vec->X + vec->Y * vec->Y + vec->Z * vec->Z);
    if (length != 0) 
    {
        vec->X /= length;
        vec->Y /= length;
        vec->Z /= length;
    }
}

static t_fp_vector EstG = { 0.0, 0.0, (float)ACCRESO }; // x,y,z
static t_fp_vector EstM = { 0.0, 1.0, 0.0 }; // x,y,z
static float accData[3]  = {0,0,0};

uint32_t  tnow; 

void getEstimatedAttitude()
{
  uint8_t axis;
  float deltaGyroAngle[3];
  uint32_t  tdiff; 
  float scale; 

  tdiff = micros() - tnow;
  tnow = micros();
  scale = (float)tdiff * F_GYRO_SCALE; 
  
  // Initialization
  for (axis = 0; axis < 3; axis++) 
  {
    deltaGyroAngle[axis] = (float)gyroADC[axis] * scale;
    accData[axis]  = (float)accADC[axis];
    gyroData[axis] = gyroADC[axis];
  }

  // AZ, EL
  rotateV(&EstG.V,deltaGyroAngle); 
  
  // Apply complimentary filter (Gyro drift correction)
  for (axis = 0; axis < 3; axis++) EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + accData[axis]) * INV_GYR_CMPF_FACTOR;
  
  // Attitude of the estimated vector
  float sqGX_sqGZ = sq(EstG.V.X) + sq(EstG.V.Z);
  float invmagXZ  = InvSqrt(sqGX_sqGZ);
  anglerad[ROLL]  = atan2(EstG.V.X , EstG.V.Z);
  anglerad[PITCH] = atan2(EstG.V.Y , invmagXZ*sqGX_sqGZ);
  angle[ROLL]  = 572.95f * anglerad[ROLL];
  angle[PITCH] = 572.95f * anglerad[PITCH];

  // Yaw
  rotateV(&EstM.V, deltaGyroAngle);
  #if MAG
    for (axis = 0; axis < 3; axis++)
      EstM.A[axis]  += (imu.magADC[axis] - EstM.A[2*axis+1])<<(16-GYR_CMPFM_FACTOR);
  #else
    normalizeV(&EstM.V);
  #endif

  float invG = InvSqrt(sqGX_sqGZ + EstG.V.Y * EstG.V.Y);
  anglerad[YAW] = atan2(EstM.V.Z * EstG.V.X - EstM.V.X * EstG.V.Z,
                        (EstM.V.Y * sqGX_sqGZ - (EstM.V.X * EstG.V.X + EstM.V.Z * EstG.V.Z) * EstG.V.Y)*invG );
  angle[YAW] = 572.95f * anglerad[YAW];

}
