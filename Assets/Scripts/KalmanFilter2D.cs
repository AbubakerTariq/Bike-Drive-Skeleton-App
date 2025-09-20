public class KalmanFilter2D
{
    // State vector [position, velocity]
    private float[] z_st = new float[2];  // [position, velocity]

    // Covariance matrix
    private float[,] P_cov = new float[2, 2];  // Covariance matrix P

    // Kalman Gain
    private float[] K_gain = new float[2];

    public KalmanFilter2D(float ERR_EST_INIT)
    {
        // Initialize state to zero (or some other initial guess)
        z_st[0] = 0;  // Position estimate
        z_st[1] = 0;  // Velocity estimate

        // Initialize covariance matrix P
        P_cov[0, 0] = ERR_EST_INIT; // Position
        P_cov[0, 1] = 0;
        P_cov[1, 0] = 0;
        P_cov[1, 1] = ERR_EST_INIT; // Velocity
    }

    // Function to predict the next state (position, velocity)
    public void Predict(float dt, float[] Q_proc)
    {
        // State transition model for position and velocity (constant velocity model)
        float F_sys_00 = 1, F_sys_01 = dt;
        float F_sys_10 = 0, F_sys_11 = 1;

        // Predict state
        z_st[0] += z_st[1] * dt;  // position = position + velocity * deltaTime
        z_st[1] += 0;  // velocity (constant velocity model)

        // Predict the covariance matrix (P)
        float[,] F = { { F_sys_00, F_sys_01 }, { F_sys_10, F_sys_11 } };

        // P = F * P * F^T + Q (Process Noise)
        float[,] Ftr = { { F_sys_00, F_sys_10 }, { F_sys_01, F_sys_11 } };
        float[,] Q = { { Q_proc[0], 0 }, { 0, Q_proc[1] } };

        float[,] F_P = MultiplyMatrices(F, P_cov);
        float[,] F_P_Ftr = MultiplyMatrices(F_P, Ftr);
        P_cov = AddMatrices(F_P_Ftr, Q);
    }

    // Function to update the filter with new measurement
    public void Update(float pos_m, float R_meas)
    {
        // Measurement model (H)
        float H0 = 1, H1 = 0;

        // Measurement residual (y)
        float y = pos_m - (H0 * z_st[0] + H1 * z_st[1]);

        // Measurement prediction error covariance (S)
        float S = H0 * P_cov[0, 0] * H0 + H1 * P_cov[1, 0] * H1 + R_meas;

        // Kalman Gain (K)
        K_gain[0] = P_cov[0, 0] * H0 / S;
        K_gain[1] = P_cov[1, 0] * H0 / S;

        // Update state estimate
        z_st[0] = z_st[0] + K_gain[0] * y;
        z_st[1] = z_st[1] + K_gain[1] * y;

        // Update covariance matrix
        float[,] I = { { 1, 0 }, { 0, 1 } }; // Identity matrix
        float[,] K_H = { { K_gain[0] * H0, K_gain[0] * H1 }, { K_gain[1] * H0, K_gain[1] * H1 } };

        P_cov = SubtractMatrices(P_cov, MultiplyMatrices(K_H, P_cov));
    }

    // Ancillary functions for matrix operations
    private float[,] MultiplyMatrices(float[,] A, float[,] B)
    {
        int rows_A = A.GetLength(0), cols_A = A.GetLength(1);
        int rows_B = B.GetLength(0), cols_B = B.GetLength(1);

        float[,] product = new float[rows_A, cols_B];

        for (int i = 0; i < rows_A; i++)
        {
            for (int j = 0; j < cols_B; j++)
            {
                product[i, j] = 0;
                for (int k = 0; k < cols_A; k++)
                {
                    product[i, j] += A[i, k] * B[k, j];
                }
            }
        }

        return product;
    }

    private float[,] AddMatrices(float[,] A, float[,] B)
    {
        int rows = A.GetLength(0);
        int cols = A.GetLength(1);

        float[,] sum = new float[rows, cols];

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                sum[i, j] = A[i, j] + B[i, j];
            }
        }

        return sum;
    }

    private float[,] SubtractMatrices(float[,] A, float[,] B)
    {
        int rows = A.GetLength(0);
        int cols = A.GetLength(1);

        float[,] diff = new float[rows, cols];

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                diff[i, j] = A[i, j] - B[i, j];
            }
        }

        return diff;
    }

    // Get the current velocity estimate
    public float GetVelocityEstimate()
    {
        return z_st[1];
    }

    // Get the current position estimate
    public float GetPositionEstimate()
    {
        return z_st[0];
    }
}

