#ifndef kalman_h
#define kalman_h

class KF 
{
    protected:
        /* 
           x    state of the system 
           P    
           R    state noise covariance
           Q    measurement noise covariance
           K    filter gain
       */
        float x, P, Q, R, K;

    public:

        float update(float measurement)
        {
            // time update
            P = P + Q; 

            // measurement update
            K = (1 / P ) / (1 / P + R);
            x = x + K * (measurement - x);
            P = (1 - K) / P * (1 - K) + K * R * K;

            return x;
        }

        KF(): 
            x(5), P(100), K(1), R(5), Q(1)
        {
        }
        
};


#endif
